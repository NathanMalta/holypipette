# coding=utf-8
"""
A class to handle a manipulator unit with coordinates calibrated to the reference system of a camera.
It contains methods to calibrate the unit.

Should messages be issued?
Also ranges should be taken into account

Should this be in devices/ ? Maybe in a separate calibration folder
"""
from __future__ import print_function
from __future__ import absolute_import
from typing import List
from .manipulatorunit import *
from numpy import (array, zeros, dot, arange, vstack, sign, pi, arcsin,
                   mean, std, isnan)
import cv2
import numpy as np
import time
import math
from holypipette.devices.manipulator import *

from numpy.linalg import inv, pinv, norm
from holypipette.vision import *
from threading import Thread
from .StageCalHelper import FocusHelper, StageCalHelper
from .PipetteCalHelper import PipetteCalHelper

__all__ = ['CalibratedUnit', 'CalibrationError', 'CalibratedStage']

verbose = True

##### Calibration parameters #####
from holypipette.config import Config, NumberWithUnit, Number, Boolean


class CalibrationConfig(Config):
    position_update = NumberWithUnit(1000, unit='ms',
                                     doc='dt for updating displayed pos.',
                                     bounds=(0, 10000))
    
    autofocus_dist = NumberWithUnit(500, unit='um',
                                     doc='z dist to scan for autofocusing.',
                                     bounds=(100, 5000))
    
    stage_diag_move = NumberWithUnit(500, unit='um',
                                     doc='x, y dist to move for stage cal.',
                                     bounds=(0, 10000))
    
    pipette_diag_move = NumberWithUnit(2500, unit='um',
                                     doc='x, y dist to move for pipette cal.',
                                     bounds=(100, 10000))
    

    categories = [('Stage Calibration', ['autofocus_dist', 'stage_diag_move']),
                  ('Pipette Calibration', ['pipette_diag_move']),
                  ('Display', ['position_update'])]


class CalibrationError(Exception):
    def __init__(self, message='Device is not calibrated'):
        self.message = message

    def __str__(self):
        return self.message


# class Objective(object):
#     '''
#     An objective is defined by a magnification factor (4, 20, 40x),
#     an offset for the focal plane, and a conversion factor from um to px
#     (which is camera-dependent).
#     '''
#     def __init__(self, magnification, factor, offset):
#         self.magnification = magnification
#         self.factor = factor
#         self.offset = offset

class CalibratedUnit(ManipulatorUnit):
    def __init__(self, unit, stage=None, microscope=None, camera=None,
                 config=None):
        '''
        A manipulator unit calibrated to a fixed reference coordinate system.
        The stage refers to a platform on which the unit is mounted, which can
        be None.

        Parameters
        ----------
        unit : ManipulatorUnit for the (XYZ) unit
        stage : CalibratedUnit for the stage
        microscope : ManipulatorUnit for the microscope (single axis)
        camera : a camera, ie, object with a snap() method (optional, for visual calibration)
        '''
        ManipulatorUnit.__init__(self, unit.dev, unit.axes)
        self.saved_state_question = ('Move manipulator and stage back to '
                                     'initial position?')
        if config is None:
            config = CalibrationConfig(name='Calibration config')
        self.config = config
        if stage is None: # In this case we assume the unit is on a fixed element.
            self.stage = FixedStage()
            self.fixed = True
        else:
            self.stage = stage
            self.fixed = False
        self.microscope = microscope
        self.camera = camera

        self.calibrated = False
        self.up_direction = [-1 for _ in range(len(unit.axes))] # Default up direction, determined during calibration

        self.pipette_position = None
        self.photos = None
        self.photo_x0 = None
        self.photo_y0 = None

        # Matrices for passing to the camera/microscope system
        self.M = zeros((3,len(unit.axes))) # stage units (in micron) to camera
        self.Minv = zeros((len(unit.axes),3)) # Inverse of M
        self.r0 = zeros(3) # Offset in reference system

        #setup pipette calibration helper class
        self.pipetteCalHelper = PipetteCalHelper(unit, camera)

    def save_state(self):
        if self.stage is not None:
            self.stage.save_state()
        if self.microscope is not None:
            self.microscope.save_state()
        self.saved_state = self.position()

    def delete_state(self):
        if self.stage is not None:
            self.stage.delete_state()
        if self.microscope is not None:
            self.microscope.delete_state()
        self.saved_state = None

    def recover_state(self):
        if self.stage is not None:
            self.stage.recover_state()
        if self.microscope is not None:
            self.microscope.recover_state()
        self.absolute_move(self.saved_state)

    def reference_position(self):
        '''
        Position in the reference camera system.

        Returns
        -------
        The current position in um as an XYZ vector.
        '''
        if not self.calibrated:
            raise CalibrationError
        u = self.position() # position vector in manipulator unit system

        return dot(self.M, u) + self.r0 + self.stage.reference_position()

    def reference_move_not_X(self, r, safe = False):
        '''
        Moves the unit to position r in reference camera system, without moving the stage,
        but without moving the X axis (so this can be done last).

        Parameters
        ----------
        r : XYZ position vector in um
        safe : if True, moves the Z axis first or last, so as to avoid touching the coverslip
        '''
        if not self.calibrated:
            raise CalibrationError
        u = dot(self.Minv, r-self.stage.reference_position()-self.r0)
        u[0] = self.position(axis=0)
        self.absolute_move(u)

    def reference_move_not_Z(self, r, safe = False):
        '''
        Moves the unit to position r in reference camera system, without moving the stage,
        but without moving the Z axis (so this can be done last).

        Parameters
        ----------
        r : XYZ position vector in um
        safe : if True, moves the Z axis first or last, so as to avoid touching the coverslip
        '''
        if not self.calibrated:
            raise CalibrationError
        u = dot(self.Minv, r-self.stage.reference_position()-self.r0)
        u[0] = self.position(axis=2)
        self.absolute_move(u)

    def reference_move(self, r, safe = False):
        '''
        Moves the unit to position r in reference camera system, without moving the stage.

        Parameters
        ----------
        r : XYZ position vector in um
        safe : if True, moves the Z axis first or last, so as to avoid touching the coverslip
        '''

        if np.isnan(np.array(r)).any():
            raise RuntimeError("can not move to nan location.")
        
        if not self.calibrated:
            raise CalibrationError
        u = dot(self.Minv, r-self.stage.reference_position()-self.r0)
        if safe:
            z0 = self.position(axis=2)
            z = u[2]
            if (z-z0)*self.up_direction[2]>0: # going up
                # Go up first
                self.absolute_move(z,axis=2)
                self.wait_until_still(2)
                self.absolute_move(u)
            else: # going down
                # Go down first
                uprime = u.copy()
                u[2] = z0
                self.absolute_move(uprime)
                self.wait_until_still()
                self.absolute_move(z,axis=2)
        else:
            self.absolute_move(u)

    def reference_relative_move(self, r):
        '''
        Moves the unit by vector r in reference camera system, without moving the stage.

        Parameters
        ----------
        r : XYZ position vector in um
        '''
        if not self.calibrated:
            raise CalibrationError
        u = dot(self.Minv, r)
        self.relative_move(u)

    def withdraw(self):
        '''
        Withdraw the pipette to the upper end position
        '''
        if self.up_direction[0]>0:
            position = self.max[0]
        else:
            position = self.min[0]
        self.absolute_move(position, axis=0)

    def focus(self):
        '''
        Move the microscope so as to put the pipette tip in focus
        '''
        self.microscope.absolute_move(self.reference_position()[2])
        self.microscope.wait_until_still()

    def safe_move(self, r, withdraw = 0., recalibrate = False):
        '''
        Moves the device to position x (an XYZ vector) in a way that minimizes
        interaction with tissue.

        If the movement is down, the manipulator is first moved horizontally,
        then along the pipette axis.
        If the movement is up, a direct move is done.

        Parameters
        ----------
        r : target position in um, an (X,Y,Z) vector
        withdraw : in um; if not 0, the pipette is withdrawn by this value from the target position x
        recalibrate : if True, pipette is recalibrated 1 mm before its target
        '''
        if not self.calibrated:
            raise CalibrationError

        # Calculate length of the move
        length = norm(dot(self.Minv,r-self.reference_position()))

        p = self.M[:,0] # this is the vector for the first manipulator axis
        uprime = self.reference_position() # I should call this uprime but rprime

        # First we check whether movement is up or down
        # if (r[2] - uprime[2])*self.microscope.up_direction<0:
        #     # Movement is down
        #     # First, we determine the intersection between the line going through x
        #     # with direction corresponding to the manipulator first axis.
        #     alpha = (uprime - r)[2] / self.M[2,0]
        #     # TODO: check whether the intermediate move is accessible

        #     print(alpha, uprime, r, self.M[2,0], p)

        #     # Intermediate move
        #     self.reference_move(r + alpha * p, safe = True)
        #     # We need to wait here!
        #     self.wait_until_still()

        # # Recalibrate 100 um before target; only if distance is greater than 500 um
        # if recalibrate & (length>500):
        #     self.reference_move(r + 50 * p * self.up_direction[0],safe=True)
        #     self.wait_until_still()
        #     z0 = self.microscope.position()
        #     self.focus()
        #     self.auto_recalibrate(center=False)
        #     self.microscope.absolute_move(z0)
        #     self.microscope.wait_until_still()

        # Final move
        self.reference_move(r + withdraw * p * self.up_direction[0], safe = True) # Or relative move in manipulator coordinates, first axis (faster)


    def pixel_per_um(self, M=None):
        '''
        Returns the objective magnification in pixel per um, calculated for each manipulator axis.
        '''
        if M is None:
            M = self.M
        p = []
        for axis in range(len(self.axes)):
            # The third axis is in um, the first two in pixels, hence the odd formula
            p.append(((M[0,axis]**2 + M[1,axis]**2)/(1-M[2,axis]**2))**.5)
        return p


    def analyze_calibration(self):
        '''
        Analyzes calibration matrices.
        '''
        # Objective magnification
        print("Magnification for each axis of the pipette: "+str(self.pixel_per_um()[:2]))
        pixel_per_um = self.pixel_per_um(M=self.M)[0]
        print("Magnification for each axis of the stage: "+str(pixel_per_um))
        print("Field size: "+str(self.camera.width/pixel_per_um)+" µm x "+str(self.camera.height/pixel_per_um)+' µm')
        # Pipette vs. stage (for each axis, mvt should correspond to 1 um)
        for axis in range(len(self.axes)):
            compensating_move = -dot(self.Minv,self.M[:,axis])
            length = (sum(compensating_move[:2]**2)+self.M[2,axis]**2)**.5
            print("Precision of axis "+str(axis)+": "+str(abs(1-length)))
            # Angles
            angle = abs(180/pi * arcsin(self.M[2,axis] / length))
            print('Angle of axis '+str(axis)+": "+str(angle))


    def calculate_up_directions(self, M): #TODO: does this work?
        '''
        Calculates up directions for all axes and microscope from the matrix.
        '''
        # Determine up direction for the first axis (assumed to be the main axis)
        positive_move = 1*M[:, 0] # move of 1 um along first axis
        self.debug('Positive move: {}'.format(positive_move))
        self.up_direction[0] = up_direction(self.pipette_position, positive_move)
        self.info('Axis 0 up direction: ' + str(self.up_direction[0]))

        # Determine microscope up direction
        if self.microscope.up_direction is None:
            self.microscope.up_direction = sign(M[2, 0])
        self.info('Microscope up direction: ' + str(self.microscope.up_direction))

        # Determine up direction of other axes
        for axis in range(1,len(self.axes)):
            # We use microscope up direction
            s = sign(M[2, axis] * self.microscope.up_direction)
            if s != 0:
                self.up_direction[axis] = s
            self.info('Axis ' + str(axis) + ' up direction: ' + str(self.up_direction[0]))

    def calibrate(self): #TODO: Z-Axis calibration?
        '''
        Automatic calibration.
        Starts without moving the stage, then moves the stage (unless it is fixed).
        '''
        
        mat = self.pipetteCalHelper.calibrate(dist=self.config.pipette_diag_move)
        print(f"orig mat {mat}\n\n")
        M = np.eye(3,3)
        M[0:2,0:2] = mat[:, 0:2]

        if not isnan(M).any():
            # *** Compute the (pseudo-)inverse ***
            Minv = pinv(M)

            # Store the new values
            self.M = M
            self.Minv = Minv
            self.r0 = np.array([-self.camera.width / 2, -self.camera.height / 2, 0])
            self.calibrated = True
            print(f"results: {self.M} {self.Minv} {self.r0}")
        else:
            raise CalibrationError('Matrix contains NaN values')

    def save_configuration(self):
        '''
        Outputs configuration in a dictionary.
        '''
        config = {'up_direction' : self.up_direction,
                  'M' : self.M,
                  'r0' : self.r0,
                  'pipette_position' : self.pipette_position,
                  'photos' : self.photos,
                  'photo_x0' : self.photo_x0,
                  'photo_y0' : self.photo_y0,
                  'min' : self.min,
                  'max' : self.max}

        return config

    def load_configuration(self, config):
        '''
        Loads configuration from dictionary config.
        Variables not present in the dictionary are untouched.
        '''
        self.up_direction = config.get('up_direction', self.up_direction)
        self.M = config.get('M', self.M)
        if 'M' in config:
            self.Minv = pinv(self.M)
            self.calibrated = True
        self.r0 = config.get('r0', self.r0)
        self.pipette_position = config.get('pipette_position', self.pipette_position)
        self.photos = config.get('photos', self.photos)
        self.photo_x0 = config.get('photo_x0', self.photo_x0)
        self.photo_y0 = config.get('photo_y0', self.photo_y0)
        #self.min = config.get('min', self.min)
        #self.max = config.get('max', self.max)

class CalibratedStage(CalibratedUnit):
    '''
    A horizontal stage calibrated to a fixed reference coordinate system.
    The optional stage refers to a platform on which the unit is mounted, which can
    be None.
    The stage is assumed to be parallel to the focal plane (no autofocus needed)

    Parameters
    ----------
    unit : ManipulatorUnit for this stage
    stage : CalibratedUnit for a stage on which this stage might be mounted
    microscope : ManipulatorUnit for the microscope (single axis)
    camera : a camera, ie, object with a ``snap()`` method (optional, for visual calibration)
    '''
    def __init__(self, unit, stage=None, microscope=None, camera=None,
                 config=None):
        CalibratedUnit.__init__(self, unit, stage, microscope, camera,
                                config=config)
        self.saved_state_question = 'Move stage back to initial position?'

        self.focusHelper = FocusHelper(microscope, camera)
        self.stageCalHelper = StageCalHelper(unit, camera)

        # It should be an XY stage, ie, two axes
        if len(self.axes) != 2:
            raise CalibrationError('The unit should have exactly two axes for horizontal calibration.')

    def reference_move(self, r):
        if len(r)==2: # Third coordinate is actually not useful
            r3D = zeros(3)
            r3D[:2] = r
        else:
            r3D = r
        CalibratedUnit.reference_move(self, r3D) # Third coordinate is ignored

    def reference_relative_move(self, r):
        if len(r)==2: # Third coordinate is actually not useful
            r3D = zeros(3)
            r3D[:2] = r
        else:
            r3D = r
        CalibratedUnit.reference_relative_move(self, r3D) # Third coordinate is ignored

    def calibrate(self):
        '''
        Automatic calibration for a horizontal XY stage

        '''
        if not self.stage.calibrated:
            self.stage.calibrate()

        self.info('Preparing stage calibration')
        self.info("auto focusing microscope...")
        self.focusHelper.autofocus(dist=self.config.autofocus_dist)
        self.info("Finished focusing.")

        # use LK optical flow to determine transformation matrix
        self.M = self.stageCalHelper.calibrate(dist=self.config.stage_diag_move).T
        self.Minv = pinv(self.M)
        self.calibrated = True
        

        self.info('Stage calibration done')
        # self.analyze_calibration()

    def mosaic(self, width = None, height = None):
        '''
        Takes a photo mosaic. Current position corresponds to
        the top left corner of the collated image.
        Stops when the unit's position is out of range, unless
        width and height are specified.

        Parameters
        ----------
        width : total width in pixel (optional)
        height : total height in pixel (optional)

        Returns
        -------
        A large image of the mosaic.
        '''
        u0=self.position()

        dx, dy = self.camera.width, self.camera.height
        # Number of moves in each direction
        nx = 1+int(width/dx)
        ny = 1+int(height/dy)
        # Big image
        big_image = zeros((ny*dy,nx*dx))

        column = 0
        xdirection = 1 # moving direction along x axis

        try:
            for row in range(ny):
                img = self.camera.snap()
                big_image[row*dy:(row+1)*dy, column*dx:(column+1)*dx] = img
                for _ in range(1,nx):
                    column+=xdirection
                    self.reference_relative_move([-dx*xdirection,0,0]) # sign: it's a compensatory move
                    self.wait_until_still()
                    self.sleep(0.1)
                    img = self.camera.snap()
                    big_image[row * dy:(row + 1) * dy, column * dx:(column + 1) * dx] = img
                if row<ny-1:
                    xdirection = -xdirection
                    self.reference_relative_move([0,-dy,0])
                    self.wait_until_still()
        finally: # move back to initial position
            self.absolute_move(u0)

        return big_image

class FixedStage(CalibratedUnit):
    '''
    A stage that cannot move. This is used to simplify the code.
    '''
    def __init__(self):
        self.stage = None
        self.microscope = None
        self.r = array([0.,0.,0.]) # position in reference system
        self.u = array([0.,0.]) # position in stage system
        self.calibrated = True

    def position(self):
        return self.u

    def reference_position(self):
        return self.r

    def reference_move(self, r):
        # The fixed stage cannot move: maybe raise an error?
        pass

    def absolute_move(self, x, axis = None):
        pass