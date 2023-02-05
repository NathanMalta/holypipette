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
    
    frame_lag = NumberWithUnit(4, unit='frames',
                                     doc='number of frames between for computing change with optical flow',
                                     bounds=(1, 20))
    
    pipette_diag_move = NumberWithUnit(200, unit='um',
                                     doc='x, y dist to move for pipette cal.',
                                     bounds=(50, 10000))
    

    categories = [('Stage Calibration', ['autofocus_dist', 'stage_diag_move', 'frame_lag']),
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
        self.r0 = zeros(3) # offset for px -> um conversion
        self.r0_inv = zeros(3) # offset for um -> px conversion

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

    def pixels_to_pipette_um(self, pos_pixels):
        '''
        Converts pixel coordinates to pipette um.
        '''
        return dot(self.Minv, pos_pixels) + self.r0_inv
    
    def pixels_to_um(self, pos_microns):
        '''
        Converts um to pixel coordinates.
        '''
        return dot(self.M, pos_microns) + self.r0

    def reference_position(self):
        '''
        Position in the reference camera system.

        Returns
        -------
        The current position in um as an XYZ vector.
        '''
        # if not self.calibrated:
        #     raise CalibrationError
        pipette_pos_um = self.position() # position vector (um) in manipulator unit system
        pos_pixels = self.pixels_to_um(pipette_pos_um) # position vector (pixels) in camera system
        return pos_pixels

    def reference_move(self, pos_pixels, safe = False):
        '''
        Moves the unit to position pos_pixels in reference camera system, without moving the stage.

        Parameters
        ----------
        r : XYZ position vector in um
        safe : if True, moves the Z axis first or last, so as to avoid touching the coverslip
        '''

        if np.isnan(np.array(pos_pixels)).any():
            raise RuntimeError("can not move to nan location.")
        
        pos_micron = self.pixels_to_um(pos_pixels) # position vector (um) in manipulator unit system
        self.absolute_move(pos_micron, blocking=True)

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
        
        # r from pyQt has origin at the center of the image, move origin to the top left corner (as expected by calibration)
        r = np.array(r)
        r = r + np.array([self.camera.width // 2, self.camera.height // 2, 0])

        self.reference_move(r, safe = True) # Or relative move in manipulator coordinates, first axis (faster)


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

    def calibrate(self): #TODO: Z-Axis calibration?
        '''
        Automatic calibration of the pipette manipulator.
        '''
        
        # move the pipette and create a calibration matrix (pix -> um)
        mat, initPos = self.pipetteCalHelper.calibrate(dist=self.config.pipette_diag_move)

        # make mat 3x3
        mat = np.append(mat, np.array([[0,0,1]]), axis=0)
        print(f'calibration matrix: {mat}')

        # *** Compute the (pseudo-)inverse ***
        mat_inv = pinv(mat)

        # store r0 and r0_inv
        self.r0 = np.append(mat[0:2, 2], 0) #um -> pixels offset
        self.r0_inv = np.append(mat_inv[0:2, 2], self.microscope.position() - self.position(2)) #pixels -> um offset

        #for M and Minv, we only want the upper 2x2 matrix (b/c assumption that z axis is equivilant), the rest of the matrix is just the identity
        self.M = np.eye(3,3)
        self.M[0:2,0:2] = mat[0:2, 0:2]

        self.Minv = np.eye(3,3)
        self.Minv[0:2,0:2] = mat_inv[0:2, 0:2]

        #check for nan values (invalid cal)
        if isnan(self.M).any() or isnan(self.Minv).any():
            raise CalibrationError('Matrix contains NaN values')

        print('Calibration Successful!')
        print('M: ', self.M)
        print('r0: ', self.r0)
        print()
        print('Minv: ', self.Minv)
        print('r0_inv: ', self.r0_inv)

        self.calibrated = True

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
        self.stageCalHelper = StageCalHelper(unit, camera, self.config.frame_lag)

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
        if width == None:
            width = self.camera.width * 4
        if height == None:
            height = self.camera.height * 4

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

        cv2.imwrite('mosaic.png', big_image)

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