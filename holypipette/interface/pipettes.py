# coding=utf-8
import pickle
import os

import numpy as np
from PyQt5 import QtCore

from holypipette.interface import TaskInterface, command, blocking_command
from holypipette.devices.manipulator.calibratedunit import CalibratedUnit, CalibratedStage, CalibrationConfig
from holypipette.devices.camera import WorldModel
import time

class PipetteInterface(TaskInterface):
    '''
    Controller for the stage, the microscope, a pipette, and the cell sorter.
    '''

    def __init__(self, stage, microscope, camera, unit, worldModel: WorldModel):
        super(PipetteInterface, self).__init__()
        self.microscope = microscope
        self.camera = camera
        # Create a common calibration configuration for all stages/manipulators
        self.calibration_config = CalibrationConfig(name='Calibration')
        self.calibrated_stage = CalibratedStage(stage, None, microscope, camera,
                                                config=self.calibration_config)
        self.calibrated_unit = CalibratedUnit(unit,
                                                self.calibrated_stage,
                                                microscope,
                                                camera,
                                                config=self.calibration_config)
        
        self.worldModel = worldModel

        self.calibrated_unit.load_configuration('M')
        self.calibrated_stage.load_configuration('S')
        print('loaded dummy config')

        self.cleaning_bath_position = None
        self.contact_position = None
        self.rinsing_bath_position = None
        self.paramecium_tank_position = None
        self.timer_t0 = time.time()
        self.pos_before_raise = None

    def connect(self, main_gui):
        pass #TODO: unused?

    @command(category='Manipulators',
             description='Record a calibration point at the current position',
             default_arg=10)
    def record_cal_point(self, none):
        self.calibrated_unit.record_cal_point()

    @blocking_command(category='Manipulators',
             description='Replace Tip of the pipette',
             task_description='Replace Pipette')
    def replaceTip(self):
        self.execute([self._replaceTip])

    def _replaceTip(self):
        currPos = self.calibrated_unit.position()
        self.calibrated_unit.relative_move(1000, 2) #move the tip way up to simulate what'd you do replacing it
        self.calibrated_unit.wait_until_still()
        self.worldModel.replacePipette()
        if currPos[2] < 0:
            currPos[2] = 0 #make sure we don't crash on the way down
        self.calibrated_unit.absolute_move(currPos) #move it back to where it was

    
    @blocking_command(category='Manipulators',
            description='Replace Tip of the pipette',
            task_description='Replace Pipette')
    def clean_pipette(self):
        self.execute([self._clean_pipette])

    def _clean_pipette(self):
        currPos = self.calibrated_unit.position()
        self.calibrated_unit.relative_move(1000, 2) #move the tip way up to simulate what'd you do replacing it
        self.calibrated_unit.wait_until_still()
        self.worldModel.cleanPipette()
        if currPos[2] < 0:
            currPos[2] = 0 #make sure we don't crash on the way down
        self.calibrated_unit.absolute_move(currPos) #move it back to where it was

    

    @command(category='Manipulators',
             description='Finish calibration',
             default_arg=10)
    def finish_calibration(self, none):
        self.calibrated_unit.finish_calibration()

    @command(category='Manipulators',
             description='Move pipette in x direction by {:.0f}μm',
             default_arg=10)
    def move_pipette_x(self, distance):
        self.calibrated_unit.relative_move(distance, axis=0)

    @command(category='Manipulators',
                description='Write current calibration to file')
    def write_calibration(self):
        if not self.calibrated_stage.calibrated:
            raise RuntimeError('Stage not calibrated')
        if not self.calibrated_unit.calibrated:
            raise RuntimeError('Manipulator not calibrated')
        
        with open('calibration.pickle', 'wb') as f:
            pickle.dump({'manip': self.calibrated_unit.save_configuration(),
                         'stage': self.calibrated_stage.save_configuration()}, f)
            
    @command(category='Manipulators',
                description='recalibrate manipulator offset while preserving matrix')
    def recalibrate_manipulator(self):
        self.calibrated_unit.recalibrate_pipette()
            
    @command(category='Manipulators',
             description='Move pipette in y direction by {:.0f}μm',
             default_arg=10)
    def move_pipette_y(self, distance):
        self.calibrated_unit.relative_move(distance, axis=1)

    @command(category='Manipulators',
             description='Move pipette in z direction by {:.0f}μm',
             default_arg=10)
    def move_pipette_z(self, distance):
        self.calibrated_unit.relative_move(distance, axis=2)

    @command(category='Microscope',
             description='Move microscope by {:.0f}μm',
             default_arg=10)
    def move_microscope(self, distance):
        self.microscope.relative_move(distance)

    @command(category='Microscope',
             description='Set the position of the floor (cover slip)',
             success_message='Cover slip position stored')
    def set_floor(self):
        self.microscope.floor_Z = self.microscope.position()

    @command(category='Stage',
             description='Move stage vertically by {:.0f}μm',
             default_arg=10)
    def move_stage_vertical(self, distance):
        self.calibrated_stage.relative_move(distance, axis=1)

    @command(category='Stage',
             description='Move stage horizontally by {:.0f}μm',
             default_arg=10)
    def move_stage_horizontal(self, distance):
        self.calibrated_stage.relative_move(distance, axis=0)

    @blocking_command(category='Stage',
                      description='Calibrate stage only',
                      task_description='Calibrating stage')
    def calibrate_stage(self):
        self.execute([self.calibrated_stage.calibrate])

    @blocking_command(category='Stage',
                    description='Create a Mosaic image',
                    task_description='Create a Mosaic Image')
    def create_mosaic(self):
        self.execute(self.calibrated_stage.mosaic)


    @blocking_command(category='Manipulators',
                      description='Calibrate manipulator',
                      task_description='Calibrating manipulator')
    def calibrate_manipulator(self):
        self.execute([self.calibrated_unit.calibrate])

    @blocking_command(category='Manipulators',
                      description='Focus the pipette',
                      task_description='Calibrating manipulator')
    def focus_pipette(self):
        self.execute([self.calibrated_unit.focus])

    @blocking_command(category='Manipulators',
                     description='Move pipette to position',
                     task_description='Moving to position with safe approach')
    def move_pipette(self, xy_position):
        x, y = xy_position
        position = np.array([x, y, self.microscope.position()])
        self.debug('asking for safe move to {}'.format(position))
        self.execute(self.calibrated_unit.safe_move, argument=position)

    @blocking_command(category='Manipulators',
                    description='Raise the pipette high enough to insert the coverslip',
                    task_description='Raising the pipette high enough to insert the coverslip')
    def raise_pipette(self, raise_distance = 1000):
        if self.pos_before_raise is None:
            self.pos_before_raise = self.calibrated_unit.dev.position()
            position = np.array([self.pos_before_raise[0], self.pos_before_raise[1], 0])
            self.execute(self.calibrated_unit.absolute_move, argument=position)
        else:
            raise RuntimeError('Pipette already raised')

    @blocking_command(category='Manipulators',
                description='Lower the pipette after inserting the coverslip',
                task_description='Lowering the pipette after inserting the coverslip')
    def lower_pipette(self):
        if self.pos_before_raise is not None:
            self.execute(self.calibrated_unit.absolute_move, argument=self.pos_before_raise)
            self.pos_before_raise = None
        else:
            raise RuntimeError('Pipette not raised')
        
    @blocking_command(category='Manipulators',
                     description='Move stage to position',
                     task_description='Moving stage to position')
    def move_stage(self, xy_position):
        x, y = xy_position
        position = np.array([x, y])
        self.debug('asking for reference move to {}'.format(position))
        self.execute(self.calibrated_stage.reference_relative_move, argument=-position) # compensatory move

    @blocking_command(category='Manipulators',
                    description='Center Pipette in Image Frame',
                    task_description='Centering Pipette in Image Frame')
    def center_pipette(self):
        x = 0
        y = 0
        z = self.microscope.position()
        position = np.array([x, y, z])
        self.execute(self.calibrated_unit.safe_move, argument=position) # compensatory move


    @blocking_command(category='Microscope',
                      description='Go to the floor (cover slip)',
                      task_description='Go to the floor (cover slip)')
    def go_to_floor(self):
        if self.microscope.floor_Z is None:
            raise RuntimeError("Coverslip floor must be set.")
        self.execute(self.microscope.absolute_move,
                     argument=self.microscope.floor_Z)