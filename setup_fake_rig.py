'''
"Fake setup" for GUI development on a computer without access to a rig
'''
from holypipette.devices.amplifier.amplifier import FakeAmplifier
from holypipette.devices.camera.pcocamera import PcoCamera
from holypipette.devices.pressurecontroller import FakePressureController
from holypipette.devices.camera.camera import FakeCamera
from holypipette.devices.camera import FakeCalCamera, FakePipetteManipulator
from holypipette.devices.manipulator import *
from holypipette.devices.cellsorter import FakeCellSorterController, FakeCellSorterManip

controller = FakeManipulator(min=[-4096, -4096, -2000],
                             max=[4096, 4096, 2000])
pipetteManip = FakePipetteManipulator(min=[-4096, -4096, -3000],
                                      max=[4096, 4096, 1000])
stage = ManipulatorUnit(controller, [1, 2])

pipetteManip.x = [200, 300, 400] # start with pipette in frame
controller.x[2] = 400 # start with stage out of focus
camera = FakeCalCamera(stageManip=controller, pipetteManip=pipetteManip, image_z=100)
microscope = Microscope(controller, 3)
microscope.up_direction = 1.0

unit = ManipulatorUnit(pipetteManip, [1, 2, 3])

amplifier = FakeAmplifier()
pressure = FakePressureController()

cellSorter = FakeCellSorterController()
cellSorterManip = FakeCellSorterManip()