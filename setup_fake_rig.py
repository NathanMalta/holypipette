'''
"Fake setup" for GUI development on a computer without access to a rig
'''
from holypipette.devices.amplifier.amplifier import FakeAmplifier
from holypipette.devices.amplifier.DAQ import FakeDAQ
from holypipette.devices.camera.pcocamera import PcoCamera
from holypipette.devices.pressurecontroller import FakePressureController
from holypipette.devices.camera.camera import FakeCamera
from holypipette.devices.camera import FakeCalCamera, WorldModel
from holypipette.devices.manipulator import *

controller = FakeManipulator(min=[-1000, -1000, -1000],
                             max=[1000, 1000, 1000])
pipetteManip = FakeManipulator(min=[-1000, -1000, -50],
                                      max=[4000, 20000, 20000])
stage = ManipulatorUnit(controller, [1, 2])

pipetteManip.x = [200, 300, 400] # start with pipette in frame
controller.x = [0, 0, 0]

worldModel = WorldModel()
camera = FakeCalCamera(stageManip=controller, pipetteManip=pipetteManip, image_z=0, worldModel=worldModel)

microscope = Microscope(controller, 3)
microscope.up_direction = 1.0

unit = ManipulatorUnit(pipetteManip, [1, 2, 3])

daq = FakeDAQ(worldModel=worldModel)
amplifier = FakeAmplifier()
pressure = FakePressureController()