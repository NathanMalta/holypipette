'''
"Fake setup" for GUI development on a computer without access to a rig
'''
from holypipette.devices.amplifier.multiclamp import MultiClampChannel
from holypipette.devices.camera.pcocamera import PcoCamera
from holypipette.devices.manipulator import SensapexManip, Scientifica
from holypipette.devices.pressurecontroller import IBBPressureController
from holypipette.devices.manipulator import *

stageController = Scientifica()

sensapexController = SensapexManip()
stage = ManipulatorUnit(stageController, [1, 2])

camera = PcoCamera()
microscope = Microscope(stageController, 3)
microscope.floor_Z = 0
microscope.up_direction = 1.0

units = [ManipulatorUnit(sensapexController, [1, 2, 3])]

amplifier = MultiClampChannel()
pressure = IBBPressureController()

# amplifier.get_primary_signal()