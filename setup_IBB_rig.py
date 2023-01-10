'''
"Fake setup" for GUI development on a computer without access to a rig
'''
from holypipette.devices.amplifier.multiclamp import MultiClampChannel
from holypipette.devices.amplifier.amplifier import FakeAmplifier
from holypipette.devices.camera.pcocamera import PcoCamera
from holypipette.devices.manipulator import SensapexManip, Scientifica
from holypipette.devices.pressurecontroller import IBBPressureController, FakePressureController
from holypipette.devices.manipulator import *
from serial import Serial

stageController = Scientifica()

sensapexController = SensapexManip()
stage = ManipulatorUnit(stageController, [1, 2])

camera = PcoCamera()
microscope = Microscope(stageController, 3)
microscope.up_direction = 1.0

units = [ManipulatorUnit(sensapexController, [1, 2, 3])]

amplifier = MultiClampChannel(channel=1)

pressureSerial = Serial(port='COM15', baudrate=9600, timeout=3)
pressure = IBBPressureController(channel=1, arduinoSerial=pressureSerial)