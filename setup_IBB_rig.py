'''
"Fake setup" for GUI development on a computer without access to a rig
'''
from holypipette.devices.amplifier.amplifier import FakeAmplifier
from holypipette.devices.camera.pcocamera import PcoCamera
from holypipette.devices.manipulator import SensapexManip
from holypipette.devices.pressurecontroller import FakePressureController
from holypipette.devices.camera.camera import FakeCamera
from holypipette.devices.manipulator import *

fakeController = FakeManipulator(min=[-4096, -4096, -1000, -4096, -4096, -1000],
                             max=[4096, 4096, 1000, 4096, 4096, 1000])

sensapexController = SensapexManip()
stage = ManipulatorUnit(fakeController, [4, 5])

fakeController.x[:3] = [-50, 10, 500]
fakeController.x[5] = 520

camera = PcoCamera()
microscope = Microscope(fakeController, 6)
microscope.floor_Z = 0
microscope.up_direction = 1.0

units = [ManipulatorUnit(sensapexController, [1, 2, 3])]

amplifier = FakeAmplifier()
pressure = FakePressureController()