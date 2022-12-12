'''
"Fake setup" for GUI development on a computer without access to a rig
'''
from holypipette.devices.amplifier.amplifier import FakeAmplifier
from holypipette.devices.camera.pcocamera import PcoCamera
from holypipette.devices.pressurecontroller import FakePressureController
from holypipette.devices.camera.camera import FakeCamera
from holypipette.devices.camera import FakeCalCamera
from holypipette.devices.manipulator import *

controller = FakeManipulator(min=[-4096, -4096, -3000, -4096, -4096, -2000],
                             max=[4096, 4096, 1000, 4096, 4096, 2000])
stage = ManipulatorUnit(controller, [4, 5])

controller.x[:3] = [-50, 10, 500]
controller.x[5] = 400
camera = FakeCalCamera(manipulator=controller, image_z=0, paramecium=True)
microscope = Microscope(controller, 6)
microscope.floor_Z = 0
microscope.up_direction = 1.0

units = [ManipulatorUnit(controller, [1, 2, 3])]

amplifier = FakeAmplifier()
pressure = FakePressureController()