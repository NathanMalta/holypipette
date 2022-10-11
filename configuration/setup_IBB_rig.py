'''
"Fake setup" for GUI development on a computer without access to a rig
'''
from holypipette.devices.amplifier.amplifier import FakeAmplifier
from holypipette.devices.camera.pcocamera import PcoCamera
from holypipette.devices.pressurecontroller import FakePressureController
from holypipette.devices.camera.camera import FakeCamera
from holypipette.devices.manipulator import *

pipetteController = UMP.get_ump()

fakeCont = FakeManipulator(min=[-4096, -4096, -1000, -4096, -4096, -1000], #TODO: replace this with scientifica stage control
                             max=[4096, 4096, 1000, 4096, 4096, 1000])

stage = ManipulatorUnit(fakeCont, [4, 5]) 

camera = PcoCamera()
microscope = Microscope(fakeCont, 6)

units = [ManipulatorUnit(pipetteController, [1, 2, 3])] #TODO correct axes by running sensapex.py

amplifier = FakeAmplifier()
pressure = FakePressureController()