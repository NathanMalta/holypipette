'''
"Fake setup" for GUI development on a computer without access to a rig
'''
from holypipette.devices.amplifier.multiclamp import MultiClampChannel
from holypipette.devices.amplifier.amplifier import FakeAmplifier
from holypipette.devices.camera.pcocamera import PcoCamera
from holypipette.devices.manipulator import SensapexManip, ScientificaSerial
from holypipette.devices.pressurecontroller import IBBPressureController, FakePressureController
from holypipette.devices.manipulator import *
from holypipette.devices.cellsorter import CellSorterController, CellSorterManip
from serial import Serial

stageSerial = Serial(port='COM6', baudrate=9600, timeout=0.5)
stageController = ScientificaSerial(stageSerial)

sensapexController = SensapexManip()
stage = ManipulatorUnit(stageController, [1, 2])

camera = PcoCamera()
microscope = Microscope(stageController, 3)
microscope.up_direction = 1.0

unit = ManipulatorUnit(sensapexController, [1, 2, 3])

amplifier = FakeAmplifier() #MultiClampChannel(channel=1)

pressureSerial = Serial(port='COM15', baudrate=9600, timeout=0)
pressure = IBBPressureController(channel=1, arduinoSerial=pressureSerial)

cellSorter = CellSorterController()
cellSorterManip = CellSorterManip()