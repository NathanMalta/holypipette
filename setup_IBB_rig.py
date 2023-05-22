'''
"Fake setup" for GUI development on a computer without access to a rig
'''
from holypipette.devices.amplifier.multiclamp import MultiClampChannel
from holypipette.devices.amplifier.amplifier import FakeAmplifier
from holypipette.devices.amplifier.DAQ import DAQ
from holypipette.devices.camera.pcocamera import PcoCamera
from holypipette.devices.manipulator import SensapexManip, ScientificaSerialNoEncoder
from holypipette.devices.pressurecontroller import IBBPressureController, FakePressureController
from holypipette.devices.manipulator import *
from holypipette.devices.cellsorter import CellSorterController, CellSorterManip, FakeCellSorterManip
import serial

stageSerial = serial.Serial(port='COM6', baudrate=9600, timeout=1)
# zAxisEncoderComms = serial.Serial(port='COM19', baudrate=115200, timeout=1)
# stageController = ScientificaSerialEncoder(stageSerial, zAxisEncoderComms)
stageController = ScientificaSerialNoEncoder(stageSerial)

sensapexController = SensapexManip()
stage = ManipulatorUnit(stageController, [1, 2])

camera = PcoCamera()
microscope = Microscope(stageController, 3)
microscope.up_direction = 1.0

unit = ManipulatorUnit(sensapexController, [1, 2, 3])

# daq = DAQ('Dev6', 'ai1', 'Dev6', 'ao0') #This is "new" DAQ
daq = DAQ('cDAQ1Mod3', 'ai0', 'cDaq1Mod1', 'ao1') #This is "old" DAQ
amplifier = MultiClampChannel(channel=1)

pressureSerial = serial.Serial(port='COM15', baudrate=9600, timeout=0)
pressure = IBBPressureController(channel=1, arduinoSerial=pressureSerial)

controllerSerial = serial.Serial('COM8', 115200, timeout=2, parity=serial.PARITY_NONE, stopbits=1, 
                                    bytesize=8, write_timeout=1, inter_byte_timeout=2)
cellSorterController = CellSorterController(controllerSerial)

# manipulatorSerial = serial.Serial('COM14', 57600, timeout=2, parity=serial.PARITY_NONE, stopbits=2, 
#                                             bytesize=8, write_timeout=1, inter_byte_timeout=2)
# cellSorterManip = CellSorterManip(manipulatorSerial)
cellSorterManip = FakeCellSorterManip()
