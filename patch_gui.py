import sys

import traceback
from PyQt5 import QtWidgets, QtCore

from holypipette.log_utils import console_logger
from holypipette.interface import AutoPatchInterface
from holypipette.interface.pipettes import PipetteInterface
from holypipette.gui import PatchGui, EPhysGraph, Tutorial

from setup_fake_rig import *

console_logger()  # Log to the standard console as well

app = QtWidgets.QApplication(sys.argv)

pipette_controller = PipetteInterface(stage, microscope, camera, unit, worldModel)
patch_controller = AutoPatchInterface(amplifier, pressure, pipette_controller, worldModel)
gui = PatchGui(camera, pipette_controller, patch_controller)
graphs = EPhysGraph(daq, pressure)
graphs.show()

gui.initialize()
gui.show()

tutorial = Tutorial()
tutorial.raise_()
tutorial.show()

ret = app.exec_()
sys.exit(ret)
