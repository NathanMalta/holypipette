import sys

import traceback
from PyQt5 import QtWidgets

from holypipette.log_utils import console_logger
from holypipette.interface import AutoPatchInterface
from holypipette.interface.pipettes import PipetteInterface
from holypipette.gui import PatchGui, EPhysGraph

from setup_IBB_rig import *
# from setup_fake_rig import *

console_logger()  # Log to the standard console as well

app = QtWidgets.QApplication(sys.argv)

pipette_controller = PipetteInterface(stage, microscope, camera, unit)
patch_controller = AutoPatchInterface(amplifier, pressure, pipette_controller)
gui = PatchGui(camera, pipette_controller, patch_controller)
graphs = EPhysGraph(daq)
graphs.show()

gui.initialize()
gui.show()
ret = app.exec_()

sys.exit(ret)
