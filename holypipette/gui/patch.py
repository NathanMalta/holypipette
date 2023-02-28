from __future__ import absolute_import

from types import MethodType

from PyQt5 import QtCore, QtWidgets
from PyQt5.QtCore import Qt

from holypipette.controller import TaskController
from holypipette.gui.manipulator import ManipulatorGui
from holypipette.interface.patch import AutoPatchInterface
from holypipette.interface.pipettes import PipetteInterface
from holypipette.interface.base import command


class PatchGui(ManipulatorGui):

    patch_command_signal = QtCore.pyqtSignal(MethodType, object)
    patch_reset_signal = QtCore.pyqtSignal(TaskController)

    def __init__(self, camera, pipette_interface, patch_interface,
                 with_tracking=False):
        super(PatchGui, self).__init__(camera, pipette_interface,
                                       with_tracking=with_tracking)
        self.setWindowTitle("Patch GUI")
        # Note that pipette interface already runs in a thread, we need to use
        # the same for the patch interface
        self.patch_interface = patch_interface
        self.patch_interface.moveToThread(pipette_interface.thread())
        self.interface_signals[self.patch_interface] = (self.patch_command_signal,
                                                        self.patch_reset_signal)
        
        button_tab = PatchButtons(self.patch_interface, pipette_interface, self.start_task, self.interface_signals)
        self.add_config_gui(self.patch_interface.config)
        self.add_tab(button_tab, 'Buttons')
        # Update the pressure and information in the status bar every 50ms
        self.pressure_timer = QtCore.QTimer()
        self.pressure_timer.timeout.connect(self.display_pressure)
        self.pressure_timer.start(50)

    def display_pressure(self):
        pressure = self.patch_interface.pressure.get_pressure()
        self.set_status_message('pressure', 'Pressure: {:.0f} mbar'.format(pressure))

    def register_commands(self):
        super(PatchGui, self).register_commands()
        self.register_mouse_action(Qt.LeftButton, Qt.ShiftModifier,
                                   self.patch_interface.patch_with_move)
        self.register_mouse_action(Qt.LeftButton, Qt.ControlModifier,
                                   self.patch_interface.patch_without_move)
        self.register_key_action(Qt.Key_B, None,
                                 self.patch_interface.break_in)
        self.register_key_action(Qt.Key_F2, None,
                                 self.patch_interface.store_cleaning_position)
        self.register_key_action(Qt.Key_F3, None,
                                 self.patch_interface.store_rinsing_position)
        self.register_key_action(Qt.Key_F4, None,
                                 self.patch_interface.clean_pipette)


class TrackingPatchGui(PatchGui):

    def __init__(self, camera, pipette_interface, patch_interface,
                 with_tracking=False):
        super(TrackingPatchGui, self).__init__(camera, pipette_interface,
                                               patch_interface,
                                               with_tracking=True)
        self.setWindowTitle("Patch GUI with tracking")

    def register_commands(self):
        super(TrackingPatchGui, self).register_commands()
        self.register_key_action(Qt.Key_F5, None,
                                 self.patch_interface.sequential_patching)
        self.register_key_action(Qt.Key_F8, None,
                                 self.patch_interface.contact_detection)
        self.register_mouse_action(Qt.RightButton, None,
                                   self.camera_interface.track_object)

class PatchButtons(QtWidgets.QWidget):
    def nothing(self):
        pass

    def __init__(self, patch_interface : AutoPatchInterface, pipette_interface : PipetteInterface, start_task, interface_signals):
        super(PatchButtons, self).__init__()
        self.patch_interface = patch_interface
        self.pipette_interface = pipette_interface

        self.start_task = start_task
        self.interface_signals = interface_signals

        layout = QtWidgets.QVBoxLayout()
        layout.setAlignment(Qt.AlignTop)
        buttonList = ['Calibrate Stage', 'Calibrate Pipette', 'Set Cell Plane', 'Focus Cell Plane', 'Focus Pipette Plane']
        cmds = [self.pipette_interface.calibrate_stage, self.pipette_interface.calibrate_manipulator, self.pipette_interface.set_floor, self.pipette_interface.go_to_floor, self.pipette_interface.focus_pipette]

        # create a new row for each button
        for i, button in enumerate(buttonList):
            new_row = QtWidgets.QHBoxLayout()
            new_row.setAlignment(Qt.AlignLeft)

            button = QtWidgets.QPushButton(button)
            
            #make sure button fills the row
            button.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)

            #connect the button to the command, run using the start_task method
            button.clicked.connect(lambda state, i=i: self.run_command(cmds[i]))

            #add the button to the frame
            new_row.addWidget(button)
            layout.addLayout(new_row)

        self.setLayout(layout)

    def run_command(self, cmd):

        #check if the task_description exists
        if hasattr(cmd, 'task_description'):
            #we're dealing with a command
            self.start_task(cmd.task_description, cmd.__self__)
            if cmd.__self__ in self.interface_signals:
                command_signal, _ = self.interface_signals[cmd.__self__]
                command_signal.emit(cmd, None)
            else:
                cmd(None)
        else:
            #we're dealing with a function
            cmd()
