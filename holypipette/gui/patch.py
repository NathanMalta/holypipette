from __future__ import absolute_import

from types import MethodType

from PyQt5 import QtCore, QtWidgets
from PyQt5.QtCore import Qt
import pyqtgraph as pg
import PyQt5.QtGui as QtGui
import numpy as np
import pygame

from holypipette.controller import TaskController
from holypipette.gui.manipulator import ManipulatorGui
from holypipette.interface.patch import AutoPatchInterface
from holypipette.interface.pipettes import PipetteInterface
from holypipette.devices.controller.xboxController import XboxController, Button, Axis


class PatchGui(ManipulatorGui):

    patch_command_signal = QtCore.pyqtSignal(MethodType, object)
    patch_reset_signal = QtCore.pyqtSignal(TaskController)

    def __init__(self, camera, pipette_interface : PipetteInterface, patch_interface : AutoPatchInterface,
                 with_tracking=False):
        super(PatchGui, self).__init__(camera, pipette_interface,
                                       with_tracking=with_tracking)
        self.setWindowTitle("Patch GUI")
        # Note that pipette interface already runs in a thread, we need to use
        # the same for the patch interface
        self.patch_interface = patch_interface
        self.pipette_interface = pipette_interface

        self.patch_interface.moveToThread(pipette_interface.thread())
        self.interface_signals[self.patch_interface] = (self.patch_command_signal,
                                                        self.patch_reset_signal)
        
        #add manual patching button tab
        man_button_tab = ManualPatchButtons(self.patch_interface, pipette_interface, self.start_task, self.interface_signals)
        self.add_tab(man_button_tab, 'Manual Patching', index = 0)

        #add automatic patching button tab
        auto_button_tab = AutoPatchButtons(self.patch_interface, pipette_interface, self.start_task, self.interface_signals)
        # self.add_config_gui(self.patch_interface.config)
        self.add_tab(auto_button_tab, 'Automatic Patching', index = 1)

        # Update the pressure and information in the status bar
        self.pressure_timer = QtCore.QTimer()
        self.pressure_timer.timeout.connect(self.display_pressure)
        self.pressure_timer.start(100)
        self.patch_interface.set_pressure_ambient()

        # Setup controller (if available)
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() > 0:
            self.set_status_message('Controller Status', 'Controller Connected')

            self.controller_timer = QtCore.QTimer()
            self.controller_timer.timeout.connect(self.update_controller)
            self.controller_timer.start(50)

            self.xbox_controller = XboxController()

            def a_pressed(status):
                if status:
                    print('A pressed')
                else:
                    print('A released')

            self.register_controller_actions()

            self.xbox_controller.link_button(Button.A_BUTTON, a_pressed)
        else:
            self.set_status_message('Controller Status', 'Controller Not Connected')


    def register_controller_actions(self):
        self.xbox_controller.link_axis(Axis.LEFT_X, lambda x: self.pipette_interface.move_stage_horizontal(x * 10))
        self.xbox_controller.link_axis(Axis.LEFT_Y, lambda x: self.pipette_interface.move_stage_vertical(x * 10))

        self.xbox_controller.link_axis(Axis.RIGHT_X, lambda x: self.pipette_interface.move_pipette_x(x * 10))
        self.xbox_controller.link_axis(Axis.RIGHT_Y, lambda x: self.pipette_interface.move_pipette_y(x * 10))

        self.xbox_controller.link_button_hold(Button.DPAD_UP, lambda: self.pipette_interface.move_microscope(10))
        self.xbox_controller.link_button_hold(Button.DPAD_DOWN, lambda: self.pipette_interface.move_microscope(-10))


        self.xbox_controller.link_axis(Axis.RIGHT_TRIGGER, lambda x: self.pipette_interface.move_pipette_z(x * 10))
        self.xbox_controller.link_axis(Axis.LEFT_TRIGGER, lambda x: self.pipette_interface.move_pipette_z(x * -10))

    def display_pressure(self):
        pressure = self.patch_interface.pressure.get_pressure()
        self.set_status_message('pressure', 'Pressure: {:.0f} mbar'.format(pressure))

    def update_controller(self):
        pygame.event.pump()
        self.xbox_controller.update()

    def register_commands(self):
        super(PatchGui, self).register_commands()
        # self.register_mouse_action(Qt.LeftButton, Qt.ShiftModifier,
        #                            self.patch_interface.patch_with_move)
        self.register_mouse_action(Qt.LeftButton, Qt.NoModifier,
                                   self.patch_interface.add_cell)



class TrackingPatchGui(PatchGui):

    def __init__(self, camera, pipette_interface, patch_interface,
                 with_tracking=False):
        super(TrackingPatchGui, self).__init__(camera, pipette_interface,
                                               patch_interface,
                                               with_tracking=True)
        self.setWindowTitle("Patch GUI with tracking")

    def register_commands(self):
        super(TrackingPatchGui, self).register_commands()

class ButtonTabWidget(QtWidgets.QWidget):
    def nothing(self):
        pass

    def __init__(self):
        super(ButtonTabWidget, self).__init__()

    def do_nothing(self):
        pass # a dummy function for buttons that aren't implemented yet

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

    def addPositionBox(self, name: str, layout, update_func, axes=['x', 'y', 'z']):
        #add a box for manipulator and stage positions
        box = QtWidgets.QGroupBox(name)
        row = QtWidgets.QHBoxLayout()
        indicies = []
        #create a new row for each position
        for j, axis in enumerate(axes):
            #create a label for the position
            label = QtWidgets.QLabel(f'{axis}: TODO')
            row.addWidget(label)

            indicies.append(len(self.pos_labels))
            self.pos_labels.append(label)
        box.setLayout(row)
        layout.addWidget(box)

        #periodically update the position labels
        pos_timer = QtCore.QTimer()
        pos_timer.timeout.connect(lambda: update_func(indicies))
        pos_timer.start(200)
        self.pos_update_timers.append(pos_timer)
    
    def addButtonList(self, box_name: str, layout, buttonNames, cmds):
        box = QtWidgets.QGroupBox(box_name)
        rows = QtWidgets.QVBoxLayout()
        # create a new row for each button
        for i, buttons_in_row in enumerate(buttonNames):
            new_row = QtWidgets.QHBoxLayout()
            new_row.setAlignment(Qt.AlignLeft)

            #for each button in the row, create a button
            for j, button in enumerate(buttons_in_row):
                button = QtWidgets.QPushButton(button)

                #make sure buttons fill the space in the x-axis
                button.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)

                #set the size of the button
                button.setMinimumWidth(50)
                button.setMinimumHeight(50)

                #connect the button to the command, run using the start_task method
                button.clicked.connect(lambda state, i=i, j=j: self.run_command(cmds[i][j]))

                #add the button to the frame
                new_row.addWidget(button)
            rows.addLayout(new_row)
        box.setLayout(rows)
        layout.addWidget(box)

    
class AutoPatchButtons(ButtonTabWidget):
    def __init__(self, patch_interface : AutoPatchInterface, pipette_interface : PipetteInterface, start_task, interface_signals):
        super(AutoPatchButtons, self).__init__()
        self.patch_interface = patch_interface
        self.pipette_interface = pipette_interface

        self.start_task = start_task
        self.interface_signals = interface_signals

        self.pos_update_timers = []
        self.pos_labels = []

        layout = QtWidgets.QVBoxLayout()
        layout.setAlignment(Qt.AlignTop)

        self.addPositionBox('stage position', layout, self.update_stage_pos_labels)
        self.addPositionBox('pipette position', layout, self.update_pipette_pos_labels)

        #add a box for cal
        buttonList = [['Set Cell Plane']]
        cmds = [[self.pipette_interface.set_floor]]
        self.addButtonList('calibration', layout, buttonList, cmds)

        #add a box for maintenance
        buttonList = [['Clean Pipette'], ['Replace Pipette']]
        cmds = [[self.pipette_interface.clean_pipette], [self.pipette_interface.replaceTip]]
        self.addButtonList('maintenance', layout, buttonList, cmds)

        #add a box for movement
        buttonList = [[ 'Focus Cell Plane', 'Focus Pipette Plane'], ['Center Pipette']]
        cmds = [[self.pipette_interface.go_to_floor, self.pipette_interface.focus_pipette], [self.pipette_interface.center_pipette]]
        self.addButtonList('movement', layout, buttonList, cmds)

        #add a box for patching cmds
        buttonList = [['Select Cell', 'Remove Last Cell'], ['Start Patch']]
        cmds = [[self.patch_interface.start_selecting_cells, self.patch_interface.remove_last_cell], [self.patch_interface.patch]]
        self.addButtonList('patching', layout, buttonList, cmds)
        
        self.setLayout(layout)

    def update_pipette_pos_labels(self, indicies):
        #update the position labels
        currPos = self.pipette_interface.calibrated_unit.unit.position()
        for i, ind in enumerate(indicies):
            label = self.pos_labels[ind]
            label.setText(f'{label.text().split(":")[0]}: {currPos[i]:.2f}')

    def update_stage_pos_labels(self, indicies):
        #update the position labels
        xyPos = self.pipette_interface.calibrated_stage.position()
        zPos = self.pipette_interface.microscope.position()
        for i, ind in enumerate(indicies):
            label = self.pos_labels[ind]
            if i < 2:
                label.setText(f'{label.text().split(":")[0]}: {xyPos[i]:.2f}')
            else:
                label.setText(f'{label.text().split(":")[0]}: {zPos:.2f}')



class ManualPatchButtons(ButtonTabWidget):
    def __init__(self, patch_interface : AutoPatchInterface, pipette_interface : PipetteInterface, start_task, interface_signals):
        super(ManualPatchButtons, self).__init__()
        self.patch_interface = patch_interface
        self.pipette_interface = pipette_interface

        self.start_task = start_task
        self.interface_signals = interface_signals

        self.pos_update_timers = []
        self.pos_labels = []

        layout = QtWidgets.QVBoxLayout()
        layout.setAlignment(Qt.AlignTop)

        self.addPositionBox('stage position', layout, self.update_stage_pos_labels)
        self.addPositionBox('pipette position', layout, self.update_pipette_pos_labels)

        #add a box for maintenance
        buttonList = [['Clean Pipette'], ['Replace Pipette']]
        cmds = [[self.pipette_interface.clean_pipette], [self.pipette_interface.replaceTip]]
        self.addButtonList('maintenance', layout, buttonList, cmds)

        #add a box for patching cmds
        buttonList = [['Ambient Pressure'], ['Gigaseal Pressure'], ['Break-in Pressure']]
        cmds = [[self.patch_interface.set_pressure_ambient], [self.patch_interface.set_pressure_sealing], [self.patch_interface.break_in]]
        self.addButtonList('Pressue Control', layout, buttonList, cmds)
        
        self.setLayout(layout)

    def update_pipette_pos_labels(self, indicies):
        #update the position labels
        currPos = self.pipette_interface.calibrated_unit.unit.position()
        for i, ind in enumerate(indicies):
            label = self.pos_labels[ind]
            label.setText(f'{label.text().split(":")[0]}: {currPos[i]:.2f}')

    def update_stage_pos_labels(self, indicies):
        #update the position labels
        xyPos = self.pipette_interface.calibrated_stage.position()
        zPos = self.pipette_interface.microscope.position()
        for i, ind in enumerate(indicies):
            label = self.pos_labels[ind]
            if i < 2:
                label.setText(f'{label.text().split(":")[0]}: {xyPos[i]:.2f}')
            else:
                label.setText(f'{label.text().split(":")[0]}: {zPos:.2f}')