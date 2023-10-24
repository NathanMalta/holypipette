import pygame
from enum import Enum
from PyQt5.QtCore import QTimer


class Button(Enum):
    A_BUTTON = 0
    B_BUTTON = 1
    X_BUTTON = 2
    Y_BUTTON = 3

    LEFT_CIRCLE_BUTTON = 4
    RIGHT_CIRCLE_BUTTON = 6

    X_BOX_BUTTON = 5

    LEFT_STICK = 7
    RIGHT_STICK = 8

    LEFT_BUMPER = 9
    RIGHT_BUMPER = 10

    DPAD_UP = 11
    DPAD_DOWN = 12
    DPAD_LEFT = 13
    DPAD_RIGHT = 14

class Axis(Enum):
    LEFT_X = 0
    LEFT_Y = 1

    RIGHT_X = 2
    RIGHT_Y = 3

    LEFT_TRIGGER = 4
    RIGHT_TRIGGER = 5


class XboxController(object):

    def __init__(self):
        pygame.init()
        pygame.joystick.init()
        assert pygame.joystick.get_count() > 0, 'No joystick detected'

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

        self.button_states = [0 for i in range(self.joystick.get_numbuttons())]
        self.axis_states = [0 for i in range(self.joystick.get_numaxes())]

        self.button_functions = [None for i in range(self.joystick.get_numbuttons())]
        self.button_hold_functions = [None for i in range(self.joystick.get_numbuttons())]

        self.axis_functions = [None for i in range(self.joystick.get_numaxes())]

        self.axis_states = [-1 for i in range(self.joystick.get_numaxes())]

        self.delta_trigger = 0.1 # the minimum trigger value to activate function

    def update(self):
        buttons = [self.joystick.get_button(i) for i in range(self.joystick.get_numbuttons())]
        axes = [self.joystick.get_axis(i) for i in range(self.joystick.get_numaxes())]

        #convert trigger values to 0-1
        axes[Axis.LEFT_TRIGGER.value] = (axes[Axis.LEFT_TRIGGER.value] + 1) / 2
        axes[Axis.RIGHT_TRIGGER.value] = (axes[Axis.RIGHT_TRIGGER.value] + 1) / 2

        # setup button functions linking
        for i in range(len(buttons)):
            if buttons[i] != self.button_states[i]:
                self.button_states[i] = buttons[i]
                if self.button_functions[i] is not None:
                    self.button_functions[i](buttons[i])

            if buttons[i]:
                if self.button_hold_functions[i] is not None:
                    self.button_hold_functions[i]()

        #setup axis functions linking
        for i in range(len(axes)):
            if abs(axes[i]) > self.delta_trigger:
                self.axis_states[i] = axes[i]
                if self.axis_functions[i] is not None:
                    self.axis_functions[i](axes[i])

        self.button_states = buttons
        self.axis_states = axes
    
    def link_button(self, button, function):
        self.button_functions[button.value] = function

    def link_button_hold(self, button, function):
        self.button_hold_functions[button.value] = function

    def link_axis(self, trigger, function):
        self.axis_functions[trigger.value] = function
