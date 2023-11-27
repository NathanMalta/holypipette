"""
A fake device useful for development.
It has 9 axes, numbered 1 to 9.
"""
from __future__ import print_function
from __future__ import absolute_import
from .manipulator import Manipulator

from numpy import zeros, clip, pi
import time
import math

__all__ = ['FakeManipulator']

class FakeManipulator(Manipulator):
    def __init__(self, min=None, max=None):
        Manipulator.__init__(self)
        # Minimum and maximum positions for all axes
        self.min = min
        self.max = max
        if all([min is not None, max is not None]):
            if len(min) != len(max):
                raise ValueError('min/max needs to be the same length (# of axes)')

        self.num_axes = len(min)

        #continuous movement values
        self.x = zeros(self.num_axes) # Position of all axes
        self.set_max_speed(10000)
        self.setpoint = self.x.copy()
        self.speeds = zeros(self.num_axes)
        self.cmd_time = [None] * self.num_axes

    def set_max_speed(self, speed : int):
        self.max_speed = speed / 1000 * 82 #for some reason, when you specify 1000 as the max speed, it actually moves at 82 um/s

    def position(self, axis=None):
        '''
        Current position along an axis.

        Parameters
        ----------
        axis : axis number

        Returns
        -------
        The current position of the device axis in um.
        '''

        if axis == None:
            for i in range(self.num_axes):
                self.update_axis(i+1)
            return self.x
        else:
            self.update_axis(axis)
            return self.x[axis-1]

    def update_axis(self, axis):
        '''Updates the position on the given axis if a command is being executed.
        Returns True if the command is still running, False otherwise.
        '''

        #Nothing commanded on this axis
        if self.cmd_time[axis-1] is None:
            return False

        #see if the last command is still running
        dt = time.time() - self.cmd_time[axis-1]
        self.cmd_time[axis-1] = time.time()

        #we're moving forward, but haven't reached the setpoint
        if (self.x[axis-1] + self.speeds[axis-1] * dt < self.setpoint[axis-1]) and self.speeds[axis-1] > 0:
            self.x[axis-1] = self.x[axis-1] + self.speeds[axis-1] * dt
            return True
        
        #we're moving backward, but haven't reached the setpoint
        if (self.x[axis-1] + self.speeds[axis-1] * dt) > self.setpoint[axis-1] and self.speeds[axis-1] < 0:
            self.x[axis-1] = self.x[axis-1] + self.speeds[axis-1] * dt
            return True

        #we've reached the setpoint
        self.speeds[axis-1] = 0
        self.cmd_time[axis-1] = None
        self.x[axis-1] = self.setpoint[axis-1]
        return False

    def absolute_move(self, x, axis, speed=None):
        '''
        Moves the device axis to position x.

        Parameters
        ----------
        axis: axis number
        x : target position in um.
        '''

        if self.update_axis(axis):
            print(f'axis: {axis}\tpos: {self.x[axis-1]}\tspeed: {self.x[axis-1]:.2f}\tsetpoint: {self.setpoint[axis-1]}\tfail 1')
            raise RuntimeError("Cannot move while another command is running")

        if self.min is None:
            self.setpoint[axis-1] = x
        else:
            self.setpoint[axis-1] = clip(x, self.min[axis-1], self.max[axis-1])
        
        self.cmd_time[axis-1] = time.time()
        self.speeds[axis-1] = self.max_speed * math.copysign(1, x - self.x[axis-1])


    def absolute_move_group(self, x, axes, speed=None):
        for xi,axis in zip(x,axes):
            print(axis, xi)
            self.absolute_move(xi, axis)
    
    def wait_until_still(self, axes=None):
        for i in range(self.num_axes):
            while self.update_axis(i+1):
                time.sleep(0.1)