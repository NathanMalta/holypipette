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

# TODO: Move in 3D
class FakeManipulator(Manipulator):
    def __init__(self, min=None, max=None, angle=25.):
        Manipulator.__init__(self)
        self.x = zeros(6) # Position of all axes
        # Minimum and maximum positions for all axes
        self.min = min
        self.max = max
        if (any([min is not None, max is not None]) and
                not all([min is not None, max is not None])):
            raise ValueError('Need to provide either both minimum and maximum '
                             'range or neither')
        if all([min is not None, max is not None]):
            if len(min) != 6 or len(max) != 6:
                raise ValueError('min/max argument needs to be a vector of '
                                 'length 6.')
        self.angle = angle*pi/180

        #continuous movement values
        self.set_max_speed(10000)
        self.setpoint = self.x.copy()
        self.speeds = zeros(6)
        self.cmd_time = [None] * 6

    def set_max_speed(self, speed : int):
        self.max_speed = speed / 1000 * 82 #for some reason, when you specify 1000 as the max speed, it actually moves at 82 um/s

    def position(self, axis):
        '''
        Current position along an axis.

        Parameters
        ----------
        axis : axis number

        Returns
        -------
        The current position of the device axis in um.
        '''

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

    def absolute_move(self, x, axis):
        '''
        Moves the device axis to position x.

        Parameters
        ----------
        axis: axis number
        x : target position in um.
        '''

        if self.update_axis(axis):
            raise RuntimeError("Cannot move while another command is running")

        if self.min is None:
            self.setpoint[axis-1] = x
        else:
            self.setpoint[axis-1] = clip(x, self.min[axis-1], self.max[axis-1])
        
        self.cmd_time[axis-1] = time.time()
        self.speeds[axis-1] = self.max_speed * math.copysign(1, x - self.x[axis-1])

        print(f'moving to {x} at {self.speeds[axis-1]} um/s current pos: {self.x[axis-1]}')

        # if 1 <= axis <= 3:
        #     self.debug(f'Pipette Moving To: {self.setpoint[:3]}')
        # elif 4 <= axis <= 6:
        #     self.debug(f'Stage Moving To: {self.setpoint[3:]}')
        # else:
        #     self.debug(f"moving unknown axis: {axis}")
