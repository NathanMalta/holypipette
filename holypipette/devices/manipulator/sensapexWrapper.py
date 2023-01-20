from sensapex import UMP
import numpy as np
from ctypes import c_int, c_float, byref
import math

from holypipette.devices.manipulator.manipulator import Manipulator

class SensapexManip(Manipulator):
    '''A wrapper class to interface between the sensapex python library and the holypipette minipulator classes
    '''
    
    def __init__(self, deviceID = None):
        Manipulator.__init__(self)
        self.ump = UMP.get_ump()

        #setup device ID
        if deviceID == None:
            umpList = self.ump.list_devices()
            assert(len(umpList) == 1, "must specify sensapex ump device id if there is more than 1 connected!")
            self.deviceID = umpList[0] #if there's only 1 device connected, use it
        else:
            self.deviceID = deviceID

        self.max_speed = 10 # "feels good" default value
        self.max_acceleration = 20 # "feels good" default value
        self.armAngle = math.radians(self._get_axis_angle())

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
        pos = self.ump.get_pos(self.deviceID, timeout=1)
        pos[0] = pos[0] * math.cos(self.armAngle) #convert to "virtual" x-axis, parallel to the cover slip
        
        if axis == None:
            return pos
        else:
            return pos[axis-1]

    def absolute_move(self, x, axis):
        '''
        Moves the device axis to position x.

        Parameters
        ----------
        axis: axis number
        x : target position in um.
        '''
        newPos = np.empty((3,)) * np.nan
        if axis == 1:
            #we're dealing with the 'virtual' d-axis
            newPos = self.ump.get_pos(self.deviceID, timeout=1)
            currD = newPos[0] * math.cos(self.armAngle)
            print(f'curr d: {currD} desired D: {x}')
            dD = currD - x
            print(f'dx: {dD / math.cos(self.armAngle)}\t dz: {-dD * math.tan(self.armAngle)}')

            newPos[0] += dD / math.cos(self.armAngle)
            newPos[2] += -dD * math.tan(self.armAngle)
        else:
            #we're dealing with a physical axis, it can be commanded directly
            newPos[axis-1] = x
        
        print(f'moving to {newPos}')

        self.ump.goto_pos(self.deviceID, newPos, self.max_speed, max_acceleration=self.max_acceleration)

    def absolute_move_group(self, x, axes):
        '''
        Moves the device group of axes to position x.

        Parameters
        ----------
        axes : list of axis numbers
        x : target position in um (vector or list).
        '''
        newPos = self.ump.get_pos(self.deviceID, timeout=1)
        currPos = newPos.copy()
        if 1 in axes:
            indx = list(axes).index(1)
            dx = x[indx] - currPos[0]
            #handle virtual axis
            newPos[0] += dx / math.cos(self.armAngle)
            newPos[2] += -dx * math.tan(self.armAngle)

        for pos, axis in zip(x, axes):
            if axis == 1:
                continue #we already handled this case.
            
            dPos = currPos[axis-1] - pos
            newPos[axis-1] += dPos

        print(x, axes, newPos, self.ump.get_pos(self.deviceID, timeout=1), currPos)
        self.ump.goto_pos(self.deviceID, newPos, self.max_speed, max_acceleration=self.max_acceleration)
        

    def position_group(self, axes):
        '''
        Current position along a group of axes.

        Parameters
        ----------
        axes : list of axis numbers

        Returns
        -------
        The current position of the device axis in um (vector).
        '''
        return np.array(self.ump.get_pos(self.deviceID, timeout=1))[np.array(axes)-1]

    def stop(self, axis):
        """
        Stops current movements.
        """
        self.ump.stop()

    def _get_axis_angle(self):
        angle = c_float()
        rVal = self.ump.call("ump_get_axis_angle", self.deviceID, byref(angle))
        return angle.value

    def set_max_speed(self, speed):
        self.max_speed = speed
    
    def set_max_accel(self, accel):
        self.max_acceleration = accel