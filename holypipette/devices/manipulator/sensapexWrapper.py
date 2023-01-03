from sensapex import UMP
import numpy as np

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

        self.max_speed = 100 # "feels good" default value
        self.max_acceleration = 20 # "feels good" default value

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
        return self.ump.get_pos(self.deviceID, timeout=1)[axis-1]

    def absolute_move(self, x, axis):
        '''
        Moves the device axis to position x.

        Parameters
        ----------
        axis: axis number
        x : target position in um.
        '''
        newPos = np.empty((3,)) * np.nan
        newPos[axis-1] = x
        self.ump.goto_pos(self.deviceID, newPos, self.max_speed, max_acceleration=self.max_acceleration)

    def absolute_move_group(self, x, axes):
        '''
        Moves the device group of axes to position x.

        Parameters
        ----------
        axes : list of axis numbers
        x : target position in um (vector or list).
        '''
        newPos = np.empty((3,)) * np.nan
        for i, pos, axis in enumerate(zip(x, axes)):
            newPos[axis-1] = pos
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

    def set_max_speed(self, speed):
        self.max_speed = speed
    
    def set_max_accel(self, accel):
        self.max_acceleration = accel