from sensapex import UMP
import numpy as np
from ctypes import c_int, c_float, byref
import math
import time

from holypipette.devices.manipulator.manipulator import Manipulator

class SensapexManip(Manipulator):
    '''A wrapper class to interface between the sensapex python library and the holypipette minipulator classes
    '''
    
    def __init__(self, deviceID = None):
        Manipulator.__init__(self)
        print('setting library path...')
        UMP.set_library_path('/usr/local/lib/libum.dylib') #only for mac
        self.ump = UMP.get_ump()
        print('devices: ', self.ump.list_devices())
        time.sleep(1)
        print('devices: ', self.ump.list_devices())
        

        #setup device ID
        if deviceID == None:
            umpList = self.ump.list_devices()
            assert(len(umpList) == 1, "must specify sensapex ump device id if there is more than 1 connected!")
            self.deviceID = umpList[0] #if there's only 1 device connected, use it
        else:
            self.deviceID = deviceID

        self.max_speed = 1000 # "feels good" default value
        self.max_acceleration = 20 # "feels good" default value
        self.armAngle = math.radians(-self._get_axis_angle())

        self.raw_to_real_mat  = np.array([[np.cos(self.armAngle), 0, 0], 
                                          [0, 1, 0], 
                                          [-np.sin(self.armAngle), 0, 1]], dtype=np.float32)

        self.real_to_raw_mat = np.linalg.inv(self.raw_to_real_mat)

    def raw_to_real(self, raw_pos : np.ndarray):
        raw_pos = raw_pos.copy()
        real_pos = np.matmul(self.raw_to_real_mat, np.array(raw_pos).T)
        return real_pos

    def real_to_raw(self, real_pos : np.ndarray):
        real_pos = real_pos.copy()
        raw_pos = np.matmul(self.real_to_raw_mat, np.array(real_pos).T)
        return raw_pos

    def position(self, axis=None):
        raw_pos = self.raw_position()
        real_pos = self.raw_to_real(raw_pos)
        
        if axis == None:
            return real_pos
        else:
            return real_pos[axis-1]

    def raw_position(self, axis=None):
        return self.ump.get_pos(self.deviceID, timeout=1)

    def absolute_move(self, x, axis):
        # print(f"Moving axis {axis} to {x}\t{self.position()}\t{self.raw_position()}")
        new_setpoint_raw = np.empty((3,)) * np.nan
        if axis == 1:
            #we're dealing with the 'virtual' d-axis
            new_setpoint_raw = self.raw_position()
            curr_pos_real = self.position()
            dx = x - curr_pos_real[0]
            dVect = self.real_to_raw(np.array([dx, 0, 0]))
            new_setpoint_raw += dVect
        elif axis == 3:
            #we're dealing with z - needs to be handeled based on offset
            new_setpoint_raw = self.raw_position()
            curr_pos_real = self.position()
            dz = x - curr_pos_real[2]
            dVect = self.real_to_raw(np.array([0, 0, dz]))
            new_setpoint_raw += dVect
        else:
            #we're dealing with a physical axis, it can be commanded directly
            new_setpoint_raw[axis-1] = x

        self.ump.goto_pos(self.deviceID, new_setpoint_raw, self.max_speed, max_acceleration=self.max_acceleration, linear=True)


    # def absolute_move_group(self, x, axes):
    #     new_setpoint_raw = self.raw_position()
    #     curr_pos_real = self.position()
    #     print(f'curr pos {curr_pos_real} req: {x}')
    #     x = np.array(x)
    #     axes = np.array(axes)

    #     if 1 in axes:
    #         #we're dealing with the 'virtual' d-axis
    #         indx = np.where(axes == 1)[0][0]
    #         dx = x[indx] - curr_pos_real[0]
    #         print('dx2', dx)
    #         dVect = self.real_to_raw(np.array([dx, 0, 0]))
    #         new_setpoint_raw += dVect
    #     if 2 in axes:
    #         indy = np.where(axes == 2)[0][0]
    #         dy = x[indy] - curr_pos_real[1]
    #         print('dy2', dy)
    #         new_setpoint_raw[1] += dy
    #     if 3 in axes:
    #         #we're dealing with z - needs to be handeled based on offset
    #         indz = np.where(axes == 3)[0][0]
    #         dz = x[indz] - curr_pos_real[2]
    #         print('dz2', dz)
    #         dVect = self.real_to_raw(np.array([0, 0, dz]))
    #         new_setpoint_raw += dVect

    #     print('move group', new_setpoint_raw, self.raw_position())
    #     self.ump.goto_pos(self.deviceID, new_setpoint_raw, self.max_speed, max_acceleration=self.max_acceleration, linear=True)

        
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