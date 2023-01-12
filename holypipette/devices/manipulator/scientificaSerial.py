import serial
from .manipulator import Manipulator
import time

__all__ = ['ScientificaSerial']

class SerialCommands():
    GET_X_POS = 'PX\r'
    GET_Y_POS = 'PY\r'
    GET_Z_POS = 'PZ\r'
    GET_X_Y_Z = '\r'
    GET_MAX_SPEED = 'TOP\r'
    GET_MAX_ACCEL = 'ACC\r'
    GET_IS_BUSY = 's\r' #TODO: is this caps?

    SET_X_Y_POS = 'abs {} {}\r'
    SET_Z_POS = 'absz {}\r'
    SET_MAX_SPEED = 'TOP {}\r'
    SET_MAX_ACCEL = 'ACC {}\r'

class ScientificaSerial(Manipulator):

    def __init__(self, comPort: serial.Serial):
        self.comPort : serial.Serial = comPort
    
    def set_max_speed(self, speed):
        '''Sets the max speed for the Scientifica Stage.  
           It seems like the range for this is around (1000, 100000)
        '''
        self.comPort.write(SerialCommands.SET_MAX_SPEED.format(int(speed)))

    def set_max_accel(self, accel):
        '''Sets the max acceleration for the Scientifica Stage.
           It seems like the range for this is around (10, 10000)
        '''
        self.comPort.write(SerialCommands.SET_MAX_ACCEL.format(int(accel)))

    def __del__(self):
        self.comPort.close()

    def _sendCmd(self, cmd, waitForResponse = True):
        self.comPort.read_all() #clear out the buffer as to not get old messages
        self.comPort.write(cmd)

        if waitForResponse:
            resp = self.comPort.read_until('\r') #read reply to message
            return resp.decode()

    def position(self, axis=None):
        if axis == 1:
            xpos = self._sendCmd(SerialCommands.GET_X_POS)
            return int(xpos)
        if axis == 2:
            ypos = self._sendCmd(SerialCommands.GET_Y_POS)
            return int(xpos)
        if axis == 3:
            zpos = self._sendCmd(SerialCommands.GET_Z_POS)
            return int(xpos)
        if axis == None:
            xyz = self._sendCmd(SerialCommands.GET_X_Y_Z)
            
            xPos = int(xyz[0])
            yPos = int(xyz[1])
            zPos = int(xyz[2])

            return [xPos, yPos, zPos]

    def absolute_move(self, pos, axis):
        if axis == 1:
            yPos = self.position(axis=2)
            self._sendCmd(SerialCommands.SET_X_Y_POS.format(int(pos), yPos), waitForResponse=False)
        if axis == 2:
            xPos = self.position(axis=1)
            self._sendCmd(SerialCommands.SET_X_Y_POS.format(xPos, int(pos)), waitForResponse=False)
        if axis == 3:
            self._sendCmd(SerialCommands.SET_Z_POS.format(pos), waitForResponse=False)

    def relative_move(self, x, axis):
        pass #TODO: there's a command for this but I forgot to record it

    def wait_until_still(self, axes = None, axis = None):
        while True:
            resp = self._sendCmd(SerialCommands.GET_IS_BUSY)
            busy = resp != '0'

            if not busy:
                break

    def stop(self):
        pass #TODO is there a command for this?