import serial
import serial.rs485
import time

__all__ = ['CellSorterController']

class SerialCommands():
    ###
    # Serial CMDs for the Cell Sorter Control Unit (Controls LEDs and Valves)
    ###

    #Test Command
    TEST = 'a\r' #expected response: 'R'
    
    #LED Control
    LED_ON = 's5+\r' #expected response: 'OK'
    LED_OFF = 's5-\r' #expected response: 'OK'
    LED_STATUS = 's5?\r' #expected response: '+' or '-'

    #LED Ring Select
    LED_SELECT_RING_1 = 's1-\r' #expected response: 'OK'
    LED_SELECT_RING_2 = 's1+\r' #expected response: 'OK'

    #Valve Open / Close
    VALVE_1_OPEN = 's6+\r' #expected response: 'OK'
    VALVE_1_CLOSE = 's6-\r' #expected response: 'OK'
    VALVE_1_STATUS = 's6?\r' #expected response: '+' or '-'

    VALVE_2_OPEN = 's7+\r' #expected response: 'OK'
    VALVE_2_CLOSE = 's7-\r' #expected response: 'OK'
    VALVE_2_STATUS = 's7?\r' #expected response: '+' or '-'

    #Open Valve for a specified time
    VALVE_1_OPEN_TIME = 'i{}\r' #expected response: 10x number received
    VALVE_2_OPEN_TIME = 'j{}\r' #expected response: 10x number received

    #Open Valve for the last specified time
    VALVE_1_OPEN_PREV_TIME = 'I\r' #expected response: 'OK'
    VALVE_2_OPEN_PREV_TIME = 'J\r' #expected response: 'OK'

    #Specify Delay between valve openings
    VALVE_DELAY = 'k{}\r' #expected response: 10x number received

    #Open Valve 1, wait specified delay, open Valve 2
    VALVE_1_2_OPEN = 'K\r' #expected response: 'OK'

    #Open Valve 2, wait specified delay, open Valve 1
    VALVE_2_1_OPEN = 'L\r' #expected response: 'OK'

    ###
    # Serial CMDs for Cell Sorter Z-Manipulator Movement
    ###

    #Initialization
    GET_VERSION = '?ver\r' #expected response Vers:ES31.00.038\r or similar

    SET_PITCH = '!pitch 1.0 1.0 1.0\r' #no response, not clear what this does, maybe angle related?
    SET_CUR = '!cur 0.5 0.5 0.5\r' #no response, not clear what this does
    SET_REDUCTION = '!reduction 0.5 0.5 0.5\r' #no response, not clear what this does
    SET_CUR_DELAY = '!curdelay 10000 10000 10000\r' #no response, not clear what this does
    SET_SEC_VEL = '!secvel 50.0 50.0 50.0\r' #no response, not clear what this does

    #Movement
    SET_ACCEL = '!accel 0.1 0.1 {:.2f}\r' #no response, sets the z-axis accel, param in the range 0.02 (1% accel) to 2 (100% accel)
    SET_VEL = '!vel {:.13f} {:.13f} {:.13f}\r' #no response, sets the z-axis velocity, param in the range 0.05 (1% vel) to 50 (100% vel)
    SET_POS_ABS = '!moa 0 0 {:.3f}\r' #no response, absolute position in microns
    SET_POS_REL = '!mor 0 0 {:.16f}\r' #no response, relative position in microns
    JOY_2 = '!joy 2\r' #no response, this always follows a SET_POS_REL command
    GET_POS = '?pos\r' #expected response: 0.0000 0.0000 z-pos\r or similar
    EMERGENCY_STOP = 'a\r' #no response, stops all movement

    #Status
    READ_SW = '?readsw\r' #expected response: 000000000000\r or similar



class CellSorterController():

    def __init__(self, comPort: serial.Serial):
        self.comPort : serial.Serial = comPort

    def _sendCmd(self, cmd):
        for a in cmd:
            self.comPort.write(a.encode()) #send test command
        self.comPort.flush()
        time.sleep(0.1) #TODO: replace with something better

        resp = self.comPort.read_all() #read reply to message
        resp = resp.decode()
        resp = resp[:-3] #remove trailing \r\n\x00 at the end of each reply
        return resp
    
    def is_online(self):
        '''Sends a test command to the CellSorter and returns True if the response is correct
        '''
        resp = self._sendCmd(SerialCommands.TEST)
        return resp == 'R'

    def set_led(self, status: bool):
        '''Sets the LED status
        '''
        if status:
            self._sendCmd(SerialCommands.LED_ON)
        else:
            self._sendCmd(SerialCommands.LED_OFF)

    def get_led(self):
        '''Returns the LED status
        '''
        resp = self._sendCmd(SerialCommands.LED_STATUS)
        return resp == '+'
    
    def set_led_ring(self, ring: int):
        '''Sets the LED ring to use
        '''
        if ring == 1:
            self._sendCmd(SerialCommands.LED_SELECT_RING_1)
        elif ring == 2:
            self._sendCmd(SerialCommands.LED_SELECT_RING_2)
        else:
            raise ValueError('Ring must be 1 or 2')
    
    def set_valve(self, valve: int, open: bool):
        '''Sets the given valve to be open or closed
        '''
        if valve == 1:
            if open:
                self._sendCmd(SerialCommands.VALVE_1_OPEN)
            else:
                self._sendCmd(SerialCommands.VALVE_1_CLOSE)
        elif valve == 2:
            if open:
                self._sendCmd(SerialCommands.VALVE_2_OPEN)
            else:
                self._sendCmd(SerialCommands.VALVE_2_CLOSE)
        else:
            raise ValueError('Valve must be 1 or 2')

    def get_valve(self, valve: int):
        '''Returns the status of the given valve
        '''
        if valve == 1:
            resp = self._sendCmd(SerialCommands.VALVE_1_STATUS)
        elif valve == 2:
            resp = self._sendCmd(SerialCommands.VALVE_2_STATUS)
        else:
            raise ValueError('Valve must be 1 or 2')
        
        return resp == '+'

    def open_valve_for_time(self, valve: int, timeMs: int):
        '''Open a valve for the given time in milliseconds
        '''
        if valve == 1:
            self._sendCmd(SerialCommands.VALVE_1_OPEN_TIME.format(timeMs))
        elif valve == 2:
            self._sendCmd(SerialCommands.VALVE_2_OPEN_TIME.format(timeMs))
        else:
            raise ValueError('Valve must be 1 or 2')

    def open_valve_for_prev_time(self, valve: int):
        '''Open a valve for the last time specified
        '''
        if valve == 1:
            self._sendCmd(SerialCommands.VALVE_1_OPEN_PREV_TIME)
        elif valve == 2:
            self._sendCmd(SerialCommands.VALVE_2_OPEN_PREV_TIME)
        else:
            raise ValueError('Valve must be 1 or 2')
    
    def set_valve_delay(self, delayMs: int):
        '''Set the delay between valve openings
        '''
        self._sendCmd(SerialCommands.VALVE_DELAY.format(delayMs))
    
    def open_valve_1_2(self):
        '''Open valve 1, wait the specified delay, then open valve 2
        '''
        self._sendCmd(SerialCommands.VALVE_1_2_OPEN)
    
    def open_valve_2_1(self):
        '''Open valve 2, wait the specified delay, then open valve 1
        '''
        self._sendCmd(SerialCommands.VALVE_2_1_OPEN)

    def __del__(self):
        '''Close the serial connection when the object is destroyed
        '''
        self.comPort.close()


class CellSorterManip():

    def __init__(self, comPort: serial.Serial):
        self.comPort : serial.Serial = comPort

        self.maxVel = 25
        self.maxAcc = 2

        #set init commands
        self._sendCmd(SerialCommands.SET_PITCH)
        self._sendCmd(SerialCommands.SET_CUR)
        self._sendCmd(SerialCommands.SET_REDUCTION)
        self._sendCmd(SerialCommands.SET_CUR_DELAY)
        self._sendCmd(SerialCommands.SET_SEC_VEL)

    def _sendCmd(self, cmd):
        self.comPort.write(cmd.encode()) #send test command
        self.comPort.flush()
        time.sleep(0.1) #TODO: replace with something better

        resp = self.comPort.read_all() #read reply to message
        resp = resp.decode()
        resp = resp[:-3] #remove trailing \r\n\x00 at the end of each reply
        return resp
    
    def is_online(self):
        '''Sends a test command to the CellSorter and returns True if the response is correct
        '''
        resp = self._sendCmd(SerialCommands.GET_VERSION)
        return resp[:4] == 'Vers' # good responses start with Vers
    
    def set_pos_rel(self, pos: float):
        '''Sets the position relative to the current position
        '''
        self._sendCmd(SerialCommands.SET_VEL.format(self.maxVel, self.maxVel, self.maxVel))
        self._sendCmd(SerialCommands.SET_ACCEL.format(self.maxAcc))
        self._sendCmd(SerialCommands.SET_POS_REL.format(pos))
        self._sendCmd(SerialCommands.JOY_2)
    
    def set_pos_abs(self, pos: float):
        '''Sets the absolute position
        '''
        self._sendCmd(SerialCommands.SET_VEL.format(self.maxVel, self.maxVel, self.maxVel))
        self._sendCmd(SerialCommands.SET_ACCEL.format(self.maxAcc))
        self._sendCmd(SerialCommands.SET_POS_ABS.format(pos))

    def emergency_stop(self):
        '''Stops all movement immediately
        '''
        self._sendCmd(SerialCommands.EMERGENCY_STOP)
    
    def get_position(self):
        '''Returns the current position of the z-axis
        '''
        resp = self._sendCmd(SerialCommands.GET_POS)
        resp = resp.split(' ')
        zPos = float(resp[2])
        return zPos
        
    def set_velocity(self, vel: float):
        '''Sets the velocity of the z-axis (velocity as a % from 0.1 to 100)
        '''
        if vel < 0.1 or vel > 100:
            raise ValueError('Velocity must be between 0.1 and 100')
        
        nativeVel = vel / 2

        self.maxVel = nativeVel
        self._sendCmd(SerialCommands.SET_VEL.format(self.maxVel, self.maxVel, self.maxVel))
    
    def set_accel(self, accel: float):
        '''Sets the acceleration of the z-axis from 1% to 100%
        '''
        if accel < 1 or accel > 100:
            raise ValueError('Acceleration must be between 1 and 100')

        nativeAccel = accel / 50
        self.maxAcc = nativeAccel
        self._sendCmd(SerialCommands.SET_ACCEL.format(self.maxAcc))

    def read_SW(self):
        '''Returns the current state of the switches
        '''
        resp = self._sendCmd(SerialCommands.READ_SW)
        return resp

    def __del__(self):
        '''Close the serial connection when the object is destroyed
        '''
        self.comPort.close()

if __name__ == '__main__':

    testController = False

    if testController:

        # Create a serial connection to the CellSorter
        # As per documentation: 115200 baud, 8 data bits, no parity, 1 stop bit
        controllerSerial = serial.Serial('COM7', 115200, timeout=2, parity=serial.PARITY_NONE, stopbits=1, 
                                            bytesize=8, write_timeout=1, inter_byte_timeout=2)
        # controllerSerial.dtr = False
        # controllerSerial.rts = False

        cellSorterController = CellSorterController(controllerSerial)

        # Check that the CellSorter is online
        if not cellSorterController.is_online():
            print('CellSorter not online')
            exit(1)
        
        # Set the LED ring to use
        cellSorterController.set_led_ring(1)

        # Flicker the LED on and off 5 times
        for i in range(5):
            cellSorterController.set_led(True)
            time.sleep(0.5)
            cellSorterController.set_led(False)
            time.sleep(0.5)

        #close serial
        del cellSorterController
    
    else:
        manipulatorSerial = serial.Serial('COM14', 57600, timeout=2, parity=serial.PARITY_NONE, stopbits=2, 
                                            bytesize=8, write_timeout=1, inter_byte_timeout=2)
        # manipulatorSerial.dtr = True
        # manipulatorSerial.rts = False

        cellSorterManip = CellSorterManip(manipulatorSerial)

        # Check that the CellSorter is online
        if not cellSorterManip.is_online():
            print('CellSorter not online')
            exit(1)
        
        for i in range(3):
            # move down
            for i in range(3):
                cellSorterManip.set_pos_rel(-1)
                time.sleep(0.5)
            
            #move up
            for i in range(3):
                cellSorterManip.set_pos_rel(1)
                time.sleep(0.5)

        #close serial
        del cellSorterManip

