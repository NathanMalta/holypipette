import serial
import serial.rs485
import time

__all__ = ['CellSorterController']

class SerialCommands():
    #Test Command
    TEST = 'a\x0D' #expected response: 'R'
    
    #LED Control
    LED_ON = 's5+\r\n' #expected response: 'OK'
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


class CellSorterController():

    def __init__(self, comPort: serial.Serial):
        self.comPort : serial.Serial = comPort

    def _sendCmd(self, cmd):
        self.comPort.flushInput()
        self.comPort.flushOutput()
        self.comPort.write(bytes(cmd, 'ascii'))
        time.sleep(1) #TODO: replace with something better

        resp = self.comPort.read_all() #read reply to message
        # resp = resp[:-1] #remove trailing \r
        return resp.decode()
    
    def is_online(self):
        '''Sends a test command to the CellSorter and returns True if the response is correct
        '''
        resp = self._sendCmd(SerialCommands.TEST)
        print(f'online resp: {resp}')
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

if __name__ == '__main__':
    # Create a serial connection to the CellSorter
    # As per documentation: 115200 baud, 8 data bits, no parity, 1 stop bit
    cellSorterSerial = serial.Serial('COM5', 9600, timeout=0.4, parity=serial.PARITY_NONE, stopbits=1.5, 
                                        bytesize=8, write_timeout=1, inter_byte_timeout=0.4, 
                                        exclusive=True, dsrdtr=False, rtscts=False, xonxoff=False)


    # cellSorterSerial2 = serial.Serial('COM14', 115200, timeout=0.4, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, 
    #                                     bytesize=serial.SEVENBITS, write_timeout=1, inter_byte_timeout=0.4, 
    #                                     exclusive=True, dsrdtr=False, rtscts=False, xonxoff=False)
    time.sleep(1)

    #set EOF to 0x1A

    cellSorter = CellSorterController(cellSorterSerial)

    # Check that the CellSorter is online
    # if not cellSorter.is_online():
    #     print('CellSorter not online')
    #     exit(1)
    
    # Set the LED ring to use
    cellSorter.set_led_ring(1)

    # Flicker the LED on and off 5 times
    # for i in range(5):
    cellSorter.set_led_ring(1)
    cellSorter.set_led(False)
    time.sleep(1)
    cellSorter.set_led(True)
    time.sleep(10)

    #close serial
    del cellSorter

