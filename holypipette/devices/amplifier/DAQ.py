import nidaqmx
import nidaqmx.system
import nidaqmx.constants
from holypipette.devices.camera import WorldModel

import numpy as np
import math

__all__ = ['DAQ', 'FakeDAQ']

class DAQ:
    def __init__(self, readDev, readChannel, cmdDev, cmdChannel):
        self.readDev = readDev
        self.cmdDev = cmdDev

        self.readChannel = readChannel
        self.cmdChannel = cmdChannel

        #read constants
        self.samplesPerSec = 5000
        self.readTime = 0.2
        self.numSamples = int(self.samplesPerSec * self.readTime)

        print(f'Using {self.readDev}/{self.readChannel} for reading and {self.cmdDev}/{self.cmdChannel} for writing.')

    def _readAnalogInput(self):
        with nidaqmx.Task() as task:
            task.ai_channels.add_ai_voltage_chan(f'{self.readDev}/{self.readChannel}', max_val=10, min_val=0)
            task.timing.cfg_samp_clk_timing(self.samplesPerSec, sample_mode=nidaqmx.constants.AcquisitionType.FINITE, samps_per_chan=self.numSamples)
            # task.triggers.reference_trigger.cfg_anlg_edge_ref_trig(f'{self.readDev}/{self.readChannel}', pretrigger_samples = 10, trigger_slope=nidaqmx.constants.Slope.RISING, trigger_level=0.2)
            data = task.read(number_of_samples_per_channel=self.numSamples, timeout=10)
            data = np.array(data, dtype=float)
            task.stop()

            #turn all infinite and nan values to 0
            data = np.nan_to_num(data, nan=0, posinf=0, neginf=0)

        #check for None values
        if data is None or np.where(data == None)[0].size > 0:
            data = np.zeros(self.numSamples)
            
        return data
        
    def _sendSquareWave(self, wave_freq, samplesPerSec, dutyCycle, amplitude, recordingTime):
        task = nidaqmx.Task()
        task.ao_channels.add_ao_voltage_chan(f'{self.cmdDev}/{self.cmdChannel}')
        task.timing.cfg_samp_clk_timing(samplesPerSec, sample_mode=nidaqmx.constants.AcquisitionType.CONTINUOUS)
        
        #create a wave_freq Hz square wave
        data = np.zeros(int(samplesPerSec * recordingTime))
        onTime = (1 / wave_freq) * dutyCycle * samplesPerSec #in samples
        offTime = (1 / wave_freq) * (1-dutyCycle) * samplesPerSec

        #calc period
        period = onTime + offTime #in samples

        #convert to int
        onTime = int(onTime)
        offTime = int(offTime)
        period = int(period)

        numWaves = int(data.shape[0] / period)

        for i in range(numWaves):
            data[i * period : i * period + onTime] = amplitude

        task.write(data)
        
        return task
    
    def getDataFromSquareWave(self, wave_freq, samplesPerSec, dutyCycle, amplitude, recordingTime):

        #grab data from DAQ
        sendTask = self._sendSquareWave(wave_freq, samplesPerSec, dutyCycle, amplitude, recordingTime)
        sendTask.start()
        raw_data = self._readAnalogInput()
        sendTask.stop()
        sendTask.close()

        #process data -- add triggering and y shift to low = 0

        mean = np.mean(raw_data)

        #split array into greater than and less than mean (high and low portions of squarewave)
        low_values = raw_data[:, raw_data < mean]
        high_values = raw_data[:, raw_data > mean]

        low_mean = np.mean(low_values)
        high_mean = np.mean(high_values)

        # set low to mean 0
        raw_data -= low_mean
        triggerSpots = np.where(raw_data[1, :] > self.triggerLevel)[0]
        lowSpots = np.where(raw_data[1, :] < 0)[0]

        #find first rising edge (first low to high transition)
        if len(lowSpots) == 0 or len(triggerSpots) == 0:
            print("no rising edge found")
            return raw_data
        try:
            # get rising edge location (first trigger spot after first low spot)
            rising_edge = triggerSpots[triggerSpots > lowSpots[0]][0]
            falling_edge = lowSpots[lowSpots > rising_edge][0]
            second_rising_edge = triggerSpots[triggerSpots > falling_edge][0]

            timePerSample = recordingTime / self.numSamples

            # trim data to rising edge
            squarewave = raw_data[:, rising_edge:second_rising_edge]

            self.lastestDaqData = squarewave
        except:
            return raw_data #could not trigger to rising edge

        xdata = np.linspace(0, recordingTime, self.numSamples, dtype=float)

        return np.array([xdata, data])

class FakeDAQ:
    def __init__(self, worldModel):
        self.worldModel: WorldModel = worldModel

    def getDataFromSquareWave(self, wave_freq, samplesPerSec, dutyCycle, amplitude, recordingTime):
        #create a wave_freq Hz square wave
        data = np.zeros(int(samplesPerSec / recordingTime))
        onTime = 1 / wave_freq * dutyCycle * samplesPerSec
        offTime = 1 / wave_freq * (1-dutyCycle) * samplesPerSec

        #calc period
        period = onTime + offTime

        #convert to int
        onTime = int(onTime)
        offTime = int(offTime)
        period = int(period)

        wavesPerSec = samplesPerSec // period

        for i in range(wavesPerSec):
            data[i * period : i * period + onTime] = amplitude


        xdata = np.linspace(0, recordingTime, len(data), dtype=float)

        data = np.array([xdata, data])
        return data

    def getResistance(self):
        return self.worldModel.getResistance()