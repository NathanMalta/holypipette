import nidaqmx
import nidaqmx.system
import nidaqmx.constants

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
        sendTask = self._sendSquareWave(wave_freq, samplesPerSec, dutyCycle, amplitude, recordingTime)
        sendTask.start()
        data = self._readAnalogInput()
        sendTask.stop()
        sendTask.close()

        xdata = np.linspace(0, recordingTime, self.numSamples, dtype=float)

        return np.array([xdata, data])

class FakeDAQ:
    def __init__(self):
        pass

    def getDataFromSquareWave(self, wave_freq, samplesPerSec, dutyCycle, amplitude, recordingTime):
        ydata = np.zeros(samplesPerSec)
        onTime = 1 / wave_freq * dutyCycle * samplesPerSec
        offTime = 1 / wave_freq * (1-dutyCycle) * samplesPerSec

        #calc period
        period = onTime + offTime

        #convert to int
        onTime = int(onTime)
        offTime = int(offTime)
        period = int(period)

        wavesPerSec = samplesPerSec // period
        numWaves = math.ceil(recordingTime * wavesPerSec)

        #make up some fake data
        for i in range(numWaves):
            ydata[i * period : i * period + onTime] = amplitude / 10

        xdata = np.linspace(0, recordingTime, samplesPerSec)

        data = np.array([xdata, ydata])
        return data