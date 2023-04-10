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
        self.numSamples = 2000

        print(f'Using {self.readDev}/{self.readChannel} for reading and {self.cmdDev}/{self.cmdChannel} for writing.')

    def _readAnalogInput(self):
        with nidaqmx.Task() as task:
            task.ai_channels.add_ai_voltage_chan(f'{self.readDev}/{self.readChannel}', max_val=10, min_val=0)
            task.timing.cfg_samp_clk_timing(20000, sample_mode=nidaqmx.constants.AcquisitionType.FINITE, samps_per_chan=self.numSamples)
            task.triggers.reference_trigger.cfg_anlg_edge_ref_trig(f'{self.readDev}/{self.readChannel}', pretrigger_samples = 10, trigger_slope=nidaqmx.constants.Slope.RISING, trigger_level = 0.1 )
            data = task.read(number_of_samples_per_channel=2000)
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

        task.write(data)
        
        return task
    
    def getDataFromSquareWave(self, wave_freq, samplesPerSec, dutyCycle, amplitude, recordingTime):
        sendTask = self._sendSquareWave(wave_freq, samplesPerSec, dutyCycle, amplitude, recordingTime)
        sendTask.start()
        data = self._readAnalogInput()
        sendTask.stop()
        sendTask.close()

        xdata = np.linspace(0, samplesPerSec * recordingTime, self.numSamples, dtype=float)

        return np.array([xdata, data])

class FakeDAQ:
    def __init__(self):
        pass

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