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

        print(f'Using {self.readDev}/{self.readChannel} for reading and {self.cmdDev}/{self.cmdChannel} for writing.')

    def _readAnalogInput(self, samplesPerSec, recordingTime):
        numSamples = int(samplesPerSec * recordingTime)
        with nidaqmx.Task() as task:
            task.ai_channels.add_ai_voltage_chan(f'{self.readDev}/{self.readChannel}', max_val=10, min_val=0)
            task.timing.cfg_samp_clk_timing(samplesPerSec, sample_mode=nidaqmx.constants.AcquisitionType.FINITE, samps_per_chan=numSamples)
            # task.triggers.reference_trigger.cfg_anlg_edge_ref_trig(f'{self.readDev}/{self.readChannel}', pretrigger_samples = 10, trigger_slope=nidaqmx.constants.Slope.RISING, trigger_level=0.2)
            data = task.read(number_of_samples_per_channel=numSamples, timeout=10)
            data = np.array(data, dtype=float)
            task.stop()

        #check for None values
        if data is None or np.where(data == None)[0].size > 0:
            data = np.zeros(self.numSamples)

        #convert from V to pA
        data = data * 2000

        #convert from pA to Amps
        data = data * 1e-12
            
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
        data = self._readAnalogInput(samplesPerSec, recordingTime)
        sendTask.stop()
        sendTask.close()
        
        data, resistance = self._triggerSquareWave(1e-9, data, amplitude * 0.02)
        triggeredSamples = data.shape[0]
        xdata = np.linspace(0, triggeredSamples / samplesPerSec, triggeredSamples, dtype=float)

        return np.array([xdata, data]), resistance
    
    def _triggerSquareWave(self, triggerVal, data, cmdVoltage):
        try:
            #split data into high and low wave
            mean = np.mean(data)
            highAvg = np.mean(data[data > mean])
            lowAvg = np.mean(data[data < mean])

            #set data mean to 0
            shiftedData = data - lowAvg

            #is trigger value ever reached?
            if np.where(shiftedData < triggerVal)[0].size == 0 or np.where(shiftedData > triggerVal)[0].size == 0:
                return data, None

            #find the first index where the data is less than triggerVal
            fallingEdge = np.where(shiftedData < triggerVal)[0][0]
            shiftedData = shiftedData[fallingEdge:]

            #find the first index where the data is greater than triggerVal
            risingEdge = np.where(shiftedData > triggerVal)[0][0]
            shiftedData = shiftedData[risingEdge:]

            #find second rising edge location
            secondFallingEdge = np.where(shiftedData < triggerVal)[0][0]
            
            #find second rising edge after falling edge
            secondRisingEdge = np.where(shiftedData[secondFallingEdge:] > triggerVal)[0][0] + secondFallingEdge
            shiftedData = shiftedData[:secondRisingEdge]

            #calculate resistance
            resistance = cmdVoltage / (highAvg - lowAvg)

            return shiftedData + lowAvg, resistance
        except:
            #we got an invalid square wave
            return data, None
        
    
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