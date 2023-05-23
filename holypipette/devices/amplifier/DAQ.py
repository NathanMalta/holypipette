import nidaqmx
import nidaqmx.system
import nidaqmx.constants

import numpy as np
import scipy.signal as signal
import math
import time

__all__ = ['DAQ', 'FakeDAQ']

class DAQ:
    def __init__(self, readDev, readChannel, cmdDev, cmdChannel):
        self.readDev = readDev
        self.cmdDev = cmdDev

        self.readChannel = readChannel
        self.cmdChannel = cmdChannel
        self.latestResistance = None

        #read constants

        print(f'Using {self.readDev}/{self.readChannel} for reading and {self.cmdDev}/{self.cmdChannel} for writing.')

    def _readAnalogInput(self, samplesPerSec, recordingTime):
        numSamples = int(samplesPerSec * recordingTime)
        with nidaqmx.Task() as task:
            task.ai_channels.add_ai_voltage_chan(f'{self.readDev}/{self.readChannel}', max_val=10, min_val=0, terminal_config=nidaqmx.constants.TerminalConfiguration.DIFF)
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
        
        data, self.latestResistance = self._triggerSquareWave(150e-12, data, amplitude * 0.02)
        triggeredSamples = data.shape[0]
        xdata = np.linspace(0, triggeredSamples / samplesPerSec, triggeredSamples, dtype=float)

        return np.array([xdata, data]), self.latestResistance
    
    def resistance(self):
        return self.latestResistance

    def _filter60Hz(self, data):
        samplesPerSec = 50000
        #60 hz filter
        b, a = signal.iirnotch(48.828125, Q=30, fs=samplesPerSec)
        data = signal.filtfilt(b, a, data, irlen=1000)
        return data

    
    def _triggerSquareWave(self, triggerVal, data, cmdVoltage):
        # #add 60 hz filter
        # data = self._filter60Hz(data)
        # # return data, None #completely disable triggering
        
        # #print most prominant frequency in the data
        # freqs, psd = signal.welch(data, fs=50000, nperseg=2048)
        
        # # #get 10 frequencies with the highest powers
        # freqs = freqs[np.argsort(psd)][-10:]
        # psd = psd[np.argsort(psd)][-10:]

        # print(freqs)

        # return data, None

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
            
            #find peak to peak spread on high side
            highPeak = np.max(shiftedData[5:secondFallingEdge-5:])
            lowPeak = np.min(shiftedData[5:secondFallingEdge-5:])
            peakToPeak = highPeak - lowPeak
            # print(f'Peak to peak: {peakToPeak * 1e12} ({highPeak * 1e12} - {lowPeak * 1e12})')

            #find second rising edge after falling edge
            secondRisingEdge = np.where(shiftedData[secondFallingEdge:] > triggerVal)[0][0] + secondFallingEdge
            shiftedData = shiftedData[:secondRisingEdge]

            #calculate resistance
            resistance = cmdVoltage / (highAvg - lowAvg)

            return shiftedData, resistance
        except:
            #we got an invalid square wave
            return data, None
        
    
class FakeDAQ:
    def __init__(self):
        self.latestResistance = 6 * 10 ** 6

    def resistance(self):
        return self.latestResistance * math.sin(time.time() / 20) + self.latestResistance + 4*10**6

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

        data = np.array([xdata, data]), self.resistance()
        return data