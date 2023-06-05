import nidaqmx
import nidaqmx.system
import nidaqmx.constants

import numpy as np
import scipy.signal as signal
import math
import time
import threading

__all__ = ['DAQ', 'FakeDAQ']

class DAQ:
    C_CLAMP_AMP_PER_VOLT = 400 * 1e-12 #400 pA (supplied cell) / V (DAQ out)
    C_CLAMP_VOLT_PER_VOLT = (10 * 1e-3) / (1e-3) #10 mV (DAQ input) / V (cell out)

    V_CLAMP_VOLT_PER_VOLT = (20 * 1e-3) #20 mV (supplied cell) / V (DAQ out)
    V_CLAMP_VOLT_PER_AMP = 0.5 / 1e-9 #0.5V DAQ out (DAQ input) / nA (cell out)

    def __init__(self, readDev, readChannel, cmdDev, cmdChannel):
        self.readDev = readDev
        self.cmdDev = cmdDev

        self.readChannel = readChannel
        self.cmdChannel = cmdChannel
        self.latestResistance = None
        self.isRunningCurrentProtocol = False
        self._deviceLock = threading.Lock()

        self.latest_protocol_data = None

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
            data[i * period : i * period + onTime] = 0
            data[i * period + onTime : (i+1) * period] = amplitude

        task.write(data)
        
        return task
    
    def getDataFromCurrentProtocol(self, startCurrentPicoAmp=-200, endCurrentPicoAmp=300, stepCurrentPicoAmp=100, highTimeMs=400):
        '''Sends a series of square waves from startCurrentPicoAmp to endCurrentPicoAmp (inclusive) with stepCurrentPicoAmp pA increments.
           Square wave period is 2 * highTimeMs ms. Returns a 2d array of data with each row being a square wave.
        '''

        self.isRunningCurrentProtocol = True
        self.latest_protocol_data = None #clear data
        num_waves = int((endCurrentPicoAmp - startCurrentPicoAmp) / stepCurrentPicoAmp) + 1

        #convert to amps
        startCurrent = startCurrentPicoAmp * 1e-12

        #get wave frequency Hz
        wave_freq = 1 / (2 * highTimeMs * 1e-3)

        #general constants for square waves
        samplesPerSec = 50000
        recordingTime = 3 * highTimeMs * 1e-3

        for i in range(num_waves):
            currentAmps = startCurrent + i * stepCurrentPicoAmp * 1e-12
            print(f'Sending {currentAmps * 1e12} pA square wave.')

            #convert to DAQ output
            amplitude = currentAmps / self.C_CLAMP_AMP_PER_VOLT
            
            #send square wave to DAQ
            self._deviceLock.acquire()
            sendTask = self._sendSquareWave(wave_freq, samplesPerSec, 0.5, amplitude, recordingTime)
            sendTask.start()
            data = self._readAnalogInput(samplesPerSec, recordingTime)
            sendTask.stop()
            sendTask.close()
            self._deviceLock.release()

            #convert to V (cell out)
            data = data / self.C_CLAMP_VOLT_PER_VOLT
            
            lowZero = currentAmps > 0
            data = self._shiftWaveToZero(data, lowZero)
            triggeredSamples = data.shape[0]
            xdata = np.linspace(0, triggeredSamples / samplesPerSec, triggeredSamples, dtype=float)
            time.sleep(0.5)

            if self.latest_protocol_data is None:
                self.latest_protocol_data = [[xdata, data]]
            else:
                self.latest_protocol_data.append([xdata, data])
        
        self.isRunningCurrentProtocol = False

        return self.latest_protocol_data

    
    def getDataFromSquareWave(self, wave_freq, samplesPerSec, dutyCycle, amplitude, recordingTime):
        self._deviceLock.acquire()
        sendTask = self._sendSquareWave(wave_freq, samplesPerSec, dutyCycle, amplitude, recordingTime)
        sendTask.start()
        data = self._readAnalogInput(samplesPerSec, recordingTime)
        sendTask.stop()
        sendTask.close()
        self._deviceLock.release()
        
        data, self.latestResistance = self._triggerSquareWave(data, amplitude * 0.02)
        triggeredSamples = data.shape[0]
        xdata = np.linspace(0, triggeredSamples / samplesPerSec, triggeredSamples, dtype=float)

        #convert from V to pA
        data = data * 2000

        #convert from pA to Amps
        data = data * 1e-12

        return np.array([xdata, data]), self.latestResistance
    
    def resistance(self):
        return self.latestResistance

    def _filter60Hz(self, data):
        samplesPerSec = 50000
        #60 hz filter
        b, a = signal.iirnotch(48.828125, Q=30, fs=samplesPerSec)
        data = signal.filtfilt(b, a, data, irlen=1000)
        return data

    def _shiftWaveToZero(self, data, lowZero=True):
        mean = np.mean(data)

        if lowZero:
            zeroAvg = np.mean(data[data < mean])
        else:
            zeroAvg = np.mean(data[data > mean])

        #set data mean to 0
        shiftedData = data - zeroAvg

        return shiftedData

    
    def _triggerSquareWave(self, data, cmdVoltage, calcResistance=True):
        try:
            shiftedData = self._shiftWaveToZero(data)
            mean = np.mean(shiftedData)
            lowAvg = np.mean(shiftedData[shiftedData < mean])
            highAvg = np.mean(shiftedData[shiftedData > mean])

            #split data into high and low wave
            triggerVal = np.mean(shiftedData)

            #is trigger value ever reached?
            if np.where(shiftedData < triggerVal)[0].size == 0 or np.where(shiftedData > triggerVal)[0].size == 0:
                if calcResistance:
                    return shiftedData, None
                else:
                    return shiftedData

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

            if calcResistance:
                #convert high and low averages to pA
                highAvgPA = highAvg * 2000 * 1e-12
                lowAvgPA = lowAvg * 2000 * 1e-12
                #calculate resistance
                resistance = cmdVoltage / (highAvgPA - lowAvgPA)

                return shiftedData, resistance
            else:
                return shiftedData
            
        except Exception as e:
            print(e)
            #we got an invalid square wave
            if calcResistance:
                return data, None
            else:
                return data
        
    
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