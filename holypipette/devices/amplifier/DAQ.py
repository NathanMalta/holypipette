from holypipette.devices.camera import WorldModel

import numpy as np
import math
import time

__all__ = ['FakeDAQ']

class FakeDAQ:
    def __init__(self, worldModel):
        self.worldModel: WorldModel = worldModel

    def getDataFromSquareWave(self, wave_freq, samplesPerSec, dutyCycle, amplitude, recordingTime):
        
        if self.worldModel.isBrokenIn() or self.worldModel.isSealed():
            #first order break-in response
                        #1st order RC response
            res_steady_state = self.worldModel.getResistance()
            res_peak = self.worldModel.getResistancePeak()
            tau = self.worldModel.getTau()

            I_peak = 0.01 / res_peak
            I_ss = 0.01 / res_steady_state

            data = np.zeros(int(samplesPerSec * recordingTime))
            #set first half to high
            data[1:int(len(data)/2)] = I_peak

            #add exponential decay
            time_per_i = 1 / samplesPerSec
            for i in range(2, len(data)//2):
                data[i] = I_peak * math.exp(-(i * time_per_i) / tau) + I_ss

            for i in range(len(data)//2, len(data)):
                dt = (i - len(data)//2) * time_per_i
                negative_peak = - I_peak
                data[i] = negative_peak * math.exp(-dt / tau)


            #add a bit of noise
            noise_level = 5 * 1e-12
            data += np.random.normal(0, noise_level, data.shape)

        else:
            #Ohmic, squarewave response
            resistance = self.worldModel.getResistance()
            amplitude = 0.01 / resistance

            data = np.zeros(int(samplesPerSec * recordingTime))
            #set first half to high
            data[0:int(len(data)/2)] = amplitude

            #add a bit of noise
            noise_level = 5 * 1e-12
            data += np.random.normal(0, noise_level, data.shape)

        xdata = np.linspace(0, recordingTime, len(data), dtype=float)

        data = np.array([xdata, data])

        return data

    def getResistance(self):
        return self.worldModel.getResistance()