import nidaqmx.system
import nidaqmx
from nidaqmx.types import CtrTime
import numpy as np
import matplotlib.pyplot as plt

ampRead = 'cDAQ1Mod2'
ampCmd = 'cDaq1Mod1'

#list all daq devices connected
system = nidaqmx.system.System.local()
devices = system.devices
print([d for d in devices])

ampReadDev = devices[ampRead]
ampCmdDev = devices[ampCmd]


def readAnalogInput(device, channel):
    with nidaqmx.Task() as task:
        task.ai_channels.add_ai_voltage_chan(f'{device}/{channel}')
        data = task.read(number_of_samples_per_channel=1000)
        return data

#continuously send square wave to channel 
def sendSquareWave(device, channel, freq, dutyCycle):
    task = nidaqmx.Task()
    task.ao_channels.add_ao_voltage_chan(f'{device}/{channel}')
    task.timing.cfg_samp_clk_timing(freq, sample_mode=nidaqmx.constants.AcquisitionType.CONTINUOUS)
    
    #create square wave numpy array
    data = np.zeros(freq)
    data[0:int(freq*dutyCycle)] = 5
    data[int(freq*dutyCycle):] = 0
    print(data)
    task.write(data)
    
    task.start()
    
    return task

#continuous read and plot with matplotlib
def readAnalogInputContinuous(device, channel):
    with nidaqmx.Task() as task:
        task.ai_channels.add_ai_voltage_chan(f'{device}/{channel}')
        task.timing.cfg_samp_clk_timing(10000, sample_mode=nidaqmx.constants.AcquisitionType.CONTINUOUS)
        # task.triggers.reference_trigger.cfg_anlg_edge_ref_trig('ao/StartTrigger', nidaqmx.constants.Edge.RISING, )
        # task.triggers.reference_trigger.cfg_anlg_edge_ref_trig("Dev1/ai0", pretrigger_samples = 10, trigger_slope=nidaqmx.constants.Slope.FALLING, trigger_level = 5)

        task.start()
        squareWave = sendSquareWave(ampCmd, 'ao1', 100, 0.5)
        while True:
            data = task.read(number_of_samples_per_channel=1000)
            plt.plot(data)
            #fix scale between -0.1 and 0.1
            plt.ylim(0, 10)
            plt.draw()
            #break on escape key
            if plt.waitforbuttonpress(0.001):
                break
            plt.pause(0.001)
            plt.clf()
        
        #stop task
        task.stop()
        squareWave.stop()
        squareWave.close()
        print('done')
    
if __name__ == '__main__':
    readAnalogInputContinuous(ampRead, 'ai0')
    