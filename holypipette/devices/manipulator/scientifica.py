"""
Scientifica Stage control.
"""
import warnings
from .manipulator import Manipulator
import sys
import time
import numpy as np
import os

mm_dir = 'C:\\Program Files\\Micro-Manager-2.0gamma'
sys.path.append(mm_dir)

try:
    import pymmcore
except ImportError: # Micromanager not installed
    warnings.warn('Micromanager is not installed, cannot use the Scientifica class.')

__all__ = ['Scientifica']


class Scientifica(Manipulator):
    def __init__(self, name = 'COM6'):
        Manipulator.__init__(self)

        #setup python micromanager bindings
        mmc = pymmcore.CMMCore()
        mmc.setDeviceAdapterSearchPaths([mm_dir])
        mmc.loadSystemConfiguration(os.path.join(mm_dir, "scientifica-v001.cfg"))
        self.mmc = mmc

        print(f"SCIENTIFICA Z STAGE PROPERTIES: {mmc.getDevicePropertyNames('ZStage')}")
        
        self.setMaxAccel(100)
        self.setMaxSpeed(10000)

        self.port_name = name

    def set_max_speed(self, speed):
        '''Sets the max speed for the Scientifica Stage.  
           Note that even though the device is called 'XYStage', this appears to affect the Z axis as well.
           It seems like the range for this is around (1000, 100000)
        '''
        self.mmc.setProperty('XYStage', 'MaxSpeed', speed)

    def set_max_accel(self, accel):
        '''Sets the max acceleration for the Scientifica Stage.
           Note that even though the device is called 'XYStage', this appears to affect the Z axis as well
           It seems like the range for this is around (10, 10000)
        '''
        self.mmc.setProperty('XYStage', 'Acceleration', accel)

        
    def __del__(self):
        try:
            self.mmc.unloadDevice('XYStage')
            self.mmc.unloadDevice('ZStage')
            self.mmc.unloadDevice(self.port_name)
        except:
            pass #object deleted before mmc init

    def position(self, axis):
        if axis == 1:
            return self.mmc.getXPosition('XYStage')
        if axis == 2:
            return self.mmc.getYPosition('XYStage')
        if axis == 3:
            return self.mmc.getPosition('ZStage')

    def position_group(self, axes):
        axes4 = [0, 0, 0, 0]
        for i in range(len(axes)):
            axes4[i] = self.position(axis = axes[i])
        return np.array(axes4[:len(axes)])

    def absolute_move(self, x, axis):
        if axis == 1:
            self.mmc.setXYPosition('XYStage', x, self.mmc.getYPosition('XYStage'))
        if axis == 2:
            self.mmc.setXYPosition('XYStage', self.mmc.getXPosition('XYStage'), x)
        if axis == 3:
            print("setting z to ", x)
            while True:
                try:
                    self.mmc.setPosition('ZStage', x)
                    break
                except:
                    time.sleep(0.1)

    def absolute_move_group(self, x, axes):
        for i in range(len(axes)):
            self.absolute_move(x[i], axes[i])
            self.wait_until_still()

    def relative_move(self, x, axis):
        while True:
            try:
                if axis == 1:
                    self.mmc.setRelativeXYPosition('XYStage', x, 0)
                if axis == 2:
                    self.mmc.setRelativeXYPosition('XYStage', 0, x)
                if axis == 3:
                    self.mmc.setRelativePosition('ZStage', x)
                    break
            except:
                print(f"COULD NOT MOVE AXIS {axis} to {x}")
                # raise Exception(f"COULD NOT MOVE AXIS {axis} to {x}")
                time.sleep(0.1)
            

    def wait_until_still(self, axes = None, axis = None):
        self.mmc.waitForSystem()
        self.sleep(.7) # That's a very long time!

    def stop(self):
        self.mmc.stop()


if __name__ == '__main__':
    prior = Scientifica()
    print(prior.position(axis = 1))
    #prior.relative_move(50,axis =1)
    #prior.wait_until_still()
    print(prior.position(axis=1))