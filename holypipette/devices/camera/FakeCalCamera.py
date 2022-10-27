from tkinter import Frame
from .camera import *
import numpy as np
import cv2

class FakeCalCamera(Camera):
    def __init__(self, manipulator=None, image_z=0, paramecium=False):
        super(FakeCalCamera, self).__init__()
        self.width = 1024
        self.height = 1024
        self.exposure_time = 30
        self.manipulator = manipulator
        self.image_z = image_z
        self.scale_factor = .5  # micrometers in pixels

        #create checkerboard pattern on smaller image
        self.frame = np.zeros((self.width // 20, self.height // 20), dtype=np.uint8)#np.array(np.clip((np.random.randn(self.width * 2, self.height * 2)*0.5)*50 + 128, 0, 255), dtype=np.uint8)
        self.frame[1::2,::2] = 255
        self.frame[::2,1::2] = 255
        #scale up to larger img 
        self.frame = cv2.resize(self.frame, dsize=(self.width * 2, self.height * 2), interpolation=cv2.INTER_NEAREST)

        self.start_acquisition()

    def set_exposure(self, value):
        if 0 < value <= 200:
            self.exposure_time = value

    def get_exposure(self):
        return self.exposure_time

    def get_microscope_image(self, x, y, z):
        frame = np.roll(self.frame, int(y), axis=0)
        frame = np.roll(frame, int(x), axis=1)
        frame = frame[self.height//2:self.height//2+self.height,
                      self.width//2:self.width//2+self.width]
        return np.array(frame, copy=True)

    def raw_snap(self):
        '''
        Returns the current image.
        This is a blocking call (wait until next frame is available)
        '''
        # Use the part of the image under the microscope

        stage_x, stage_y, stage_z = self.manipulator.position_group([4, 5, 6])
        stage_z -= self.image_z
        stage_x *= -self.scale_factor
        stage_y *= -self.scale_factor
        stage_z *= -self.scale_factor
        frame = self.get_microscope_image(stage_x, stage_y, stage_z)

        #add noise
        exposure_factor = self.exposure_time/30.
        frame = frame + np.random.randn(self.height, self.width)*5

        #blur proportionally to how far stage_z is from 0 (being focused in the img plane)
        focusFactor = abs(stage_z) / 10
        if focusFactor == 0:
            focusFactor = 0.1
        frame = cv2.GaussianBlur(frame,(63,63), focusFactor)

        return np.array(np.clip(frame*exposure_factor, 0, 255),
                        dtype=np.uint8)
