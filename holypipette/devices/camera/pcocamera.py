'''
Camera for a PCO Panda Camera
'''
import numpy as np
import time
import copy

from . import *
import warnings
import pco

try:
    import cv2
except ImportError:
    warnings.warn('OpenCV is not installed.')

__all__ = ['PcoCamera']


class PcoCamera(Camera):
    def __init__(self, width=500, height=500):
        super(PcoCamera, self).__init__()
        self.cam = pco.Camera()
        self.width = width
        self.height = height
        self.lowerBound = 0
        self.upperBound = 2**16
        self.start_acquisition()

    def set_exposure(self, value):
        self.cam.set_exposure_time(value)

    def get_exposure(self):
        return self.cam.get_exposure_time()

    def __del__(self):
        if hasattr(self, 'cam'):
            self.cam.close()

    def get_frame_rate(self):
        return 1 / self.cam.get_exposure_time() #TODO: this is ideal, can we get actual fps?

    def reset(self):
        self.cam.close()
        self.cam = pco.Camera()

    def normalize(self):
        self.cam.record()
        img, meta = self.cam.image()

        #is there a better way to do this?
        #maybe 2 stdevs instead?
        self.lowerBound = np.min(img)
        self.upperBound = np.max(img)

    def raw_snap(self):
        '''
        Returns the current image.
        This is a blocking call (wait until next frame is available)
        '''
        self.cam.record()
        img, meta = self.cam.image()
        img = img.astype(np.uint16)

        #apply upper / lower bounds (normalization)
        img = np.clip(img, self.lowerBound, self.upperBound, dtype=np.float64)
        span = self.upperBound - self.lowerBound

        if span == 0:
            span = 1 #prevent divide by 0 for blank images

        img = img - self.lowerBound
        img = img / span

        #convert to 8 bit color
        img = img * 255
        img = img.astype(np.uint8)

        #resize if needed
        if self.width != None and self.height != None:
            img = cv2.resize(img, (self.width, self.height), interpolation= cv2.INTER_LINEAR)

        return img
