'''
Camera for a PCO Panda Camera
'''
import numpy as np

from . import *
import warnings
import pco

try:
    import cv2
except ImportError:
    warnings.warn('OpenCV is not installed.')

__all__ = ['PcoCamera']


class PcoCamera(Camera):
    PCO_RECORDER_LATEST_IMAGE = 0xFFFFFFFF

    def __init__(self, width=500, height=500):
        super(PcoCamera, self).__init__()
        self.cam = pco.Camera()
        self.cam.record(number_of_images=10, mode='ring buffer') #use "ring buffer" mode for continuous streaming from camera
        self.cam.wait_for_first_image()
        self.width = width
        self.height = height
        self.lowerBound = 0
        self.upperBound = 2**16
        self.currExposure = 0
        self.lastFrameNum = 0
        self.start_acquisition()

    def set_exposure(self, value):
        self.cam.set_exposure_time(value / 1000)

    def get_exposure(self):
        '''return the exposure time of the camera in ms
        '''
        exposure = self.cam.get_exposure_time() #this is in seconds
        self.currExposure = exposure
        return exposure * 1000 #convert to ms

    def __del__(self):
        if hasattr(self, 'cam'):
            self.cam.close()

    def get_frame_rate(self):
        return 1 / self.cam.get_exposure_time() #TODO: this is ideal, can we get actual fps?

    def reset(self):
        self.cam.close()
        self.cam = pco.Camera()
        self.cam.record(number_of_images=10, mode='ring buffer')
        self.cam.wait_for_first_image()

    def normalize(self, img = None):
        if img is None:
            img, meta = self.cam.image()

        #is there a better way to do this?
        #maybe 2 stdevs instead?
        self.lowerBound = img.min()
        self.upperBound = img.max()


    def raw_snap(self):
        '''
        Returns the current image.
        This is a blocking call (wait until next frame is available)
        '''
        img, meta = self.cam.image(image_number=PcoCamera.PCO_RECORDER_LATEST_IMAGE)
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