'''
Camera for a PCO Panda Camera
'''
import numpy as np
import time

from . import *
import warnings
import pco

try:
    import cv2
except ImportError:
    warnings.warn('OpenCV is not installed.')

__all__ = ['PcoCamera']


class PcoCamera(Camera):
    '''A camera class for the PCO Panda microscope camera.
       more info on the camera can be found here: https://www.pco.de/fileadmin/fileadmin/user_upload/pco-manuals/pco.panda_manual.pdf
    '''

    PCO_RECORDER_LATEST_IMAGE = 0xFFFFFFFF

    def __init__(self, width : int = 1024, height : int = 1024):
        super(PcoCamera, self).__init__()

        self.width = width #update superclass img width / height vars
        self.height = height

        #setup the pco camera for continuous streaming
        self.cam = pco.Camera()
        self.cam.record(number_of_images=10, mode='ring buffer') #use "ring buffer" mode for continuous streaming from camera
        self.cam.wait_for_first_image()

        self.lastFrame = None

        self.currExposure = 0

        self.start_acquisition() #start thread that updates camera gui

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
        self.cam.sdk.set_binning(2,2)
        self.cam.record(number_of_images=10, mode='ring buffer')
        self.cam.wait_for_first_image()

    def normalize(self, img = None):
        if img is None:
            img = self.get_16bit_image()

        #is there a better way to do this?
        #maybe 2 stdevs instead?
        self.lowerBound = img.min()
        self.upperBound = img.max()

    def get_frame_no(self):
        return self.cam.rec.get_status()['dwProcImgCount']
        
    def get_16bit_image(self):
        '''get a 16 bit color image from the camera (no normalization)
           this compares to raw_snap which returns a 8 bit image with normalization
        '''
        if self.frameno == self.cam.rec.get_status()['dwProcImgCount'] and self.lastFrame is not None:
            return self.lastFrame
        else:
            self.frameno = self.cam.rec.get_status()['dwProcImgCount']
        
        try:
            img, meta = self.cam.image(image_number=PcoCamera.PCO_RECORDER_LATEST_IMAGE)
            self.lastFrame = img
        except:
            return self.lastFrame #there was an error grabbing the most recent frame

        return img

    def raw_snap(self):
        '''
        Returns the current image (8 bit color, with normalization).
        This is a blocking call (wait until next frame is available)
        '''
        img = self.get_16bit_image()

        #apply upper / lower bounds (normalization)
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