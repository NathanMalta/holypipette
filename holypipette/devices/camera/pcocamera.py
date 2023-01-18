'''
Camera for a PCO Panda Camera
'''
import numpy as np
import time

from . import *
import warnings
import pco
from holypipette.deepLearning.pipetteFinder import PipetteFinder

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
        # self.cam.sdk.set_timestamp_mode('binary & ascii')
        config = {'exposure time': 10e-3,
                    'roi': (1, 1, 1024, 1024),
                    'timestamp': 'off',
                    'trigger': 'auto sequence',
                    'acquire': 'auto',
                    'metadata': 'on',
                    'binning': (2, 2)}
        self.cam.configuration = config

        self.cam.record(number_of_images=10, mode='ring buffer') #use "ring buffer" mode for continuous streaming from camera
        self.cam.wait_for_first_image()

        self.frameno = None

        self.currExposure = 0

        self.upperBound = 255
        self.lowerBound = 0
        self.pipetteFinder = PipetteFinder()

        self.normalize() #normalize image on startup

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
        
        config = {'exposure time': 10e-3,
                    'roi': (0, 0, 1024, 1024),
                    'timestamp': 'off',
                    'pixel rate': 500_000_000,
                    'trigger': 'auto sequence',
                    'acquire': 'auto',
                    'metadata': 'on',
                    'binning': (2, 2)}
        self.cam.configuration = config

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
        return self.frameno
        
    def get_16bit_image(self):
        '''get a 16 bit color image from the camera (no normalization)
           this compares to raw_snap which returns a 8 bit image with normalization
        '''
        # if self.frameno == self.cam.rec.get_status()['dwProcImgCount'] and self.lastFrame is not None:
        #     return self.lastFrame
        # else:
        self.frameno = self.cam.rec.get_status()['dwProcImgCount']
        
        try:
            img, meta = self.cam.image(image_number=PcoCamera.PCO_RECORDER_LATEST_IMAGE)
            self.lastFrame = img
            # print(meta)
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

        img = img.astype(np.float32)
        img = img - self.lowerBound
        img = img / span

        #convert to 8 bit color
        img = img * 255
        img[np.where(img < 0)] = 0
        img[np.where(img > 255)] = 255
        img = img.astype(np.uint8)

        #resize if needed
        if self.width != None and self.height != None:
            img = cv2.resize(img, (self.width, self.height), interpolation= cv2.INTER_LINEAR)

        if img is not None:
            out = self.pipetteFinder.find_pipette(img)
            if out is not None:
                img = cv2.circle(img, out, 2, 0, 2)

        return img