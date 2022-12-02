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
    PCO_RECORDER_LATEST_IMAGE = 0xFFFFFFFF

    def __init__(self, width=512, height=512):
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
        self.lastFrame = None
        self.start_acquisition()
        self.p0 = None

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
            img = self.get16BitImg()

        #is there a better way to do this?
        #maybe 2 stdevs instead?
        self.lowerBound = img.min()
        self.upperBound = img.max()

    scores = [0] * 25
    def get16BitImg(self):
        if self.lastFrameNum == self.cam.rec.get_status()['dwProcImgCount'] and self.lastFrame is not None:
            return self.lastFrame
        else:
            self.lastFrameNum = self.cam.rec.get_status()['dwProcImgCount']
        
        try:
            img, meta = self.cam.image(image_number=PcoCamera.PCO_RECORDER_LATEST_IMAGE)
            self.lastFrame = img
        except:
            return self.lastFrame #there was an error grabbing the most recent frame

        # focusSize = 300
        # x = img.shape[1]/2 - focusSize/2
        # y = img.shape[0]/2 - focusSize/2
        # crop_img = img[int(y):int(y+focusSize), int(x):int(x+focusSize)]

        # score = cv2.Laplacian(crop_img, cv2.CV_64F).var()
        # self.scores.append(score)
        return img

    opticalPrev = None
    lastResetTime = 0
    def raw_snap(self):
        '''
        Returns the current image.
        This is a blocking call (wait until next frame is available)
        '''
        img = self.get16BitImg()
        #apply upper / lower bounds (normalization)
        # img = np.clip(img, self.lowerBound, self.upperBound, dtype=np.float64)
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

        # #optical flow
        # if self.opticalPrev is None:
        #     self.opticalPrev = img

        # # params for ShiTomasi corner detection
        # feature_params = dict( maxCorners = 100,
        #                     qualityLevel = 0.5,
        #                     minDistance = 7,
        #                     blockSize = 7 )
        # # Parameters for lucas kanade optical flow
        # lk_params = dict( winSize  = (30, 30),
        #                 maxLevel = 5,
        #                 criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))


        # if self.p0 is None or (time.time() - self.lastResetTime) > 10:
        #     self.p0 = cv2.goodFeaturesToTrack(img, mask = None, **feature_params).astype(np.float32)
        #     self.opticalPrev = None
        #     self.lastResetTime = time.time()
        
        # compositeImg = None
        # if self.opticalPrev is not None:
        #     p1, st, err = cv2.calcOpticalFlowPyrLK(self.opticalPrev, img, self.p0, None, **lk_params)

        #     # Select good points
        #     if p1 is not None:
        #         good_new = p1[st==1]
        #         good_old = self.p0[st==1]
        #     # draw the tracks
        #     if len(good_new) > 0:
        #         for i, (new, old) in enumerate(zip(good_new, good_old)):
        #             a, b = new.ravel()
        #             c, d = old.ravel()
        #             color = np.random.randint(0, 255, (100, 3))

        #             mask = np.zeros_like(img)
        #             # mask = cv2.line(mask, (int(a), int(b)), (int(c), int(d)), color[i].tolist(), 2)
        #             compositeImg = cv2.circle(img, (int(a), int(b)), 2, 255, -1)
        #         # compositeImg = cv2.add(img, frame)
            
        #         self.p0 = good_new.reshape(-1, 1, 2).astype(np.float32)
            
        #     print(self.p0[:, 0, 1])
            
            # self.opticalPrev = None
            # self.p0 = None
        
        # self.opticalPrev = img

        # if compositeImg is None:
        return img
        # else:
        #     return compositeImg