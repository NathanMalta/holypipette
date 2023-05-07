from holypipette.devices.manipulator import Manipulator, FakeManipulator
from .camera import Camera
import numpy as np
import cv2
from pathlib import Path
import time
import math

from PIL import Image, ImageDraw, ImageFilter, ImageEnhance

class FakeCalCamera(Camera):
    def __init__(self, stageManip=None, pipetteManip=None, image_z=0, targetFramerate=40, cellSorterManip=None):
        super(FakeCalCamera, self).__init__()
        self.width : int = 1024
        self.height : int = 1024
        self.exposure_time : int = 30
        self.stageManip : Manipulator = stageManip
        self.pipetteManip : Manipulator = pipetteManip
        self.image_z : float = image_z
        self.pixels_per_micron : float = 1  # pixels / micrometers
        self.frameno : int = 0
        self.pipette = FakePipette(self.pipetteManip, self.pixels_per_micron)
        self.targetFramerate = targetFramerate

        curFile = str(Path(__file__).parent.absolute())

        #setup frame image (numpy because of easy rolling)
        self.frame = cv2.imread(curFile + "/FakeMicroscopeImgs/background.png", cv2.IMREAD_GRAYSCALE)
        self.frame = cv2.resize(self.frame, dsize=(self.width * 2, self.height * 2), interpolation=cv2.INTER_NEAREST)

        #normalize image
        self.frame += np.min(self.frame)
        self.frame = self.frame / np.max(self.frame)

        #convert to 8 bit
        self.frame *= 255
        self.frame = self.frame.astype(np.uint8)

        self.last_img = None
        self.last_stage_pos = None

        #creating large noise arrays slows down fps, create 100 arrays at startup instead
        self.noiseArrs = []
        for _ in range(100):
            self.noiseArrs.append((np.random.random((self.width, self.height)) * 30).astype(np.uint16))

        #start image recording thread
        self.start_acquisition()

    def normalize(self):
        print('normalize not implemented for FakeCalCamera')

    def set_exposure(self, value):
        if 0 < value <= 200:
            self.exposure_time = value

    def get_exposure(self):
        return self.exposure_time

    def get_microscope_image(self, x, y):
        if self.last_img is None or self.last_stage_pos[0] != x or self.last_stage_pos[1] != y:
            #we need to recalculate what the stage sees
            frame = np.roll(self.frame, int(y), axis=0)
            frame = np.roll(frame, int(x), axis=1)
            frame = frame[self.height//2:self.height//2+self.height,
                        self.width//2:self.width//2+self.width]
            
            #update cached frame
            self.last_stage_pos = [x, y]
            self.last_img = frame
        else:
            frame = self.last_img
            
        self.frameno += 1
        return Image.fromarray(frame)

    def get_16bit_image(self):
        #Note: use float 32 rather than int16 for opencv sobel filter compatability (focus score)
        return (self.raw_snap().astype(np.float32) / 255) * 65535 

    def get_frame_no(self):
        return self.frameno

    def raw_snap(self):
        '''
        Returns the current image.
        This is a blocking call (wait until next frame is available)
        '''
        start = time.time()
        # Use the part of the image under the microscope
        stage_x, stage_y, stage_z = self.stageManip.position_group([1, 2, 3])

        startPos = [0, 0, 0]
        stage_x = stage_x - startPos[0]
        stage_y = stage_y - startPos[1]
        stage_z = stage_z - startPos[2]

        #get background at current stage position
        img_x = -stage_x * self.pixels_per_micron
        img_y = -stage_y * self.pixels_per_micron
        frame = self.get_microscope_image(img_x, img_y)

        #blur cover slip proportionally to how far stage_z is from 0 (being focused in the img plane)
        focusFactor = abs(stage_z - self.image_z) / 10
        if focusFactor == 0:
            focusFactor = 0.1

        frame = cv2.GaussianBlur(np.array(frame), (63,63), focusFactor)
        frame = Image.fromarray(frame)

        #add pipette to image
        frame = self.pipette.add_pipette_to_img(frame, [stage_x, stage_y, stage_z])

        #add noise, exposure
        exposure_factor = self.exposure_time/30.

        frame = frame + self.noiseArrs[self.frameno % len(self.noiseArrs)] #use pregenerated noise to increase fps
        frame[np.where(frame >= 255)] = 255
        frame[np.where(frame < 0)] = 0
        frame = frame.astype(np.uint8)

        dt = time.time() - start
        if dt < (1/self.targetFramerate):
            time.sleep((1/self.targetFramerate) - dt)
        
        return frame
    
class FakePipette():

    def __init__(self, manipulator:Manipulator, microscope_pixels_per_micron, stage_to_pipette=np.eye(4,4), pipetteAngle=np.pi/6):

        stage_to_pipette = np.eye(4,4)
        self.rot_mat  = np.eye(4,4)

        stage_to_pipette = np.matmul(np.linalg.inv(self.rot_mat), stage_to_pipette)

        
        self.manipulator = manipulator
        self.pixels_per_micron = microscope_pixels_per_micron
        self.stage_to_pipette = stage_to_pipette #homoegeneous transform matrix from stage to pipette
        self.pipette_to_stage = np.linalg.inv(self.stage_to_pipette)

        #setup pipette image (PIL b/c of easy pasting)
        curFile = str(Path(__file__).parent.absolute())
        self.pipetteImg = Image.open(curFile + "/FakeMicroscopeImgs/pipette.png").convert("L")
        self.pipetteImg = self.pipetteImg.resize((self.pipetteImg.size[0] * 4, self.pipetteImg.size[1] * 2), Image.Resampling.BILINEAR)
        filter = ImageEnhance.Brightness(self.pipetteImg)
        self.pipetteImg = filter.enhance(1.2)

        #create an alpha mask for the pipette (to make pipette see through)
        filter = ImageEnhance.Brightness(self.pipetteImg)
        self.alphaMask = filter.enhance(1.2)
        
    def add_pipette_to_img(self, frame:Image, stagePos:list):

        # print(self.manipulator.position(), self.manipulator.raw_position())
        #get stage micron coords
        stage_x, stage_y, stage_z = stagePos

        #get stage pixel coords
        stage_img_x = stage_x * self.pixels_per_micron
        stage_img_y = stage_y * self.pixels_per_micron

        #get pipette micron coords
        pipette_x, pipette_y, pipette_z = self.manipulator.position()
        pipette_pos_h = np.array([pipette_x, pipette_y, pipette_z, 1])

        #get pipette position in stage coordinates
        pipette_pos_stage_coords_h = np.matmul(self.pipette_to_stage, pipette_pos_h.T)
        pipette_pos_stage_coords = pipette_pos_stage_coords_h[0:3] / pipette_pos_stage_coords_h[3]

        #get pipette position in image coordinates
        pipette_pos_img_coords = pipette_pos_stage_coords * self.pixels_per_micron

        #get x,y - convert to int, make relative to frame
        pipette_img_x = int(pipette_pos_img_coords[0] - stage_img_x) - self.pipetteImg.size[0] #pipette_pos should correspond to tip of pipette (upper right) 
        pipette_img_y = int(pipette_pos_img_coords[1] - stage_img_y)

        #blur pipette proportionally to distance between stage_z and pipette_z
        focusFactor = abs(stage_z - pipette_pos_stage_coords[2]) / 10
        if focusFactor == 0:
            focusFactor = 0.1 #resolve divide by 0 error

        #blur img
        pipetteImg = cv2.GaussianBlur(np.array(self.pipetteImg), (63,63), focusFactor)
        pipetteImg = Image.fromarray(pipetteImg)

        #blur alpha channel
        alphaMask = cv2.GaussianBlur(np.array(self.alphaMask), (63,63), focusFactor / 2)
        alphaMask = alphaMask / 1.3
        alphaMask = Image.fromarray(alphaMask.astype(np.uint8))

        #add pipette to frame
        frame.paste(pipetteImg, (pipette_img_x, pipette_img_y), alphaMask)
        frame = np.array(frame) #convert back to numpy for opencv support
        return frame