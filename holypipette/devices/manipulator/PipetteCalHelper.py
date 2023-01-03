import time
import cv2
import numpy as np
from holypipette.devices.manipulator.microscope import Microscope
from holypipette.devices.manipulator import Manipulator
from holypipette.devices.camera import Camera
from holypipette.deepLearning.pipetteFinder import PipetteFinder
from threading import Thread

class PipetteCalHelper():
    '''A helper class to aid with Pipette Calibration
    '''
    
    CAL_MAX_SPEED = 3000
    NORMAL_MAX_SPEED = 10000

    def __init__(self, pipette: Manipulator, camera: Camera):
        self.pipette : Manipulator = pipette
        self.camera : Camera = camera
        self.pipetteFinder : PipetteFinder = PipetteFinder()
        self.lastFrameNo = 0 #the last frame we've recorded when moving the pipette

    def calibrateContinuous(self, distance):
        '''Tell the stage to go a certain distance at a low max speed.
           Take a bunch of pictures and use optical flow. Use optical flow information
           to create a linear transform from stage microns to image pixels. 
        '''

        #first, make sure the pipette is in frame
        _, _, _, frame = self.camera._last_frame_queue[0]
        pos = self.pipetteFinder.find_pipette(frame)
        if pos is None:
            raise RuntimeError("pipette must be in frame for pipette calibration!")

        #move the pipette a certain distance forward and up
        currPos = self.pipette.position()
        commandedPos = np.array([currPos[0] + distance, currPos[1] + distance])
        axes = np.array([0, 1], dtype=int)
        self.pipette.absolute_move_group(commandedPos, axes)

        framesAndPoses = []

        #wait for the pipette to reach the pos. Record frames and pos as the pipette moves
        currPos = self.pipette.position()
        while abs(currPos[0] - commandedPos[0]) > 0.3 or abs(currPos[1] - commandedPos[1]) > 0.3:
            while self.lastFrameNo == self.camera.get_frame_no():
                time.sleep(0.01) #wait for a new frame to be read from the camera
            self.lastFrameNo = self.camera.get_frame_no()
            currPos = self.pipette.position()
            
            #get latest img and pipette pos, add to arr
            _, _, _, frame = self.camera._last_frame_queue[0]
            framesAndPoses.append((frame, currPos))

        #find the pipette in all the frames
        pixelsAndPoses = []
        for i, (frame, pipettePos) in enumerate(framesAndPoses):
            if i % 10 == 0:
                print(f"determining pipette path... {i/len(framesAndPoses)}%")
            x, y = self.pipetteFinder.find_pipette(frame)
            pixelsAndPoses.append([x,y, pipettePos[0], pipettePos[1]])
        
        pixelsAndPoses = np.array(pixelsAndPoses)

        #for some reason, estimateAffinePartial2D only works with int64
        #we can multiply by 100, to preserve 2 decimal places without affecting rotation / scaling portion of affline transform
        pixelsAndPoses = (pixelsAndPoses.copy()).astype(np.int64) 
        #compute affine transformation matrix
        mat, _ = cv2.estimateAffinePartial2D(pixelsAndPoses[:,2:4], pixelsAndPoses[:,0:2])

        # #fix intercept - set image center --> stage center
        # mat[0,2] = 0
        # mat[1,2] = 0

        print('completed object detection. matrix:')
        print(mat)

        #return transformation matrix
        return mat

    def calibrate(self):
        '''Calibrates the pipette using YOLO object detection and pipette encoders to create a um -> pixels transformation matrix
        '''

        self.pipette.set_max_speed(self.CAL_MAX_SPEED)
        initPos = self.pipette.position()
        mat = self.calibrateContinuous(2500)
        
        self.pipette.set_max_speed(self.NORMAL_MAX_SPEED)
        self.pipette.absolute_move_group(initPos, [0,1,2])
        self.pipette.wait_until_still()

        return mat