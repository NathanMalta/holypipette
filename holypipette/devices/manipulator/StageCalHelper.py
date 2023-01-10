import time
import cv2
import numpy as np
from holypipette.devices.manipulator.microscope import Microscope
from holypipette.devices.manipulator import Manipulator
from holypipette.devices.camera import Camera
from threading import Thread

class FocusHelper():
    '''A helper class to aid with microscope focusing
    '''
    
    FOCUSING_MAX_SPEED = 1000
    NORMAL_MAX_SPEED = 10000

    def __init__(self, microscope: Microscope, camera: Camera):
        self.microscope : Microscope = microscope
        self.camera : Camera = camera

    def autofocusContinuous(self, distance):
        '''tell the stage to go a certain (larger) distance at a low max speed.
           Take a bunch of pictures and determine focus score.  Finally,
           move the stage to the position with the best focus score 
        '''

        #move the microscope a certain distance forward
        commandedPos = self.microscope.position() + distance
        self.microscope.absolute_move(commandedPos)

        #start recording focus values and positions
        focusThread = FocusUpdater(self.microscope, self.camera)
        focusThread.start()

        #wait for the microscope to reach the pos
        posTimeArr = []
        currPos = self.microscope.position()
        while abs(currPos - commandedPos) > 0.3:
            currPos = self.microscope.position()
            posTimeArr.append((time.time(), currPos))
            time.sleep(0.1)

        posTimeArr = np.array(posTimeArr)

        #stop the focus recording thread
        focusThread.stop()

        #wait for focus thread to stop
        while not focusThread.didFinish:
            # print('waiting for thread finish...')
            time.sleep(0.01)

        #find index with best score
        bestIndex = np.argmax(focusThread.posFocusList[:, 1])

        #find results for that index
        bestScore = focusThread.posFocusList[bestIndex][1]
        bestPos = focusThread.posFocusList[bestIndex][0]
        
        #return best score, position
        return bestPos, bestScore

    def autofocus(self):
        '''Attempts to auto focus the micrscope image by moving in the z-axis
        '''

        self.microscope.set_max_speed(self.FOCUSING_MAX_SPEED)
        initPos = self.microscope.position()
        bestForwardPos, bestForwardScore = self.autofocusContinuous(500)
        
        self.microscope.set_max_speed(self.NORMAL_MAX_SPEED)
        self.microscope.absolute_move(initPos)
        self.microscope.wait_until_still()
        self.microscope.set_max_speed(self.FOCUSING_MAX_SPEED)

        bestBackwardPos, bestBackwardScore = self.autofocusContinuous(-500)
        self.microscope.set_max_speed(self.NORMAL_MAX_SPEED)

        if bestBackwardScore < bestForwardScore:
            self.microscope.absolute_move(bestForwardPos)
        else:
            self.microscope.absolute_move(bestBackwardPos)

        self.microscope.wait_until_still()

class FocusUpdater(Thread):
    def __init__(self, microscope : Microscope, camera : Camera):
       Thread.__init__(self)

       self.isRunning : bool = True
       self.camera : Camera = camera
       self.microscope : Microscope = microscope
       self.lastFrame : int = 0
       self.posFocusList : list = []
       self.didFinish : bool = False

    def run(self):
        '''continuously read frames from camera, and record their focus and the microscope's z position.  Assumes constant velocity!
        '''
        while self.isRunning:
            while self.lastFrame == self.camera.get_frame_no():
                time.sleep(0.01) #wait for a new frame to be read from the camera
            self.lastFrame = self.camera.get_frame_no()
            
            #get focus score from frame
            _, frametime, _, img = self.camera._last_frame_queue[0]
            score = self._getFocusScore(img)

            #append to list
            self.posFocusList.append([self.microscope.position(), score])
        
        self.posFocusList = np.array(self.posFocusList)
        self.didFinish = True #create a flag when we creating the arr


    def _getFocusScore(self, image) -> float:
        '''Get a score stating how focused a given image is
        Higher Score == more focused image
        '''

        focusSize = 512
        x = image.shape[1]/2 - focusSize/2
        y = image.shape[0]/2 - focusSize/2
        crop_img = image[int(y):int(y+focusSize), int(x):int(x+focusSize)]

        xEdges = cv2.norm(cv2.Sobel(src=crop_img, ddepth=cv2.CV_32F, dx=1, dy=0, ksize=7))
        yEdges = cv2.norm(cv2.Sobel(src=crop_img, ddepth=cv2.CV_32F, dx=0, dy=1, ksize=7))
        score = xEdges ** 2 + yEdges ** 2
        # score = cv2.Laplacian(crop_img, cv2.CV_32F).var()
        
        return xEdges


    def stop(self):
        self.isRunning = False


class StageCalHelper():
    '''A helper class to aid with Stage Calibration
    '''
    
    CAL_MAX_SPEED = 1000
    NORMAL_MAX_SPEED = 10000

    def __init__(self, stage: Manipulator, camera: Camera):
        self.stage : Manipulator = stage
        self.camera : Camera = camera
        self.lastFrameNo : int = None

    def calibrateContinuous(self, distance):
        '''Tell the stage to go a certain distance at a low max speed.
           Take a bunch of pictures and run optical flow. Use optical flow information
           to create a linear transform from stage microns to image pixels. 
        '''

        #move the microscope a certain distance forward
        currPos = self.stage.position()
        commandedPos = np.array([currPos[0] + distance, currPos[1] - distance])
        axes = np.array([0, 1], dtype=int)
        self.stage.absolute_move_group(commandedPos, axes)

        #wait for the microscope to reach the pos, recording frames
        framesAndPoses = []
        currPos = self.stage.position()
        startPos = currPos
        start = time.time()
        _, _, _, firstFrame = self.camera._last_frame_queue[0]
        p0 = self.calcOpticalFlowP0(firstFrame)
        while abs(currPos[0] - commandedPos[0]) > 0.3 or abs(currPos[1] - commandedPos[1]) > 0.3:
            while self.lastFrameNo == self.camera.get_frame_no():
                time.sleep(0.05) #wait for a new frame to be read from the camera
            self.lastFrameNo = self.camera.get_frame_no()
            currPos = self.stage.position()

            #get latest img
            _, _, _, frame = self.camera._last_frame_queue[0]

            framesAndPoses.append([frame.copy(), currPos[0] - startPos[0], currPos[1] - startPos[1]])

        #run optical flow on the recorded frames
        print('running optical flow...')
        imgPosStagePosList = []
        x_pix_total = 0
        y_pix_total = 0
        for i in range(len(framesAndPoses) - 1):
            if i % 10 == 0:
                print(f"{i}/{len(framesAndPoses)}")
            lastFrame, _, _ = framesAndPoses[i]
            currFrame, x_microns, y_microns = framesAndPoses[i + 1]

            p0 = self.calcOpticalFlowP0(lastFrame)
            x_pix, y_pix = self.calcOpticalFlow(lastFrame, currFrame, p0)
            x_pix_total += x_pix
            y_pix_total += y_pix

            print(x_pix, y_pix, x_microns, y_microns)
            imgPosStagePosList.append([x_pix_total, y_pix_total, x_microns, y_microns])
        imgPosStagePosList = np.array(imgPosStagePosList)
        
        #for some reason, estimateAffinePartial2D only works with int64
        #we can multiply by 100, to preserve 2 decimal places without affecting rotation / scaling portion of affline transform
        imgPosStagePosList = (imgPosStagePosList * 100).astype(np.int64) 
        #compute affine transformation matrix
        print(imgPosStagePosList)
        mat, inVsOut = cv2.estimateAffinePartial2D(imgPosStagePosList[:,2:4], imgPosStagePosList[:,0:2])

        #fix intercept - set image center --> stage center
        mat[0,2] = 0
        mat[1,2] = 0

        print('completed optical flow. matrix:')
        print(mat)

        #return transformation matrix
        return mat

    def calcOpticalFlowP0(self, firstFrame):
        #params for corner detector
        feature_params = dict(maxCorners = 100,
                                qualityLevel = 0.5,
                                minDistance = 7,
                                blockSize = 7)
        p0 = cv2.goodFeaturesToTrack(firstFrame, mask = None, **feature_params)
        return p0


    def calcOpticalFlow(self, lastFrame, currFrame, p0):
        #params for optical flow
        lk_params = dict(winSize  = (15, 15),
                    maxLevel = 5,
                    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

        # calculate optical flow from first frame
        p1, st, err = cv2.calcOpticalFlowPyrLK(lastFrame, currFrame, p0, None, **lk_params)

        # Select good points
        if p1 is not None:
            good_new = p1[st==1]
            good_old = p0[st==1]
        
            # draw the tracks
            for i, (new, old) in enumerate(zip(good_new, good_old)):
                a, b = new.ravel()
                c, d = old.ravel()
                currFrame = cv2.line(currFrame, (int(a), int(b)), (int(c), int(d)), 0, 2)
                currFrame = cv2.circle(currFrame, (int(a), int(b)), 5, 0, -1)


         #find median movement vector
        dMovement = good_new - good_old
        medianVect = np.median(dMovement, axis=0)
        
        return medianVect[0], medianVect[1]


    def calibrate(self):
        '''Calibrates the microscope stage using optical flow and stage encoders to create a um -> pixels transformation matrix
        '''

        self.stage.set_max_speed(self.CAL_MAX_SPEED)
        # self.stage.set_max_accel(10)

        initPos = self.stage.position()
        mat = self.calibrateContinuous(500)
        # commandedPos = np.array([initPos[0] + 200, initPos[1] - 200])
        # axes = np.array([0, 1], dtype=int)
        # self.stage.absolute_move_group(commandedPos, axes)
        self.stage.wait_until_still()
        currPos = self.stage.position()
        
        self.stage.set_max_speed(self.NORMAL_MAX_SPEED)
        self.stage.absolute_move(initPos)
        self.stage.wait_until_still()

        return mat
