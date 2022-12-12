import time
import cv2
import numpy as np
from holypipette.devices.manipulator.microscope import Microscope
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
        bestTime = focusThread.posFocusList[bestIndex][2]
        bestScore = focusThread.posFocusList[bestIndex][1]

        #linearly interpolate to find position
        
        #find closest time above and below best time
        dtArr = np.abs(posTimeArr[:, 0] - bestTime)

        #get closest point
        iclosest = dtArr.argmin()
        
        #handle edge case - closest point is at the edge of the array
        if iclosest == dtArr.shape[0] - 1 or iclosest == 0:
            return posTimeArr[iclosest][1], bestScore

        #get second closest point
        secondClosest = min([dtArr[iclosest - 1], dtArr[iclosest + 1]])
        isecondClosest = iclosest - 1 if dtArr[iclosest - 1] == secondClosest else iclosest + 1

        #determine which is above / below the known point
        iclosestAbove = iclosest if posTimeArr[iclosest][0] > posTimeArr[isecondClosest][0] else isecondClosest
        iclosestBelow = iclosest if posTimeArr[iclosest][0] <= posTimeArr[isecondClosest][0] else isecondClosest

        #interpolate based on the values with known positions closest
        dt = bestTime - posTimeArr[iclosestBelow][0]
        vel = (posTimeArr[iclosestAbove][1] - posTimeArr[iclosestBelow][1]) / (posTimeArr[iclosestAbove][0] - posTimeArr[iclosestBelow][0])
        bestPos = posTimeArr[iclosestBelow][1] + vel * dt
        
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
            self.posFocusList.append([None, score, frametime.timestamp()])
        
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
