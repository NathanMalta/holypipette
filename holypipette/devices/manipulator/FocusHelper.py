import time
import cv2
import numpy as np
from holypipette.devices.manipulator.microscope import Microscope
from holypipette.devices.camera import pcocamera
from threading import Thread

class FocusHelper():
    '''A helper class to aid with microscope focusing
    '''
    
    def __init__(self, microscope: Microscope, camera: pcocamera):
        self.microscope : Microscope = microscope
        self.camera : pcocamera = camera

    def autofocusContinuous(self, distance):
        '''
        '''

        #move the microscope a certain distance forward
        commandedPos = self.microscope.position() + distance
        self.microscope.absolute_move(commandedPos)

        #start recording focus values and positions
        focusThread = FocusUpdater(self.microscope, self.camera)
        focusThread.start()

        #wait for the microscope to reach the pos
        while abs(self.microscope.position() - commandedPos) > 0.1:
            print('waiting for pos...')
            time.sleep(0.01)

        #stop the focus recording thread
        focusThread.stop()

        #wait for focus thread to stop
        while not focusThread.didFinish:
            print('waiting for finish...')
            time.sleep(0.01)

        bestIndex = np.argmax(focusThread.posFocusList[:, 1])
        #print out results
        bestPos = focusThread.posFocusList[bestIndex][0]
        self.microscope.absolute_move(bestPos)
    
    def autofocusInDirection(self, timeout:int = 15, pastMaxDist:int = 120, movementTol:int = 0.5, movementStep:int = 10) -> tuple[int, list]:
        ''' Move the stage in small increments of movementStep (microns), recording focus score as the stage moves.
        Terminate once we move the stage pastMaxDist (microns) past it's highest value or after timeout (seconds).
        '''

        #initialize relevant vars
        self.microscope.relative_move(-1)
        highestScore = self._getFocusScore() #the highest focus score we've seen
        highestPos = self.microscope.position() #the micrscope pos of the highest focus score
        posAndFocus = []
        commandedPos = self.microscope.position() #position setpoint
        
        startTime = time.time()
        while (time.time() - startTime < timeout) and abs(highestPos - self.microscope.position()) < pastMaxDist:
            if abs(commandedPos - self.microscope.position()) < movementTol:
                #only send a new movement command if the last one was reached
                commandedPos = self.microscope.position() + movementStep
                self.microscope.absolute_move(commandedPos)

            currScore = self._getFocusScore()
            currPos = self.microscope.position()
            posAndFocus.append([currPos, currScore])
            if currScore > highestScore:
                highestScore = currScore
                highestPos = currPos
            
        #move to place with the highest score (most focused)
        self.microscope.absolute_move(highestPos)
        print(f"moving to z to: {highestPos}")
        
        #wait until we reach most focused point
        while abs(self.microscope.position() - highestPos) > movementTol and time.time() - startTime < timeout:
            time.sleep(0.01)
        
        return highestPos, posAndFocus

    def autofocus(self):
        '''Attempts to auto focus the micrscope image by moving in the z-axis
        '''
        self.autofocusContinuous(500)
        return

        startPos = self.microscope.position()
        refocusTol = 100 #microns, if final pos is within this tol, focus in the other direction as well.

        for i in [500, 5]:
            #first try focusing down (negative direction)
            highestPos, posAndFocus = self.autofocusInDirection(pastMaxDist=2*i, movementStep=-i, timeout=10)
            # for pt in posAndFocus:
                # print(f"{pt[0]}, {pt[1]}")

            if abs(highestPos - startPos) < refocusTol:
                #if the final pos is close to our starting pos, we need to refocus in the
                #other direction: try focusing up (positive direction)
                highestPos, posAndFocus = self.autofocusInDirection(pastMaxDist=1*i, movementStep=i, timeout=10)

        posAndFocus = np.array(posAndFocus)


class FocusUpdater(Thread):

    def __init__(self, microscope, camera):
       Thread.__init__(self)
       self.isRunning = True
       self.camera = camera
       self.microscope = microscope
       self.lastFrame = 0
       self.posFocusList = []
       self.didFinish = False

    def run(self):
        '''continuously read frames from camera, and record their focus and the microscope's z position.  Assumes constant velocity!
        '''
        startTime = time.time()
        startPos = self.microscope.position()

        while self.isRunning:
            while self.lastFrame == self.camera.getFrameNo():
                time.sleep(0.01) #wait for a new frame to be read from the camera
            self.lastFrame = self.camera.getFrameNo()

            #grab current position
            
            #get focus score from frame
            img = self.camera.get16BitImgRaw()
            score = self._getFocusScore(img)

            #append to list
            self.posFocusList.append([None, score, time.time()])
        
        stopTime = time.time()
        stopPos = self.microscope.position()
        
        #now interpolate with time to approximate frame positions
        vel = (stopPos - startPos) / (stopTime - startTime)

        for i, (_, _, frameTime) in enumerate(self.posFocusList):
            dt = frameTime - startTime
            self.posFocusList[i][0] = startPos + dt * vel
        
        self.posFocusList = np.array(self.posFocusList)
        self.didFinish = True #create a flag when we creating the arr


    def _getFocusScore(self, image) -> float:
        '''Get a score stating how focused a given image is
        Higher Score == more focused image
        '''

        focusSize = 1024
        x = image.shape[1]/2 - focusSize/2
        y = image.shape[0]/2 - focusSize/2
        crop_img = image[int(y):int(y+focusSize), int(x):int(x+focusSize)]

        xEdges = cv2.norm(cv2.Sobel(src=crop_img, ddepth=cv2.CV_32F, dx=1, dy=0, ksize=7))
        yEdges = cv2.norm(cv2.Sobel(src=crop_img, ddepth=cv2.CV_32F, dx=0, dy=1, ksize=7))
        score = xEdges ** 2 + yEdges ** 2
        # score = cv2.Laplacian(crop_img, cv2.CV_64F).var()
        
        return score


    def stop(self):
        self.isRunning = False
