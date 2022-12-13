import pco
import time

PCO_RECORDER_LATEST_IMAGE = 0xFFFFFFFF

cam = pco.Camera()
cam.record(number_of_images=10, mode='ring buffer') #use "ring buffer" mode for continuous streaming from camera
cam.wait_for_first_image()

frameno = cam.rec.get_status()['dwProcImgCount']
numFrames = 100

#wait for the frame to update
while frameno == cam.rec.get_status()['dwProcImgCount']:
    pass

#the frame has just arrived
lastTime = time.time()
frameno = cam.rec.get_status()['dwProcImgCount']

#wait for the next numFrames frames to arrive
timeList = []
lastFrame = frameno
while cam.rec.get_status()['dwProcImgCount'] < frameno + numFrames:
    if lastFrame != cam.rec.get_status()['dwProcImgCount']:
        lastFrame = cam.rec.get_status()['dwProcImgCount']
        timeList.append(time.time() - lastTime)
        lastTime = time.time()

print(f"Average time between frames: {sum(timeList) / len(timeList)}")
print(f"Average frame rate: {1 / (sum(timeList) / len(timeList))}")

#print out the time between frames
for timeVal in timeList:
    print(timeVal)

#close the camera
cam.close()



