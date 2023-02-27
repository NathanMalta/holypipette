import pco
import time
import threading
import cv2

from clampy import MultiClampChannel

def testPCO():
    '''A basic test script to verify pco camera functionality in ring buffer mode
    '''
    cam = pco.Camera()
    cam.record(number_of_images=10, mode='ring buffer')
    cam.wait_for_first_image()
    start_time = time.time()

    #record for 10 seconds, display images
    while time.time() - start_time < 10:
        preframe = time.time()
        frame, meta = cam.image() #Try 0xFFFFFFFF aka PCO_RECORDER_LATEST_IMAGE aka -1 ?
        postFrame = time.time()
        print(f"acquiring at {1/(postFrame - preframe)} fps")
        print(f"cam status: {cam.rec.get_status()}") #look into dwProcImgCount and fifo
        cv2.imshow("test", frame)
        cv2.waitKey(1)

    #stop the camera
    cam.close()

def testMultiClamp():
    amplifier = MultiClampChannel()
    amplifier.set_bridge_balance(True)
    start = time.time()
    while time.time() - start < 10:
        print(amplifier.get_bridge_resistance())
        print(amplifier.auto_bridge_balance())
        time.sleep(0.01)

testMultiClamp()
