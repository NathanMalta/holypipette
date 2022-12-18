import pymmcore
import time
import os
import pco
import cv2
import numpy as np

#start camera recording
PCO_RECORDER_LATEST_IMAGE = 0xFFFFFFFF

cam = pco.Camera()
config = {'exposure time': 10e-3,
            'roi': (1, 1, 1024, 1024),
            'timestamp': 'off',
            'trigger': 'auto sequence',
            'acquire': 'auto',
            'metadata': 'on',
            'binning': (2, 2)}
cam.configuration = config
cam.record(number_of_images=10, mode='ring buffer') #use "ring buffer" mode for continuous streaming from camera
cam.wait_for_first_image()

#setup python micromanager bindings
mm_dir = 'C:\\Program Files\\Micro-Manager-2.0gamma'
mmc = pymmcore.CMMCore()
mmc.setDeviceAdapterSearchPaths([mm_dir])
mmc.loadSystemConfiguration(os.path.join(mm_dir, "scientifica-v001.cfg"))

# Get the current stage position
stage_z = mmc.getPosition("ZStage")

#take initial image
img, meta = cam.image(image_number=PCO_RECORDER_LATEST_IMAGE)

#normalize image
lowerBound = img.min()
upperBound = img.max()
span = upperBound - lowerBound
if span == 0:
    span = 1 #prevent divide by 0 for blank images

img = img - lowerBound
img = img / span
img = img * 255
img = img.astype(np.uint8)

img = cv2.putText(img, f'setpoint: {stage_z:.2f}  encoders: {mmc.getPosition("ZStage"):.2f}', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 
                1, 0, 2, cv2.LINE_AA)
cv2.imwrite(f"stage0.png", img)

img_init = img
sub_init = None

# Set the stage position to 500 microns above the current position
mmc.setProperty('XYStage', 'MaxSpeed', 10000)

for i in range(1, 5):
    mmc.setPosition("ZStage", stage_z + 1000)
    time.sleep(5)
    mmc.setPosition("ZStage", stage_z)
    time.sleep(5)

    #take image at new location
    img, meta = cam.image(image_number=PCO_RECORDER_LATEST_IMAGE)
    img = img - lowerBound
    img = img / span
    img = img * 255
    img = img.astype(np.uint8)
    
    img = cv2.putText(img, f'setpoint: {stage_z:.2f}  encoders: {mmc.getPosition("ZStage"):.2f}', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 
                    1, 0, 2, cv2.LINE_AA)

    # if sub_init is None:
    #     sub_init = cv2.subtract(img_init, img)
    # else:
    #     subsub = cv2.subtract(sub_init, cv2.subtract(img_init, img))
    #     cv2.imshow(f'subsub{i}', subsub)
    #     cv2.imwrite(f"subsub{i}.png", img)



    cv2.imshow(f'sub{i}', cv2.subtract(img_init, img))
    print(np.sum(cv2.subtract(img_init, img)))
    cv2.imwrite(f"sub{i}.png", img)

    cv2.imshow('orig', img_init)
    cv2.imshow('second', img)

    cv2.waitKey(0)


    # cv2.imshow(f"stage {i}", img)
    cv2.imwrite(f"stage{i}.png", img)
    # cv2.waitKey(0)
    time.sleep(1)
