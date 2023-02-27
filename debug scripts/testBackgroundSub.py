from __future__ import print_function
import argparse
import cv2

parser = argparse.ArgumentParser(description='This program shows how to use background subtraction methods provided by \
                                              OpenCV. You can process both videos and images.')
parser.add_argument('--input', type=str, help='Path to a video or a sequence of image.', default='vtest.avi')
parser.add_argument('--algo', type=str, help='Background subtraction method (KNN, MOG2).', default='MOG2')
args = parser.parse_args()
if args.algo == 'MOG2':
    #create gsoc background subtractor
    backSub = cv2.bgsegm.createBackgroundSubtractorLSBP()

else:
    backSub = cv2.createBackgroundSubtractorKNN()
capture = cv2.VideoCapture(cv2.samples.findFileOrKeep(args.input))
if not capture.isOpened():
    print('Unable to open: ' + args.input)
    exit(0)

i = 0
while True:
    i += 1
    if i > 200:
        break
    if i % 100 == 0:
        print(i)
    ret, frame = capture.read()
    if frame is None:
        break
    
    fgMask = backSub.apply(frame, learningRate=0.1)
    
    
    cv2.rectangle(frame, (10, 2), (100,20), (255,255,255), -1)
    cv2.putText(frame, str(capture.get(cv2.CAP_PROP_POS_FRAMES)), (15, 15),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5 , (0,0,0))

    
    
    
    cv2.imshow('Frame', frame)
    cv2.imshow('FG Mask', fgMask)
    
    cv2.waitKey(30)

#show the background image
cv2.imshow('Background', backSub.getBackgroundImage())
cv2.waitKey(0)