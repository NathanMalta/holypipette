import sys
import cv2
import time

from setup_IBB_rig import stage, microscope, camera
# from setup_fake_rig import *

move_steps = [20, 35, 50, 100, 200, 500]
total_step = 0
for s in move_steps:
    total_step += s
pic_num = 0
collecting_no_pipette_data = True

while True:
    #grab frame from the camera
    img = camera.snap()

    #display using opencv
    cv2.imshow('img', img)

    #break if escape key is pressedx
    key = cv2.waitKey(1)
    if key == 27:
        break

    #take pictures up if spacebar is pressed
    if key == 32:
        print('taking picture!')
        time.sleep(0.5)
        img = camera.get_16bit_image()
        cv2.imwrite(f'outputs/pipetteFocusData/{pic_num}.png', img)
        with open(f'outputs/pipetteFocusData/{pic_num}.txt', 'w') as f:
            if not collecting_no_pipette_data:
                f.write(f'{0}')
            else:
                f.write(f'{13}')
        pic_num += 1

        #move stage up
        for i in range(len(move_steps)):
            microscope.relative_move(move_steps[i])
            time.sleep(0.5)
            #grab frame from the camera
            img = camera.get_16bit_image()
            #display using opencv
            cv2.imshow('img', img)
            cv2.waitKey(1)
            cv2.imwrite(f'outputs/pipetteFocusData/{pic_num}.png', img)

            #create annotation txt file
            with open(f'outputs/pipetteFocusData/{pic_num}.txt', 'w') as f:
                if not collecting_no_pipette_data:
                    f.write(f'{i+1}')
                else:
                    f.write(f'{13}')
            pic_num += 1
        
        #move to initial position
        microscope.relative_move(-total_step)
    
    #take pictures down if 'd' is pressed
    if key == 100:
        #move stage down
        #move stage up
        for i in range(len(move_steps)):
            microscope.relative_move(-move_steps[i])
            time.sleep(0.5)
            #grab frame from the camera
            img = camera.get_16bit_image()

            #display using opencv
            cv2.imshow('img', img)
            cv2.waitKey(1)
            cv2.imwrite(f'outputs/pipetteFocusData/{pic_num}.png', img)

            #create annotation txt file
            with open(f'outputs/pipetteFocusData/{pic_num}.txt', 'w') as f:
                if not collecting_no_pipette_data:
                    f.write(f'{i+len(move_steps)+1}')
                else:
                    f.write(f'{13}')
            pic_num += 1

        #move to initial position
        microscope.relative_move(total_step)


print('closing everything')
cv2.destroyAllWindows()
del camera
del microscope
del stage

sys.exit()
