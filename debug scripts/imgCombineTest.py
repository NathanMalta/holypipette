import cv2
import numpy as np

import cv2
import numpy as np

def _paste_image(img1, img2, x_off, y_off):
    h1, w1 = img1.shape
    h2, w2 = img2.shape
    # print("h1: {}, w1: {}, h2: {}, w2: {} x_off: {} y_off: {}".format(h1, w1, h2, w2, x_off, y_off))
    new_img = np.zeros((h1 + h2, w1 + w2), dtype=np.uint8)
    if x_off >= 0 and y_off >= 0:
        real_x_off = x_off + w1
        real_y_off = y_off + h1
        new_img[real_y_off-h2:real_y_off, real_x_off-w2:real_x_off] = img2
        new_img[0:h1, 0:w1] = img1
    elif x_off >= 0 and y_off < 0:
        y_off_abs = abs(y_off)
        real_x_off = x_off + w1
        new_img[0:h2, real_x_off-w2:real_x_off] = img2
        new_img[y_off_abs:h1+y_off_abs, 0:w1] = img1
    elif x_off < 0 and y_off >= 0:
        real_y_off = y_off + h1
        x_off_abs = abs(x_off)
        new_img[real_y_off - h2:real_y_off, 0:w2] = img2
        new_img[0:h1, x_off_abs:w1+x_off_abs] = img1
    else:
        y_off_abs = abs(y_off)
        x_off_abs = abs(x_off)
        new_img[0:h2, 0:w2] = img2
        new_img[y_off_abs:h1+y_off_abs, x_off_abs:w1+x_off_abs] = img1

    #crop out extra row, cols from img
    non_black_cols = np.where(np.sum(new_img, axis=0) != 0)[0]
    non_black_rows = np.where(np.sum(new_img, axis=1) != 0)[0]
    crop_box = (min(non_black_rows), max(non_black_rows), min(non_black_cols), max(non_black_cols))
    new_img = new_img[crop_box[0]:crop_box[1]+1, crop_box[2]:crop_box[3]+1]

    return new_img

def calcORB_2(images):
    # Initiate the first image as reference
    reference_image = images[0]
    prev_image = images[0]
    orb = cv2.ORB_create(nfeatures=1000, scaleFactor=2, nlevels=2, edgeThreshold=30, firstLevel=0, WTA_K=3, scoreType=cv2.ORB_HARRIS_SCORE, patchSize=30, fastThreshold=20)
    prev_keypoints, prev_descriptors = orb.detectAndCompute(reference_image, None)

    translations = []
    total_x_off = 0
    total_y_off = 0

    # Loop through the images to find Homography Matrix
    for i in range(1, len(images)):
        current_image = images[i]
        current_keypoints, current_descriptors = orb.detectAndCompute(current_image, None)
        prev_keypoints, prev_descriptors = orb.detectAndCompute(reference_image, None)
        matches = cv2.BFMatcher(cv2.NORM_HAMMING2, crossCheck=True).match(prev_descriptors, current_descriptors)

        # Find Homography Matrix
        source_points = np.float32([prev_keypoints[match.queryIdx].pt for match in matches]).reshape(-1, 2)
        target_points = np.float32([current_keypoints[match.trainIdx].pt for match in matches]).reshape(-1, 2)
        # homography_matrix, _ = cv2.findHomography(source_points, target_points, cv2.RANSAC, 5.0, None, 1000, 0.999)

        source_to_target = target_points - source_points
        #find average movement
        avg_movement = np.median(source_to_target, axis=0)
        homography_matrix = np.zeros((3, 3))

        #calculate translation from homography matrix (translation from just this step)
        x_off = int(avg_movement[1]) - total_x_off
        y_off = int(avg_movement[0]) - total_y_off

        if x_off <= 0:
            print(f'blocking: {x_off} {y_off}')
            continue
        if y_off >= 0:
            print(f'blocking: {x_off} {y_off}')
            continue

        #calc cumulative translation (translation from all steps)
        total_x_off += x_off
        total_y_off += y_off

        translations.append([total_x_off, total_y_off])

        #combine new image with reference image
        # print("x_off: {}, y_off: {}".format(total_x_off, total_y_off))
        # Sort them in the order of their distance.
        matches = sorted(matches, key = lambda x:x.distance)
        # Draw first 10 matches.
        img3 = cv2.drawMatches(reference_image,prev_keypoints,current_image,current_keypoints,matches[:100],None,flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

        # print(x_off - reference_image.shape[1] + current_image.shape[1])
        print("x_off: {}, y_ofasdf: {}".format(x_off, y_off))
        cv2.imshow("Current Image", img3)
        cv2.waitKey(10)
        reference_image = _paste_image(reference_image, current_image, x_off, y_off)
        prev_image = current_image

    translations = np.array(translations)

    return reference_image, translations




# Load the two images
base_path = 'outputs/'
imgs = [cv2.imread(f"{base_path}{i}.png", cv2.IMREAD_GRAYSCALE) for i in range(1, 100)]
# imgs = [i for i in reversed(imgs)]
# Set the offset values
x_offset = 100
y_offset = 100

# Combine the two images
result, translations = calcORB_2(imgs)

# print(translations)

# Show the result
cv2.imshow("Result", result)
cv2.waitKey(0)
cv2.destroyAllWindows()
