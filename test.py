import cv2
import numpy as np

import cv2
import numpy as np

def build_panorama(images):
    # Initiate the first image as reference
    reference_image = images[0]
    orb = cv2.ORB_create()
    reference_keypoints, reference_descriptors = orb.detectAndCompute(reference_image, None)

    # Initiate Homography Matrix
    homography_matrix = np.eye(3, 3, dtype=np.float32)

    # Loop through the images to find Homography Matrix
    for i in range(1, len(images)):
        cv2.imshow('ref', reference_image)
        cv2.waitKey(0)

        current_image = images[i]
        current_keypoints, current_descriptors = orb.detectAndCompute(current_image, None)
        matches = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True).match(reference_descriptors, current_descriptors)

        # Filter out weak matches
        good_matches = [match for match in matches if match.distance < 30]
        if len(good_matches) < 4:
            continue

        # Find Homography Matrix
        source_points = np.float32([reference_keypoints[match.queryIdx].pt for match in good_matches]).reshape(-1, 1, 2)
        target_points = np.float32([current_keypoints[match.trainIdx].pt for match in good_matches]).reshape(-1, 1, 2)
        transformation_matrix, _ = cv2.findHomography(source_points, target_points, cv2.RANSAC, 5.0)
        homography_matrix = transformation_matrix

        # Warp the current image to align with the reference image
        ref_height, ref_width = reference_image.shape[:2]
        cur_height, cur_width = current_image.shape[:2]
        current_image = cv2.warpPerspective(current_image, homography_matrix, (ref_width + cur_width, ref_height + cur_height), flags=cv2.INTER_LINEAR + cv2.WARP_INVERSE_MAP)

        # Update reference image
        current_image[0:reference_image.shape[0], 0:reference_image.shape[1]] = reference_image

        #crop off black edges
        non_black_cols = np.where(np.sum(current_image, axis=0) != 0)[0]
        non_black_rows = np.where(np.sum(current_image, axis=1) != 0)[0]
        crop_box = (min(non_black_rows), max(non_black_rows), min(non_black_cols), max(non_black_cols))
        current_image = current_image[crop_box[0]:crop_box[1]+1, crop_box[2]:crop_box[3]+1]

        reference_image = current_image

        # Update reference image

        #find where reference image is black
        black = np.where(current_image == 0)
        # current_image[black] = reference_image[black]

        reference_keypoints, reference_descriptors = orb.detectAndCompute(reference_image, None)

    return reference_image


# Load the two images
img1 = cv2.imread("/home/nathan/Desktop/001.jpg", cv2.IMREAD_GRAYSCALE)
img2 = cv2.imread("/home/nathan/Desktop/002.jpg", cv2.IMREAD_GRAYSCALE)
img3 = cv2.imread("/home/nathan/Desktop/003.jpg", cv2.IMREAD_GRAYSCALE)
img4 = cv2.imread("/home/nathan/Desktop/004.jpg", cv2.IMREAD_GRAYSCALE)
img5 = cv2.imread("/home/nathan/Desktop/005.jpg", cv2.IMREAD_GRAYSCALE)
img6 = cv2.imread("/home/nathan/Desktop/006.jpg", cv2.IMREAD_GRAYSCALE)
img7 = cv2.imread("/home/nathan/Desktop/007.jpg", cv2.IMREAD_GRAYSCALE)

# Set the offset values
x_offset = 100
y_offset = 100

# Combine the two images
result = build_panorama([img1, img2, img3, img4, img5, img6, img7])

# Show the result
cv2.imshow("Result", result)
cv2.waitKey(0)
cv2.destroyAllWindows()
