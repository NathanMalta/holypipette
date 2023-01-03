import cv2
import numpy as np
import time
from pathlib import Path

class PipetteFinder():

	def __init__(self):
		curFile = str(Path(__file__).parent.absolute())
		self.yoloNet = cv2.dnn.readNetFromDarknet(curFile + '/pipetteModel/pipetteFinder.cfg', curFile + '/pipetteModel/pipetteFinder.weights')

		layer_names = self.yoloNet.getLayerNames()
		self.output_layers = [layer_names[i-1] for i in self.yoloNet.getUnconnectedOutLayers()]

		self.pipette_class = 0

	def find_pipette(self, img):
		''' return the x and y position (in pixels) of the pipette in the given image using a YOLO object detection model
			or None if there is no detected pipette in the image
		'''

		if len(img.shape) == 2:
			#neural net expects color image, convert to color
			img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)

		height, width, _ = img.shape #color image

		#run the image through the model
		blob = cv2.dnn.blobFromImage(img, 1 / 255.0, (416, 416), swapRB=True, crop=False)
		self.yoloNet.setInput(blob)
		outs = self.yoloNet.forward(self.output_layers)

		confidences = []
		boxes = []
		for out in outs:
			for detection in out:
				scores = detection[5:]
				class_id = np.argmax(scores)
				confidence = scores[class_id]

				if confidence > 0.5 and class_id == self.pipette_class:
					# Object detected
					center_x = int(detection[0] * width)
					center_y = int(detection[1] * height)

					boxes.append([center_x, center_y])
					confidences.append(float(confidence))

		if len(boxes) == 0:
			return None #no pipette detected
		
		confidence = np.array(confidence)
		best_x, best_y = boxes[confidence.argmax()]

		return best_x, best_y
	
if __name__ == '__main__':
	finder = PipetteFinder()
	img = cv2.imread('/home/nathan/Desktop/82.png')

	#find pipette, draw location to img
	start = time.time()
	x,y = finder.find_pipette(img)
	print(f'framerate: {1 / (time.time() - start)}')
	cv2.circle(img, (x,y), 3, (0, 255, 0))

	#show img
	cv2.imshow("pipette finding test", img)
	cv2.waitKey(0)


		

				