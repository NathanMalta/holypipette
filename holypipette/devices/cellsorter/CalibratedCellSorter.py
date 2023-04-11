import cv2
import numpy as np

from holypipette.devices.camera.camera import Camera
from holypipette.devices.cellsorter import CellSorterManip
from holypipette.devices.cellsorter import CellSorterController
from holypipette.controller import TaskController

__all__ = ['CalibratedCellSorter']

class CalibratedCellSorter(TaskController):
    def __init__(self, cellsorterManip : CellSorterManip, cellSorterController: CellSorterController, stage, microscope, camera : Camera):
        self.cellsorterManip = cellsorterManip
        self.cellsorterController = cellSorterController
        self.stage = stage
        self.camera = camera
        self.microscope = microscope
        self.calibrated = False
        self.pipetteOffset = None

    def pulse_suction(self, duration):
        self.cellsorterController.open_valve_for_time(1, duration)

    def pulse_pressure(self, duration):
        self.cellsorterController.open_valve_for_time(2, duration)
    
    def position(self):
        return self.cellsorterManip.position()
    
    def absolute_move(self, position):
        self.cellsorterManip.absolute_move(position)
    
    def relative_move(self, position):
        self.cellsorterManip.relative_move(position)

    def calibrate(self):
        #make sure stage is calibrated
        if not self.stage.calibrated:
            raise Exception("Stage is not calibrated")
        
        #grab latest image
        img, _ = self.camera.snap()

        #convert to grayscale if needed
        if len(img.shape) > 2:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        #find the center of the tube via hough transform
        img = cv2.medianBlur(img, 5)
        #add channel dimension to make it 3D
        img = img[:, :, np.newaxis]
        circles = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT, 1, 20,
                                      param1=50, param2=30, minRadius=30, maxRadius=100)
        
        if circles is None or len(circles) == 0:
            raise Exception("No circles found in image")
        
        if len(circles) > 1:
            raise Exception("Multiple circles found in image")
        
        #get the center of the pipette
        x, y, r = circles[0][0]

        self.pipetteOffsetPix = np.array([x, y])
        self.pipetteOffsetZ = self.microscope.position() - self.cellsorterManip.position()

        print("Pipette offset: ", self.pipetteOffsetPix)
        print("Pipette Z: ", self.pipetteOffsetZ)

        #draw a circle on the image
        self.camera.show_point([int(x), int(y)], radius=int(r), color=(0, 0, 255))
        self.calibrated = True
    
    def center_cellsorter_on_point(self, point): #x,y in pixels, z in stage units
        x, y, z = point
        if not self.stage.calibrated:
            raise Exception("Stage is not calibrated")
        if not self.calibrated or self.pipetteOffsetPix is None:
            raise Exception("Cell Sorter is not calibrated")
        if self.microscope.floor_Z == None:
            raise Exception("Cell Plane not set")
        

        #move the stage such that it's centered in x, y, put cells in focus
        self.microscope.absolute_move(self.microscope.floor_Z)
        self.stage.reference_move(np.array([x + self.pipetteOffsetPix[0], y + self.pipetteOffsetPix[1]]))
        print("moving stage to cell")
        self.stage.wait_until_still()

        print("moving cellsorter to cell")
        #move the cellsorter to the cell plane
        self.cellsorterManip.absolute_move(z - self.pipetteOffsetZ)
        print('done')

