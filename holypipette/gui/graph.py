import logging

from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel
from PyQt5 import QtCore

from pyqtgraph import PlotWidget, plot

import numpy as np

from holypipette.devices.amplifier import DAQ
from holypipette.devices.pressurecontroller import PressureController

__all__ = ["EPhysGraph"]

class EPhysGraph(QWidget):
    """A window that plots electrophysiology data from the DAQ
    """
    
    def __init__(self, daq : DAQ, pressureController : PressureController, parent=None):
        super().__init__()

        #stop matplotlib font warnings
        logging.getLogger('matplotlib.font_manager').disabled = True

        self.daq = daq
        self.pressureController = pressureController

        #setup window
        self.setWindowTitle("Electrophysiology")

        self.squareWavePlot = PlotWidget()
        self.pressurePlot = PlotWidget()

        #set background color of plots
        self.squareWavePlot.setBackground('w')
        self.pressurePlot.setBackground('w')

        #set labels
        self.squareWavePlot.setLabel('left', "Voltage", units='V')
        self.squareWavePlot.setLabel('bottom', "Time", units='s')
        self.pressurePlot.setLabel('left', "Pressure", units='mbar')
        self.pressurePlot.setLabel('bottom', "Time", units='s')

        self.pressureData = []

        #create a quarter layout for 4 graphs
        layout = QVBoxLayout()
        layout.addWidget(self.squareWavePlot)
        layout.addWidget(self.pressurePlot)

        self.setLayout(layout)
        
        self.updateTimer = QtCore.QTimer()
        self.updateTimer.timeout.connect(self.update_plot)
        self.updateTimer.start(100)

        #show window and bring to front
        self.raise_()
        self.show()


    def update_plot(self):
        #update data
        squareWaveData = self.daq.getDataFromSquareWave(10, 2000, 0.5, 0.1, 0.1)
        self.squareWavePlot.plot(squareWaveData[0], squareWaveData[1])


        self.pressureData.append(self.pressureController.measure())
        pressureX = [i for i in range(len(self.pressureData))]
        self.pressurePlot.plot(pressureX, self.pressureData)