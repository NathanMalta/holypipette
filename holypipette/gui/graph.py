import logging

from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel
from PyQt5 import QtCore

from pyqtgraph import PlotWidget, plot

import threading
import time

import numpy as np
from collections import deque
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

        #constants for Multi Clamp
        self.externalCommandSensitivity = 20 #mv/V
        self.triggerLevel = 0.05 #V

        #setup window
        self.setWindowTitle("Electrophysiology")

        self.squareWavePlot = PlotWidget()
        self.pressurePlot = PlotWidget()
        self.resistancePlot = PlotWidget()

        #set background color of plots
        self.squareWavePlot.setBackground('w')
        self.pressurePlot.setBackground('w')
        self.resistancePlot.setBackground('w')

        #set axis colors to black
        self.squareWavePlot.getAxis('left').setPen('k')
        self.squareWavePlot.getAxis('bottom').setPen('k')
        self.pressurePlot.getAxis('left').setPen('k')
        self.pressurePlot.getAxis('bottom').setPen('k')
        self.resistancePlot.getAxis('left').setPen('k')
        self.resistancePlot.getAxis('bottom').setPen('k')

        #set labels
        self.squareWavePlot.setLabel('left', "Current", units='A')
        self.squareWavePlot.setLabel('bottom', "Time", units='s')
        self.pressurePlot.setLabel('left', "Pressure", units='mbar')
        self.pressurePlot.setLabel('bottom', "Time", units='s')
        self.resistancePlot.setLabel('left', "Resistance", units='Ohms')
        self.resistancePlot.setLabel('bottom', "Samples", units='')

        self.pressureData = deque(maxlen=100)
        self.resistanceDeque = deque(maxlen=100)

        #create a quarter layout for 4 graphs
        layout = QVBoxLayout()
        layout.addWidget(self.squareWavePlot)
        layout.addWidget(self.pressurePlot)
        layout.addWidget(self.resistancePlot)

        self.setLayout(layout)
        
        self.updateTimer = QtCore.QTimer()
        self.updateDt = 100 #ms
        self.updateTimer.timeout.connect(self.update_plot)
        self.updateTimer.start(self.updateDt)

        #start async daq data update
        self.lastestDaqData = None
        self.daqUpdateThread = threading.Thread(target=self.updateDAQDataAsync, daemon=True)
        self.daqUpdateThread.start()

        #show window and bring to front
        self.raise_()
        self.show()

    def updateDAQDataAsync(self):
        while True:
            self.lastestDaqData, resistance = self.daq.getDataFromSquareWave(10, 50000, 0.5, 0.5, 0.25)
            if resistance is not None:
                self.resistanceDeque.append(resistance)
            time.sleep(0.1)

    def update_plot(self):
        #update current graph
        if self.lastestDaqData is not None:
            self.squareWavePlot.clear()
            self.squareWavePlot.plot(self.lastestDaqData[0], self.lastestDaqData[1])
            self.lastestDaqData = None
        
        #update pressure graph
        self.pressureData.append(self.pressureController.measure())
        pressureX = [i * self.updateDt / 1000 for i in range(len(self.pressureData))]
        self.pressurePlot.clear()
        self.pressurePlot.plot(pressureX, self.pressureData, pen='k')

        #update resistance graph
        self.resistancePlot.clear()
        resistanceDeque = [i for i in range(len(self.resistanceDeque))]
        self.resistancePlot.plot(resistanceDeque, self.resistanceDeque, pen='k')