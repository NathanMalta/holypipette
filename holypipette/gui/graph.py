from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel
from PyQt5 import QtCore
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure

from holypipette.devices.amplifier import DAQ

__all__ = ["EPhysGraph"]

class EPhysGraph(QWidget):
    """A window that plots electrophysiology data from the DAQ
    """
    
    def __init__(self, daq : DAQ):
        super().__init__()

        self.daq = daq

        #setup window
        layout = QVBoxLayout()
        self.setWindowTitle("Electrophysiology")

        #add matplotlib graph
        self.figure = Figure()
        self.axes = self.figure.add_subplot(111)
        self.canvas = FigureCanvasQTAgg(self.figure)
        layout.addWidget(self.canvas)
        self.setLayout(layout)

        #update plot every 500ms
        timer = QtCore.QTimer(self)
        timer.timeout.connect(self.update_plot)
        timer.start(500)

        #show window and bring to front
        self.show()
        self.raise_()

    def update_plot(self):
        #update data
        data = self.daq.getDataFromSquareWave(10, 1000, 0.5, 0.5)
        self.plot(data[0], data[1])

    def plot(self, x, y):
        self.axes.clear()
        self.axes.plot(x, y)
        self.canvas.draw()