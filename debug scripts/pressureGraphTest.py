import numpy as np   
def update_pressure_graph(self):
    self.graph_data = np.roll(self.graph_data, -1, axis=0)
    self.graph_data[-1, 0] = self.graph_data[-2, 0] + 1
    self.graph_data[-1, 1] = self.patch_interface.pressure.get_pressure()

    #only plot the most recent 100 points
    self.graph_widget.clear()
    self.graph_widget.plot(self.graph_data[-100:, 0], self.graph_data[-100:, 1], pen=None, symbol='o')
    
    #autoscale the graph
    self.graph_widget.enableAutoRange('xy', True)


def create_pressure_graph():
    self.graph_data = np.zeros((100, 2))

    #add numpy data to the graph as a scatter plot
    self.graph_widget = pg.PlotWidget()
    self.graph_widget.plot(self.graph_data[:, 0], self.graph_data[:, 1], pen=None, symbol='o')
    self.graph_layout = QtWidgets.QVBoxLayout()
    layout.addWidget(self.graph_widget)

    #periodically update the graph
    self.graph_timer = QtCore.QTimer()
    self.graph_timer.timeout.connect(self.update_pressure_graph)
    self.graph_timer.start(500)

    #shrink the graph to fit the window
    self.graph_widget.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
