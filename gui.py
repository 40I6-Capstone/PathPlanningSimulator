import sys
from sim import Sim
from PySide6.QtCore import Qt, Slot, Signal
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QTabWidget, QVBoxLayout, QHBoxLayout, 
                                QPushButton, QLabel, QDialog, QDialogButtonBox, QSlider, QCheckBox,
                                QSpinBox, QSpacerItem)
import ast
import pyqtgraph as pg
import numpy as np
import threading

class MapWindow(QMainWindow):
    updatePathSignal = Signal(str);
    def __init__(self, simulation: Sim):
        super().__init__();
        self.setWindowTitle("Path Planning Map");
        self.__main = QWidget();
        self.setCentralWidget(self.__main);


        infoText = QLabel("*Changes will only affect UGV position, velocity, and heading plots");
        infoText.setObjectName("PlainText"); 

        self.mapPlot = pg.PlotWidget();

        self.sim = simulation;
        
        self.pathPen = pg.mkPen(color=(100,0,100), width=5);
        self.shapePen = pg.mkPen(color=(100,0,0), width=5);
        self.pointPen = pg.mkBrush(color=(100,100,0), width=5);


        # self.pathPlot.setBackground(mainColour);
        self.mapPlot.addLegend();
        self.mapPlot.setLabel('left', 'Y position');
        self.mapPlot.setLabel('bottom', 'X Position');
        self.mapPlot.showGrid(x=True,y=True);
        self.mapPlot.setAspectLocked(True);

        self.shape = self.mapPlot.plot(self.sim.shape.vertices[:,0], self.sim.shape.vertices[:,1], pen=self.shapePen);
        self.shape_mid = self.mapPlot.plot(self.sim.shape.midpoints[:,0], self.sim.shape.midpoints[:,1], pen=None, symbolBrush=self.pointPen);

        self.pathsPlot = [];
        for path in self.sim.pathPlan.paths:
            self.pathsPlot.append(self.mapPlot.plot(path.points[:,0], path.points[:,1], pen=self.pathPen));

        layout = QVBoxLayout(self.__main);
        layout.addWidget(self.mapPlot);
        
        self.showMaximized();

        self.updatePathSignal.connect(self.updatePath);

    @Slot(str)
    def updatePath(self, value):
        pointsDict = ast.literal_eval(value);
        points = np.array(pointsDict["points"]);
        print(pointsDict["index"]);
        # self.paths[pointsDict["index"]].setData(points[:,0], points[:,1]);


    def setPath(self, index, points: np.ndarray):
        pointsList = points.tolist();
        valueDict = {
            "index": index,
            "points": pointsList
        };
        value = str(valueDict);
        self.updatePathSignal.emit(value);
