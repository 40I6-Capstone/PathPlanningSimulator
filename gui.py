import sys
from sim import Sim
from PySide6.QtCore import Qt, Slot, Signal
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QStyle, QVBoxLayout, QHBoxLayout, 
                                QPushButton, QLabel, QDialogButtonBox, QSlider, QCheckBox,
                                QSpinBox, QSpacerItem)
from PySide6.QtGui import (QIcon)
import ast
import pyqtgraph as pg
import numpy as np
import threading
from time import sleep

class MapWindow(QMainWindow):
    updateRobotPosSignal = Signal();
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
        self.pointPen = pg.mkBrush(color=(100,100,0), width=2);

        self.validRobotRadPen = pg.mkPen(color=(0, 100, 0), width=5);
        self.collisionRobotRadPen = pg.mkPen(color=(100, 0, 0), width=5);

        robotColourMap = pg.colormap.getFromMatplotlib("rainbow");
        self.robotColourTable = robotColourMap.getLookupTable(0, 1, 101);


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

        self.robotRadPlot = [];
        robotRadPath = self.makeCirclePath(0, 0);
        for robot in self.sim.robots:
            self.robotRadPlot.append(self.mapPlot.plot(robotRadPath[:, 0], robotRadPath[:, 1], pen=self.validRobotRadPen));

        self.robotPlot = self.mapPlot.plot([],[], pen=None, symbol="+");

        self.playPauseButton = QPushButton();
        self.playPauseButton.setIcon(QApplication.style().standardIcon(QStyle.StandardPixmap.SP_MediaPlay));
        self.playPauseButton.clicked.connect(self.pushPlayPauseButton);
        self.isPlaying = False;

        self.slider = QSlider(Qt.Horizontal);
        self.slider.setMinimum(0);
        self.slider.setMaximum(int(self.sim.simTime/self.sim.dt));
        self.slider.setValue(0);
        self.slider.setTickInterval(1);
        self.slider.valueChanged.connect(self.sliderUpdateValue);
        self.ifSliderDragged = False;

        minVal = QLabel("0 sec");
        maxVal = QLabel(str(self.sim.simTime) + " sec");

        sliderLayout = QHBoxLayout();
        sliderLayout.addWidget(self.playPauseButton);
        sliderLayout.addWidget(minVal);
        sliderLayout.addWidget(self.slider);
        sliderLayout.addWidget(maxVal);

        layout = QVBoxLayout(self.__main);
        layout.addWidget(self.mapPlot);
        layout.addLayout(sliderLayout);

        self.showMaximized();

        self.updateRobotPosSignal.connect(self.updateRobotPos);

        self.startSim();

    def startSim(self):
        self.simThread = threading.Thread(target=self.animate, daemon=True);
        self.simThread.start();
    
    def animate(self):
        while(self.sim.time < self.sim.simTime):
            if(self.ifSliderDragged):
                # print(self.slider.value()*self.sim.dt);
                self.sim.updateRobots(self.slider.value()*self.sim.dt);
                self.updateRobotPosSignal.emit();
                self.ifSliderDragged = False;
            elif(self.isPlaying):
                sleep(self.sim.dt);
                self.sim.updateRobots();
                self.updateRobotPosSignal.emit();
            

    def makeCirclePath(self, xCent,yCent):
        points = [];
        for angle in np.linspace(0, 2*np.pi, 100):
            x = self.sim.robotRad*np.cos(angle) + xCent;
            y = self.sim.robotRad*np.sin(angle) + yCent;
            points.append([x, y]);
        return (np.array(points));

    @Slot()
    def updateRobotPos(self):
        print("updated plot");
        linspace = np.linspace(0, 100, len(self.sim.robots)).astype(int);
        brushes = self.robotColourTable[linspace];
        xpoints = [];
        ypoints = [];
        for index, robot in enumerate(self.sim.robots):
            xpoints.append(robot.pos[0]);
            ypoints.append(robot.pos[1]);
            radPath = self.makeCirclePath(robot.pos[0],robot.pos[1]);
            pen = self.collisionRobotRadPen if robot.checkCollision(self.sim.robots) else self.validRobotRadPen;
            self.robotRadPlot[index].setData(radPath[:,0], radPath[:,1], pen=pen);

        self.robotPlot.setData(xpoints,ypoints, symbolBrush=brushes);
        self.slider.blockSignals(True);
        self.slider.setValue(int(self.sim.time/self.sim.dt));
        self.slider.blockSignals(False);
    
    @Slot()
    def pushPlayPauseButton(self):
        if(self.isPlaying):
            self.playPauseButton.setIcon(QApplication.style().standardIcon(QStyle.StandardPixmap.SP_MediaPlay));
            self.isPlaying = False;
        else:
            self.playPauseButton.setIcon(QApplication.style().standardIcon(QStyle.StandardPixmap.SP_MediaPause));
            self.isPlaying = True;
    
    @Slot()
    def sliderUpdateValue(self):
        self.ifSliderDragged = True;
