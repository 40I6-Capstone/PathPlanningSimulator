from sim import Sim
from pathPlanning import PathPlanning
from PySide6.QtCore import Qt, Slot, Signal
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QStyle, QVBoxLayout, QHBoxLayout, 
                                QPushButton, QLabel, QSlider)
import ast
import pyqtgraph as pg
import numpy as np
import threading
from time import sleep

class MapWindow(QMainWindow):
    updateRobotPosSignal = Signal();
    updatePathSignal = Signal();
    def __init__(self, simulation: Sim):
        super().__init__();
        self.setWindowTitle("Path Planning Map");
        self.__main = QWidget();
        self.setCentralWidget(self.__main);

        #get environment data from file
        env_data_file = open("env_data.txt", "r");
        env_data_str = env_data_file.read();
        self.env_data = ast.literal_eval(env_data_str);

        infoText = QLabel("*Changes will only affect UGV position, velocity, and heading plots");
        infoText.setObjectName("PlainText"); 

        self.mapPlot = pg.PlotWidget();

        self.sim = simulation;
        
        self.pathPen = pg.mkPen(color=(100,0,100), width=5);
        self.shapePen = pg.mkPen(color=(100,0,0), width=5);
        self.pointPen = pg.mkBrush(color=(100,100,0), width=2);

        self.validRobotRadPen = pg.mkPen(color=(0, 100, 0), width=5);
        self.collisionRobotRadPen = pg.mkPen(color=(100, 0, 0), width=5);
        self.critRadPen = pg.mkPen(color=(100, 0, 0), width=1, style=Qt.DashLine);

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

        self.crit_rad = self.env_data["crit_rad"];
        critRadPoints = self.plot_crit_rad();
        self.critRadPlot = self.mapPlot.plot(critRadPoints[:,0], critRadPoints[:,1], pen=self.critRadPen);

        self.pathsPlot = [];

        self.robotRadPlot = [self.mapPlot.plot([],[], pen=self.validRobotRadPen) for x in range(0,self.sim.nodeCount)];

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
        self.time = 0.0;
        
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
        self.updatePathSignal.connect(self.updatePath);

        self.startSim();

    def startSim(self):
        self.simThread = threading.Thread(target=self.animate, daemon=True);
        self.simThread.start();
    
    def animate(self):


        self.pathPlan = PathPlanning();
        self.pathPlan.planPath(self.sim.shape, self.env_data["crit_rad"], self.sim.robotRad, self.env_data["spoke_len"], self.updatePathSignal);

        self.sim.setPath(self.pathPlan);
        self.sim.nodeSetup();
        self.sim.runSim();
        
        while(self.time < self.sim.simTime):
            if(self.ifSliderDragged):
                self.time = self.slider.value()*self.sim.dt;
                self.sim.getPos(self.time);
                self.updateRobotPosSignal.emit();
                self.ifSliderDragged = False;
            elif(self.isPlaying):
                sleep(self.sim.dt);
                self.time = self.time + self.sim.dt;
                print("time",self.time);
                self.sim.getPos(self.time);
                self.updateRobotPosSignal.emit();

    def plot_crit_rad(self):
        points = [];
        for angle in np.linspace(0, 2*np.pi, 100):
            x = self.crit_rad*np.cos(angle);
            y = self.crit_rad*np.sin(angle);
            points.append([x, y]);
        return (np.array(points));

    def makeRobotCirclePath(self, xCent,yCent):
        points = [];
        for angle in np.linspace(0, 2*np.pi, 100):
            x = self.sim.robotRad*np.cos(angle) + xCent;
            y = self.sim.robotRad*np.sin(angle) + yCent;
            points.append([x, y]);
        return (np.array(points));

    @Slot()
    def updatePath(self):
        if(not self.crit_rad == self.pathPlan.crit_rad):
            self.crit_rad = self.pathPlan.crit_rad;
            print("crit rad", self.crit_rad)
            critRadPoints = self.plot_crit_rad();
            self.critRadPlot.setData(critRadPoints[:,0], critRadPoints[:,1]);

        for index,path in enumerate(self.pathPlan.paths):
            if(index < len(self.pathsPlot)):
                self.pathsPlot[index].setData(path.points[:,0], path.points[:,1]);
            else:
                self.pathsPlot.append(self.mapPlot.plot(path.points[:,0], path.points[:,1], pen=self.pathPen));

    @Slot()
    def updateRobotPos(self):
        linspace = np.linspace(0, 100, len(self.sim.robots)).astype(int);
        brushes = self.robotColourTable[linspace];
        xpoints = [];
        ypoints = [];
        for index, robot in enumerate(self.sim.robots):
            xpoints.append(robot.pos[0]);
            ypoints.append(robot.pos[1]);
            radPath = self.makeRobotCirclePath(robot.pos[0],robot.pos[1]);
            pen = self.collisionRobotRadPen if robot.checkCollision(self.sim.robots) else self.validRobotRadPen;
            self.robotRadPlot[index].setData(radPath[:,0], radPath[:,1], pen=pen);

        self.robotPlot.setData(xpoints,ypoints, symbolBrush=brushes);
        self.slider.blockSignals(True);
        self.slider.setValue(int(self.time/self.sim.dt));
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
