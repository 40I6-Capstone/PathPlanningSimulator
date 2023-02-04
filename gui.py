import sys
from PySide6.QtCore import Qt, Slot, Signal
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QTabWidget, QVBoxLayout, QHBoxLayout, 
                                QPushButton, QLabel, QDialog, QDialogButtonBox, QSlider, QCheckBox,
                                QSpinBox, QSpacerItem)
import pyqtgraph as pg

class MapWindow(QMainWindow):

    def __init__(self):
        super().__init__();
        self.setWindowTitle("Path Planning Map");
        self.__main = QWidget();
        self.setCentralWidget(self.__main);

class GUI:
    def __init__(self):
        self.application = QApplication([]);
        self.mapWindow = MapWindow();
        self.mapWindow.show();
        sys.exit(self.mapWindow.exec());