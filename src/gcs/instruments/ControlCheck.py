from PyQt5.QtWidgets import QWidget, QSlider, QHBoxLayout, QVBoxLayout, QLabel, QPushButton
from PyQt5.QtCore import Qt, pyqtSignal
from time import time
from pymavlink import mavutil

class OutputSlider(QWidget):

    outputUpdateSignal = pyqtSignal(int, int)  # seq, val

    def __init__(self, seq, minVal, maxVal, parent = None):
        super().__init__(parent)
        self.seq = seq
        self.setLayout(QHBoxLayout())
        self.outputUpdateMinInterval = 100  # ms
        self.lastUpdateTime = time() * 1000
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(minVal, maxVal)
        self.valueLabel = QLabel(str(minVal))
        self.slider.valueChanged.connect(self.setOutputValue)
        self.slider.sliderReleased.connect(lambda: self.setOutputValue(self.slider.minimum() - 1))
        self.slider.valueChanged.connect(lambda val: self.valueLabel.setText(str(val)))
        self.layout().addWidget(QLabel('Output {}'.format(seq)))
        self.layout().addWidget(self.slider)
        self.layout().addWidget(self.valueLabel)

    def setOutputValue(self, val):
        if val < self.slider.minimum():
            self.outputUpdateSignal.emit(self.seq, self.slider.value())
        else:
            now = time() * 1000
            if now - self.lastUpdateTime > self.outputUpdateMinInterval:
                self.outputUpdateSignal.emit(self.seq, val)
                self.lastUpdateTime = now

class ServoOutputCheckWindow(QWidget):

    mavlinkMotorTestSignal = pyqtSignal(object)  # set servo msg

    def __init__(self, opNumber, parent = None):
        super().__init__(parent)
        self.setWindowTitle('Servo Output Check')
        self.setLayout(QVBoxLayout())
        self.outputEnable = False
        self.outputPanels = []
        for i in range(opNumber):
            op = OutputSlider(i + 1, 1000, 2000)
            op.outputUpdateSignal.connect(self.processSliderOutput)
            self.outputPanels.append(op)
            self.layout().addWidget(op)
        self.outputControlPanel = QWidget()
        self.outputControlPanel.setLayout(QHBoxLayout())
        self.armButton = QPushButton('Arm')
        self.armButton.setStyleSheet('background-color: red')
        self.armButton.clicked.connect(self.__armOutput)
        self.closeButton = QPushButton('Close')
        self.closeButton.clicked.connect(self.__close)
        self.outputControlPanel.layout().addWidget(self.armButton)
        self.outputControlPanel.layout().addStretch(1)
        self.outputControlPanel.layout().addWidget(self.closeButton)
        self.layout().addWidget(self.outputControlPanel)

    def processSliderOutput(self, seq, val):
        if self.outputEnable:
            msg = mavutil.mavlink.MAVLink_command_long_message(255, 255,
                                                               mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
                                                               seq, val, 0, 0, 0, 0, 0)
            self.mavlinkMotorTestSignal.emit(msg)

    def showEvent(self, event):
        super().showEvent(event)
        self.__disableOutput()

    def hideEvent(self, event):
        super().hideEvent(event)
        self.__disableOutput()

    def __armOutput(self):
        if self.outputEnable:
            self.__disableOutput()
        else:
            self.outputEnable = True
            self.armButton.setText('Disarm')

    def __disableOutput(self):
        self.outputEnable = False
        self.armButton.setText('Arm')

    def __close(self):
        self.__disableOutput()
        self.close()
