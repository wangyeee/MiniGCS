from PyQt5.QtCore import Qt  # , pyqtSignal
from PyQt5.QtWidgets import QGridLayout, QLabel, QPushButton
from pymavlink.dialects.v10 import autoquad as mavlink
from plugins.common import AbstractControlPanel

class AutoQuadControlPanel(AbstractControlPanel):

    def __init__(self, parent = None):
        super().__init__(parent)
        l = QGridLayout()
        row = 0
        l.addWidget(QLabel('AutoQuad Tools'), row, 0, 1, 3, Qt.AlignLeft)
        row += 1
        self.dIMUTareButton = QPushButton('DIMU Tare')
        self.magCalibButton = QPushButton('MAG Calib.')
        self.dIMUTareButton.clicked.connect(self.__dIMUTare)
        self.magCalibButton.clicked.connect(self.__magCalib)
        l.addWidget(self.dIMUTareButton, row, 0, 1, 1, Qt.AlignLeft)
        l.addWidget(self.magCalibButton, row, 2, 1, 1, Qt.AlignLeft)
        row += 1
        self.setLayout(l)

    def __dIMUTare(self):
        # set target system to 255 to let telemetry.py auto correct the values
        msg = mavlink.MAVLink_command_long_message(255, 0, #target_system, mav.target_component,
                 mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 0,
                 0, 0, 0, 0, 1, 0, 0)
        print('DIMU tare:', msg)
        self.mavlinkTxSignal.emit(msg)

    def __magCalib(self):
        print('MAG calib.')

    def tabName(self):
        return 'AutoQuad'

    def registerMavlinkMessageListeners(self):
        return ['AQ_TELEMETRY_F', 'AQ_ESC_TELEMETRY']
