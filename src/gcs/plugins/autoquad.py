from PyQt5.QtCore import Qt  # , pyqtSignal
from PyQt5.QtWidgets import QGridLayout, QLabel, QPushButton
from pymavlink.dialects.v10 import autoquad as mavlink
from plugins.common import AbstractControlPanel

class AutoQuadControlPanel(AbstractControlPanel):

    __mavlinkMessageTypes = ['AQ_TELEMETRY_F', 'AQ_ESC_TELEMETRY']
    cmdSent = 0

    def __init__(self, parent = None):
        '''
        layout:
        From SD       To SD
        DIMU Tare     MAG Calib.
        Calib. Save
        '''
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
        msg = mavlink.MAVLink_command_long_message(255, 0, mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 0,
                                                   0, 0, 0, 0, 1, 0, 0)
        self.__addMessageType('COMMAND_ACK')
        self.cmdSent =  mavlink.MAV_CMD_PREFLIGHT_CALIBRATION
        self.mavlinkTxSignal.emit(msg)

    def __magCalib(self):
        msg = mavlink.MAVLink_command_long_message(255, 0, mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 0,
                                                   0, 1, 0, 0, 0, 0, 0)
        print('MAG calib:', msg)
        self.__addMessageType('COMMAND_ACK')
        self.cmdSent =  mavlink.MAV_CMD_PREFLIGHT_CALIBRATION
        self.mavlinkTxSignal.emit(msg)

    def tabName(self):
        return 'AutoQuad'

    def __addMessageType(self, msgType):
        if msgType not in self.__mavlinkMessageTypes:
            self.__mavlinkMessageTypes.append(msgType)

    def __removeMessageType(self, msgType):
        if msgType in self.__mavlinkMessageTypes:
            self.__mavlinkMessageTypes.remove(msgType)

    def registerMavlinkMessageListeners(self):
        return self.__mavlinkMessageTypes

    def __mavlinkMessageReceived(self, msg):
        if msg.get_type() == 'COMMAND_ACK' and msg.command == self.cmdSent:
            print('Command result: {}'.format(self.cmdSent))
            self.__removeMessageType('COMMAND_ACK')
