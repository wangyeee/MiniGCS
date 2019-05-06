from PyQt5.QtCore import Qt  # , pyqtSignal
from PyQt5.QtWidgets import QGridLayout, QLabel, QPushButton
from pymavlink.dialects.v10 import autoquad as mavlink
from plugins.common import AbstractControlPanel

class AutoQuadControlPanel(AbstractControlPanel):

    __mavlinkMessageTypes = ['AQ_TELEMETRY_F', 'AQ_ESC_TELEMETRY']
    cmdSent = 0

    def __init__(self, parent = None):
        super().__init__(parent)
        l = QGridLayout()
        row = 0
        l.addWidget(QLabel('AutoQuad Tools'), row, 0, 1, 3, Qt.AlignLeft)
        row += 1
        self.fromSDButton = QPushButton('From SD')
        self.toSDButton = QPushButton('To SD')
        self.dIMUTareButton = QPushButton('DIMU Tare')
        self.magCalibButton = QPushButton('MAG Calib.')
        self.calibSaveButton = QPushButton('Calib. Save')
        self.fromSDButton.clicked.connect(self.__loadParametersFromSDCard)
        self.toSDButton.clicked.connect(self.__saveParametersToSDCard)
        self.calibSaveButton.clicked.connect(self.__saveDIMUCalibration)
        self.dIMUTareButton.clicked.connect(self.__dIMUTare)
        self.magCalibButton.clicked.connect(self.__magCalib)
        l.addWidget(self.fromSDButton, row, 0, 1, 1, Qt.AlignLeft)
        l.addWidget(self.toSDButton, row, 2, 1, 1, Qt.AlignLeft)
        row += 1
        l.addWidget(self.dIMUTareButton, row, 0, 1, 1, Qt.AlignLeft)
        l.addWidget(self.magCalibButton, row, 2, 1, 1, Qt.AlignLeft)
        row += 1
        l.addWidget(self.calibSaveButton, row, 0, 1, 1, Qt.AlignLeft)
        row += 1
        self.setLayout(l)

    def __saveParametersToSDCard(self):
        print('Save to SD')
        self.__sendMAVLinkLongMessage(command=mavlink.MAV_CMD_PREFLIGHT_STORAGE, param1=2)

    def __loadParametersFromSDCard(self):
        print('Load from SD')
        self.__sendMAVLinkLongMessage(command=mavlink.MAV_CMD_PREFLIGHT_STORAGE, param1=3)

    def __loadDIMUCalibration(self):
        print('Calib. Load')
        self.__sendMAVLinkLongMessage(target_component=mavlink.MAV_COMP_ID_IMU, command=mavlink.MAV_CMD_PREFLIGHT_STORAGE, param1=0)

    def __saveDIMUCalibration(self):
        print('Calib. Save')
        self.__sendMAVLinkLongMessage(target_component=mavlink.MAV_COMP_ID_IMU, command=mavlink.MAV_CMD_PREFLIGHT_STORAGE, param1=1)

    def __sendMAVLinkLongMessage(self, target_system = 255, # set target system to 255 to let telemetry.py auto correct the values
                                       target_component = 0,
                                       command = 0,
                                       confirmation = 0,
                                       param1 = 0,
                                       param2 = 0,
                                       param3 = 0,
                                       param4 = 0,
                                       param5 = 0,
                                       param6 = 0,
                                       param7 = 0):
        msg = mavlink.MAVLink_command_long_message(target_system, target_component,
                                                   command, confirmation,
                                                   param1, param2, param3,
                                                   param4, param5, param6, param7)
        self.__addMessageType('COMMAND_ACK')
        self.cmdSent = command
        self.mavlinkTxSignal.emit(msg)

    def __dIMUTare(self):
        self.__sendMAVLinkLongMessage(command=mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, param5=1)

    def __magCalib(self):
        self.__sendMAVLinkLongMessage(command=mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, param2=1)

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
