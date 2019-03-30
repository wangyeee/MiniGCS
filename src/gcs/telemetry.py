from pymavlink import mavutil
from enum import Enum
from PyQt5.QtCore import Qt, QThread, QVariant, pyqtSignal
from PyQt5.QtWidgets import (QComboBox, QGridLayout, QLabel, QPushButton,
                             QSizePolicy, QWidget)
from serial.tools.list_ports import comports
from waypoint import Waypoint


BAUD_RATES = {
    110 : '110',
    300 : '300',
    600 : '600',
    1200 : '1200',
    2400 : '2400',
    4800 : '4800',
    9600 : '9600',
    14400 : '14400',
    19200 : '19200',
    38400 : '38400',
    56000 : '56000',
    57600 : '57600',
    115200 : '115200',
    128000 : '128000',
    230400 : '230400',
    256000 : '256000',
    406800 : '406800',
    921600 : '921600'
}

FLOW_CONTROL = {
    0 : 'None',
    1 : 'HW',
    2 : 'SW'
}

PARITY = {
    0 : 'None',
    1 : 'Odd',
    2 : 'Even'
}

DATA_BITS = {
    8 : '8',
    7 : '7',
    6 : '6',
    5 : '5'
}

STOP_BITS = {
    1 : '1',
    2 : '2'
}

class MavStsKeys(Enum):
    AP_SYS_ID = 0
    VEHICLE_TYPE = 1
    AP_TYPE = 2
    AP_MODE = 3
    CUSTOM_AP_MODE = 4
    AP_SYS_STS = 5
    MAVLINK_VER = 6

class ConnectionEditWindow(QWidget):

    portList = {}

    connectToMAVLink = pyqtSignal(object)

    def __init__(self, parent = None):
        super().__init__(parent)
        self.listSerialPorts()
        self.setWindowTitle('Serial Link')
        l = QGridLayout()
        row = 0

        lbl, self.portsDropDown = self._createDropDown('Serial Port', self.portList)
        l.addWidget(lbl, row, 0, 1, 1, Qt.AlignRight)
        l.addWidget(self.portsDropDown, row, 1, 1, 3, Qt.AlignLeft)
        row += 1

        lbl, self.baudDropDown = self._createDropDown('Baud Rate', BAUD_RATES)
        l.addWidget(lbl, row, 0, 1, 1, Qt.AlignRight)
        l.addWidget(self.baudDropDown, row, 1, 1, 3, Qt.AlignLeft)
        row += 1

        lbl, self.flowDropDown = self._createDropDown('Flow Control', FLOW_CONTROL)
        l.addWidget(lbl, row, 0, 1, 1, Qt.AlignRight)
        l.addWidget(self.flowDropDown, row, 1, 1, 1, Qt.AlignLeft)

        lbl, self.parityDropDown = self._createDropDown('Parity', PARITY)
        l.addWidget(lbl, row, 2, 1, 1, Qt.AlignRight)
        l.addWidget(self.parityDropDown, row, 3, 1, 1, Qt.AlignLeft)
        row += 1

        lbl, self.bitsDropDown = self._createDropDown('Data Bits', DATA_BITS)
        l.addWidget(lbl, row, 0, 1, 1, Qt.AlignRight)
        l.addWidget(self.bitsDropDown, row, 1, 1, 1, Qt.AlignLeft)

        lbl, self.stopDropDown = self._createDropDown('Stop Bits', STOP_BITS)
        l.addWidget(lbl, row, 2, 1, 1, Qt.AlignRight)
        l.addWidget(self.stopDropDown, row, 3, 1, 1, Qt.AlignLeft)
        row += 1

        self.connectButton = QPushButton('Connect')
        self.closeButton = QPushButton('Close')
        l.addWidget(self.connectButton, row, 2, 1, 1, Qt.AlignCenter)
        l.addWidget(self.closeButton, row, 3, 1, 1, Qt.AlignCenter)
        self.connectButton.clicked.connect(self.connectSerialPort)
        self.closeButton.clicked.connect(self.cancelConnection)
        self.setLayout(l)

    def _createDropDown(self, label, data: dict):
        dropDown = QComboBox(self)
        dropDown.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        for key, val in data.items():
            dropDown.addItem(str(val), QVariant(key))
        return QLabel(label), dropDown

    def listSerialPorts(self):
        portsInfo = sorted(comports(False))
        cnts = 0
        for p in portsInfo:
            self.portList[p.device] = p
            cnts += 1
        if cnts == 0:
            self.portList['No ports available'] = 'No ports available'
        # print(self.portList)

    def cancelConnection(self):
        self.close()

    def connectSerialPort(self):
        port = self.portsDropDown.currentData()
        baud = self.baudDropDown.currentData()
        # print('{} -- {}'.format(port, baud))
        mavutil.set_dialect('autoquad')  # test
        connection = mavutil.mavlink_connection(port, int(baud))
        self.connectToMAVLink.emit(connection)
        self.close()

class MAVLinkConnection(QThread):

    gpsRawIntHandler = pyqtSignal(object)
    localPositionNEDHandler = pyqtSignal(object)
    navControllerOutputHandler = pyqtSignal(object)
    radioStatusHandler = pyqtSignal(object)
    rcChannelRawHandler = pyqtSignal(object)
    scaledIMUHandler = pyqtSignal(object)
    scaledPressureHandler = pyqtSignal(object)
    heartBeatHandler = pyqtSignal(object)
    altitudeHandler = pyqtSignal(object)
    systemStatusHandler = pyqtSignal(object)
    parameterValueHandler = pyqtSignal(object)

    handlerLookup = {}
    mavStatus = {MavStsKeys.AP_SYS_ID : 1}
    isConnected = False

    def __init__(self, connection):
        super().__init__()
        self.running = True
        self.connection = connection
        self.handlerLookup['HEARTBEAT'] = self.heartBeatHandler
        self.handlerLookup['ATTITUDE'] = self.altitudeHandler
        self.handlerLookup['GPS_RAW_INT'] = self.gpsRawIntHandler
        self.handlerLookup['LOCAL_POSITION_NED'] = self.localPositionNEDHandler
        self.handlerLookup['NAV_CONTROLLER_OUTPUT'] = self.navControllerOutputHandler
        self.handlerLookup['RADIO_STATUS'] = self.radioStatusHandler
        self.handlerLookup['RC_CHANNELS_RAW'] = self.rcChannelRawHandler
        self.handlerLookup['SCALED_IMU'] = self.scaledIMUHandler
        self.handlerLookup['SCALED_PRESSURE'] = self.scaledPressureHandler
        self.handlerLookup['SYS_STATUS'] = self.systemStatusHandler
        self.handlerLookup['PARAM_VALUE'] = self.parameterValueHandler
        self.parameterValueHandler.connect(self.receiveOnboardParameter)

    def requestExit(self):
        # print('exit conn thread...')
        self.running = False

    def run(self):
        # print('waiting for heart beat...')
        self._establishConnection()
        while self.running:
            try:
                msg = self.connection.recv_match(blocking=False)
                if msg != None:
                    msgType = msg.get_type()
                    if msgType in self.handlerLookup:
                        self.handlerLookup[msgType].emit(msg)
                    else:
                        print('UNKNOWN MSG:', msg)
            except UnicodeDecodeError as e:
                # print(e)
                raise e

        self.connection.close()
        # print('connection closed')

    def _establishConnection(self):
        hb = self.connection.wait_heartbeat()
        self.mavStatus[MavStsKeys.VEHICLE_TYPE] = hb.type
        self.mavStatus[MavStsKeys.AP_TYPE] = hb.autopilot
        self.mavStatus[MavStsKeys.AP_MODE] = hb.base_mode
        self.mavStatus[MavStsKeys.CUSTOM_AP_MODE] = hb.custom_mode
        self.mavStatus[MavStsKeys.AP_SYS_STS] = hb.system_status
        self.mavStatus[MavStsKeys.MAVLINK_VER] = hb.mavlink_version
        # request all parameters
        self.connection.param_fetch_all()

    def receiveOnboardParameter(self, msg):
        print(msg)
        if msg.param_index + 1 == msg.param_count:
            self.isConnected = True
            print('connection established.')

    def navigateToWaypoint(self, wp: Waypoint):
        print('Goto', wp)
        # TODO sent MAV_CMD_NAV_WAYPOINT message to UAV
        # mavlink.mission_item_send(self.target_system,
        #                           self.target_component,
        #                           0,
        #                           self.get_default_frame(),
        #                           mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        #                           2, 0, 0, 0, 0, 0,
        #                           wp.latitude, wp.longitude, wp.altitude)