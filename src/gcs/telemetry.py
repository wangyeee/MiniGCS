import os
from enum import Enum
from time import time
from pymavlink import mavutil
from pymavlink.mavwp import MAVWPLoader
from pymavlink.dialects.v10 import common as mavlink
from PyQt5.QtCore import (QMutex, Qt, QThread, QTimer, QVariant,
                          QWaitCondition, pyqtSignal)
from PyQt5.QtWidgets import (QComboBox, QGridLayout, QLabel, QPushButton, QLineEdit, QFileDialog,
                             QSizePolicy, QWidget, QTabWidget, QVBoxLayout, QHBoxLayout, QMessageBox)
from serial.tools.list_ports import comports

from parameters import ParameterPanel
from waypoint import Waypoint
from UserData import UserData

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

    connectToMAVLink = pyqtSignal(object)

    def __init__(self, parent = None):
        super().__init__(parent)
        self.tabs = QTabWidget(self)
        self.serialConnTab = SerialConnectionEditTab(self)
        self.logReplayTab = LogFileReplayEditTab(self)
        self.tabs.addTab(self.serialConnTab, 'Serial Link')
        self.tabs.addTab(self.logReplayTab, 'Log File Replay')
        l = QVBoxLayout()
        l.setContentsMargins(0, 0, 0, 0)
        l.addWidget(self.tabs)
        l.addWidget(self.__createActionButtons())
        self.setLayout(l)

    def __createActionButtons(self):
        l = QHBoxLayout()
        l.setContentsMargins(5, 0, 5, 5)
        self.connectButton = QPushButton('Connect')
        self.closeButton = QPushButton('Close')
        self.connectButton.clicked.connect(self._doConnect)
        self.closeButton.clicked.connect(self.close)
        l.addWidget(self.connectButton)
        l.addWidget(self.closeButton)
        self.actionButtonWidget = QWidget()
        self.actionButtonWidget.setLayout(l)
        return self.actionButtonWidget

    def _doConnect(self):
        currTab = self.tabs.currentWidget()
        if hasattr(currTab, 'doConnect'):
            if currTab.doConnect():
                self.close()

class LogFileReplayEditTab(QWidget):

    def __init__(self, parent):
        super().__init__(parent)
        self.connectToMAVLink = parent.connectToMAVLink
        l = QVBoxLayout()
        l.setAlignment(Qt.AlignTop)
        lbl = QLabel('Choose Log File')
        l.addWidget(lbl)

        fileWidget = QWidget(self)
        l1 = QHBoxLayout()
        self.logFilePathEdit = QLineEdit(self)
        sp = self.logFilePathEdit.sizePolicy()
        sp.setHorizontalStretch(1)
        self.logFilePathEdit.setSizePolicy(sp)
        l1.addWidget(self.logFilePathEdit)
        self.browseButton = QPushButton('Browse')
        self.browseButton.clicked.connect(self.__chooseLogFile)
        l1.addWidget(self.browseButton)
        fileWidget.setLayout(l1)

        l.addWidget(fileWidget)
        self.setLayout(l)

    def doConnect(self):
        fileName = self.logFilePathEdit.text()
        if os.path.isfile(fileName):
            print('Replay Log file:', fileName)
            connection = mavutil.mavlogfile(fileName)
            self.connectToMAVLink.emit(connection)
            return True
        QMessageBox.critical(self.window(), 'Error', 'Invalid log file: {}'.format(fileName), QMessageBox.Ok)
        return False

    def __chooseLogFile(self):
        fileName = QFileDialog.getOpenFileName(self, 'Choose Log File')
        if fileName != None:
            self.logFilePathEdit.setText(fileName[0])

class SerialConnectionEditTab(QWidget):

    portList = {}

    def __init__(self, parent):
        super().__init__(parent)
        self.connectToMAVLink = parent.connectToMAVLink
        self.listSerialPorts()
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

    def doConnect(self):
        port = self.portsDropDown.currentData()
        baud = self.baudDropDown.currentData()
        # print('{} -- {}'.format(port, baud))
        mavutil.set_dialect('autoquad')  # test
        connection = mavutil.mavlink_connection(port, int(baud))
        self.connectToMAVLink.emit(connection)
        return True

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
    statusTextHandler = pyqtSignal(object)

    connectionEstablishedSignal = pyqtSignal()
    onboardWaypointsReceivedSignal = pyqtSignal(object)  # pass the list of waypoints as parameter

    handlerLookup = {}
    internalHandlerLookup = {}
    mavStatus = {MavStsKeys.AP_SYS_ID : 1}
    isConnected = False
    paramList = []
    paramPanel = None

    txLock = QMutex()  # uplink lock
    txResponseCond = QWaitCondition()
    txTimeoutTimer = QTimer()
    txTimeoutmsec = 200000000  # 2 seconds
    finalWPSent = False

    wpLoader = MAVWPLoader()
    onboardWPCount = 0
    numberOfonboardWP = 0
    onboardWP = []

    def __init__(self, connection):
        super().__init__()
        self.param = UserData.getInstance().getUserDataEntry('TELEMETRY')
        if self.param == None:
            self.param = {}
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
        self.handlerLookup['STATUSTEXT'] = self.statusTextHandler

        self.internalHandlerLookup['PARAM_VALUE'] = self.receiveOnboardParameter
        self.internalHandlerLookup['MISSION_REQUEST'] = self.receiveMissionRequest
        self.internalHandlerLookup['MISSION_ACK'] = self.receiveMissionAcknowledge
        self.internalHandlerLookup['MISSION_COUNT'] = self.receiveMissionItemCount
        self.internalHandlerLookup['MISSION_ITEM'] = self.receiveMissionItem

        self.txTimeoutTimer.timeout.connect(self._timerTimeout)
        self.txTimeoutTimer.setSingleShot(True)

    def requestExit(self):
        # print('exit conn thread...')
        self.running = False

    def run(self):
        # print('waiting for heart beat...')
        self._establishConnection()
        while self.running:
            msg = self.connection.recv_match(blocking=False)
            if msg != None:
                msgType = msg.get_type()
                if msgType in self.internalHandlerLookup:
                    self.internalHandlerLookup[msgType](msg)
                else:
                    self._msgDispatcher(msg)
        self.connection.close()
        if self.connection.logfile_raw != None:
            self.connection.logfile_raw.close()
        # print('connection closed')

    def _msgDispatcher(self, msg):
        msgType = msg.get_type()
        if msgType in self.handlerLookup:
            self.handlerLookup[msgType].emit(msg)
        else:
            print('UNKNOWN MSG:', msg)

    def _establishConnection(self):
        hb = self.connection.wait_heartbeat()
        self.__createLogFile()
        self.mavStatus[MavStsKeys.VEHICLE_TYPE] = hb.type
        self.mavStatus[MavStsKeys.AP_TYPE] = hb.autopilot
        self.mavStatus[MavStsKeys.AP_MODE] = hb.base_mode
        self.mavStatus[MavStsKeys.CUSTOM_AP_MODE] = hb.custom_mode
        self.mavStatus[MavStsKeys.AP_SYS_STS] = hb.system_status
        self.mavStatus[MavStsKeys.MAVLINK_VER] = hb.mavlink_version
        # request all parameters
        self.connection.param_fetch_all()

    def receiveOnboardParameter(self, msg):
        self.paramList.append(msg)
        if msg.param_index + 1 == msg.param_count:
            self.isConnected = True
            if self.param['DOWNLOAD_WAYPOINTS_ON_CONNECT']:
                self.downloadWaypoints()  # request to read all onboard waypoints
            self.connectionEstablishedSignal.emit()

    def receiveMissionItem(self, msg):
        self.numberOfonboardWP += 1
        wp = Waypoint(msg.seq, msg.x, msg.y, msg.z)
        wp.waypointType = msg.command
        self.onboardWP.append(wp)
        if self.numberOfonboardWP < self.onboardWPCount:
            self.connection.waypoint_request_send(self.numberOfonboardWP)  # read next one
        else:
            print('Total {} waypoint(s) onboard'.format(len(self.onboardWP)))
            self.onboardWaypointsReceivedSignal.emit(self.onboardWP)  # all done, send signal

    def receiveMissionItemCount(self, msg):
        self.onboardWPCount = msg.count
        if self.onboardWPCount > 0:
            self.connection.waypoint_request_send(0)  # start reading onboard waypoints

    def receiveMissionRequest(self, msg):
        # print('missionRequest:', msg)
        self._sendOneWaypoint(self.wpLoader.wp(msg.seq))

    def receiveMissionAcknowledge(self, msg):
        print('missionRequestAck:', msg)
        self.txResponseCond.wakeAll()

    def showParameterEditWindow(self):
        if self.isConnected:
            if self.paramPanel == None:
                self.paramPanel = ParameterPanel(self.paramList)
                self.paramPanel.uploadNewParametersSignal.connect(self.uploadNewParametersEvent)
            self.paramPanel.show()

    def downloadWaypoints(self):
        self.connection.waypoint_request_list_send()

    def uploadWaypoints(self, wpList):
        seq = 0
        for wp in wpList:
            item = wp.toMavlinkMessage(self.connection.target_system, self.connection.target_component, seq, 0, 1)
            seq += 1
            self.wpLoader.add(item)
        print('all wp queued!')
        self._sendMissionCount(len(wpList))

    def setHomePosition(self, wp):
        item = mavutil.mavlink.MAVLink_mission_item_message(self.connection.target_system, self.connection.target_component, 0,
                                                            mavlink.MAV_FRAME_GLOBAL, mavlink.MAV_CMD_DO_SET_HOME , 1, 0,
                                                            1, None, None, None,
                                                            wp.latitude, wp.longitude, wp.altitude)
        self._sendOneWaypoint(item)

    def _sendMissionCount(self, cnt):
        print('{} waypoints to be sent'.format(cnt))
        # self.txTimeoutTimer.start(self.txTimeoutmsec)
        self.txLock.lock()
        # self.connection.waypoint_clear_all_send()
        self.connection.waypoint_count_send(cnt)
        print('[CNT] wait for response...')
        self.txResponseCond.wait(self.txLock)
        self.txLock.unlock()
        print('[CNT] Got response!')

    def _sendOneWaypoint(self, wp):
        print('sending wp: {}'.format(wp))
        # self.txTimeoutTimer.start(self.txTimeoutmsec)
        self.connection.mav.send(wp)

    def _timerTimeout(self):
        print('Timeout')
        self.txResponseCond.wakeAll()

    def navigateToWaypoint(self, wp: Waypoint):
        item = mavutil.mavlink.MAVLink_mission_item_message(self.connection.target_system, self.connection.target_component, 0,
                                                            mavlink.MAV_FRAME_GLOBAL, mavlink.MAV_CMD_NAV_WAYPOINT, 1,
                                                            1,  # Auto continue to next waypoint
                                                            0, 0, 0, 0, wp.latitude, wp.longitude, wp.altitude)
        self._sendOneWaypoint(item)

    def initializeReturnToHome(self, home: Waypoint):
        print('RTH started:', home)
        # TODO compare the home location in UAV with it in GCS
        item = mavutil.mavlink.MAVLink_mission_item_message(self.connection.target_system, self.connection.target_component, 0,
                                                            mavlink.MAV_FRAME_GLOBAL, mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 1, 0,
                                                            None, None, None, None, None, None, None)
        self._sendOneWaypoint(item)

    def uploadNewParametersEvent(self, params):
        print('sending parameters:', params)

    def __createLogFile(self):
        name = 'MAV_{}.bin'.format(int(time() * 1000))
        self.connection.setup_logfile_raw(os.path.join(self.param['LOG_FOLDER'], name), 'wb')  # a patch is needed in mavutil.py to support binary mode
