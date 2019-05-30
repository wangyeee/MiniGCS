import os, struct
from secrets import token_bytes
from enum import Enum
from time import time, sleep
from collections import deque
from pymavlink import mavutil
from pymavlink.mavutil import mavlogfile
from pymavlink.mavwp import MAVWPLoader
from pymavlink.dialects.v10 import common as mavlink
from PyQt5.QtCore import (QMutex, Qt, QThread, QTimer, QVariant, QObject,
                          QWaitCondition, pyqtSignal)
from PyQt5.QtWidgets import (QComboBox, QGridLayout, QLabel, QPushButton, QLineEdit, QFileDialog,
                             QSizePolicy, QWidget, QTabWidget, QVBoxLayout, QHBoxLayout, QMessageBox, QProgressBar)
from PyQt5.QtGui import QFontMetrics
from serial.tools.list_ports import comports

from parameters import ParameterPanel
from waypoint import Waypoint
from UserData import UserData
from uas import UASInterfaceFactory

BAUD_RATES = {
    0 : 'AUTO',
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

MAVLINK_DIALECTS = {
    mavlink.MAV_AUTOPILOT_GENERIC : 'standard',
    # mavlink.MAV_AUTOPILOT_RESERVED : '',
    mavlink.MAV_AUTOPILOT_SLUGS : 'slugs',
    mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA : 'ardupilotmega',
    mavlink.MAV_AUTOPILOT_OPENPILOT : 'standard',
    mavlink.MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY : 'minimal',
    mavlink.MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY : 'minimal',
    mavlink.MAV_AUTOPILOT_GENERIC_MISSION_FULL : 'standard',
    # mavlink.MAV_AUTOPILOT_INVALID : '',
    mavlink.MAV_AUTOPILOT_PPZ : 'paparazzi',
    mavlink.MAV_AUTOPILOT_UDB : 'standard',
    mavlink.MAV_AUTOPILOT_FP : 'standard',
    mavlink.MAV_AUTOPILOT_PX4 : 'standard',
    mavlink.MAV_AUTOPILOT_SMACCMPILOT : 'standard',
    mavlink.MAV_AUTOPILOT_AUTOQUAD : 'autoquad',
    mavlink.MAV_AUTOPILOT_ARMAZILA : 'standard',
    mavlink.MAV_AUTOPILOT_AEROB : 'standard',
    mavlink.MAV_AUTOPILOT_ASLUAV : 'ASLUAV',
    mavlink.MAV_AUTOPILOT_SMARTAP : 'standard',
    mavlink.MAV_AUTOPILOT_AIRRAILS : 'standard'
}

UD_TELEMETRY_KEY = 'TELEMETRY'
UD_TELEMETRY_LOG_FOLDER_KEY = 'LOG_FOLDER'
UD_TELEMETRY_TIMEOUT_THRESHOLD_KEY = 'TIMEOUT_THRESHOLD'
UD_TELEMETRY_HEARTBEAT_TIMEOUT_KEY = 'HB_TIMEOUT'
UD_TELEMETRY_LAST_CONNECTION_KEY = 'LAST_CONN'
UD_TELEMETRY_LAST_CONNECTION_PORT_KEY = 'PORT'
UD_TELEMETRY_LAST_CONNECTION_BAUD_RATE_KEY = 'BAUD_RATE'

DEFAULT_RC_AUTO_SCALE_SAMPLES = 10
MAVLINKV2_MESSAGE_SIGNING_KEY_LEN = 32 # bytes

class MavStsKeys(Enum):
    AP_SYS_ID = 0
    VEHICLE_TYPE = 1
    AP_TYPE = 2
    AP_MODE = 3
    CUSTOM_AP_MODE = 4
    AP_SYS_STS = 5
    MAVLINK_VER = 6

class MessageSigningSetupWindow(QWidget):

    __mavlinkVersionUpdated = pyqtSignal()
    setMessageSigningKeySignal = pyqtSignal(object, object) # key(hex str), initial timestamp (str of 64 bit integer)

    def __init__(self, mavlinkVersion = -1.0, parent = None):
        super().__init__(parent)
        self.setWindowTitle('Message Signing')
        self.__mavlinkVersion = mavlinkVersion
        self.setLayout(QGridLayout())
        self.__initUI()
        self.__mavlinkVersionUpdated.connect(self.__initUI)

    def setMAVLinkVersion(self, mavlinkVersion):
        print('Set MAVLink version to:', mavlinkVersion)
        self.__mavlinkVersion = float(mavlinkVersion)
        self.__mavlinkVersionUpdated.emit()

    def __initUI(self):
        l = self.layout()
        self.cancelButton = QPushButton('Close')
        self.cancelButton.clicked.connect(self.close)
        row = 0
        if self.__mavlinkVersion == 1.0:
            self.__errorMessage('Message signing is not available in MAVLink v1')
        elif self.__mavlinkVersion == 2.0:
            self.__errorMessage('Setup Message Signing')
            row += 1
            l.addWidget(QLabel('Secret Key'), row, 0, 1, 1)
            self.msgSignSecretField = QLineEdit()
            l.addWidget(self.msgSignSecretField, row, 1, 1, 1)
            self.generateButton = QPushButton('Random')
            self.generateButton.clicked.connect(self.__generateRandomSigningKey)
            l.addWidget(self.generateButton, row, 2, 1, 1)
            row += 1
            l.addWidget(QLabel('Initial Timestamp'), row, 0, 1, 1)
            self.msgSignTimeField = QLineEdit()
            l.addWidget(self.msgSignTimeField, row, 1, 1, 1)
            self.nowButton = QPushButton('Now')
            self.nowButton.clicked.connect(self.__getCurrentMavlinkV2Time)
            l.addWidget(self.nowButton, row, 2, 1, 1)
            row += 1
            self.okayButton = QPushButton('OK')
            self.cancelButton.setText('Cancel')
            self.okayButton.clicked.connect(self.__processMsgSigningSetup)
            l.addWidget(self.okayButton, row, 0, 1, 1, Qt.AlignRight)
            l.addWidget(self.cancelButton, row, 1, 1, 1, Qt.AlignRight)
            ft = self.msgSignSecretField.font()
            if ft != None:
                metrics = QFontMetrics(ft)
                # metrics.height() ~ metrics.width() x 2
                w = metrics.height() * MAVLINKV2_MESSAGE_SIGNING_KEY_LEN
                self.msgSignSecretField.setFixedWidth(w)
                self.msgSignTimeField.setFixedWidth(w)
        elif self.__mavlinkVersion == -1.0:
            self.__errorMessage('Connect to MAVLink first')
        else:
            self.__errorMessage('Unknown MAVLink version: {}'.format(self.__mavlinkVersion))
        self.setLayout(l)

    def __errorMessage(self, msg):
        msgLabel = self.layout().itemAt(0)
        if msgLabel == None:
            self.layout().addWidget(QLabel(msg), 0, 0, 1, 1)
        else:
            msgLabel.widget().setText(msg)

    def __generateRandomSigningKey(self):
        key = token_bytes(MAVLINKV2_MESSAGE_SIGNING_KEY_LEN).hex()
        self.msgSignSecretField.setText(key)

    def __getCurrentMavlinkV2Time(self):
        # units of 10 microseconds since 01-JAN-2015 GMT
        # https://mavlink.io/en/guide/message_signing.html#timestamp
        tm = int((time() - 1420070400) * 100 * 1000)
        self.msgSignTimeField.setText(str(tm))

    def __processMsgSigningSetup(self):
        self.setMessageSigningKeySignal.emit(self.msgSignSecretField.text(), self.msgSignTimeField.text())

class RadioControlTelemetryWindow(QWidget):

    def __init__(self, parent = None):
        super().__init__(parent)
        self.isAnyRCChannelsUpdate = False
        self.__defaultWidget = None
        self.setWindowTitle('Radio Control Telemetry')
        self.__createDefaultWidget()
        self.tabs = QTabWidget()
        self.tabs.addTab(self.__defaultWidget, 'RC Telemetry')
        self.ports = {}
        l = QVBoxLayout()
        l.addWidget(self.tabs)
        self.setLayout(l)

    def updateRCChannelValues(self, msg):
        if msg.port not in self.ports:
            if self.isAnyRCChannelsUpdate == False:
                self.isAnyRCChannelsUpdate = True
                self.tabs.removeTab(0)
            self.ports[msg.port] = RadioControlTelemetryPanel()
            self.tabs.addTab(self.ports[msg.port], 'Receiver {}'.format(msg.port))

        channels = []
        channels.append(msg.chan1_raw)
        channels.append(msg.chan2_raw)
        channels.append(msg.chan3_raw)
        channels.append(msg.chan4_raw)
        channels.append(msg.chan5_raw)
        channels.append(msg.chan6_raw)
        channels.append(msg.chan7_raw)
        channels.append(msg.chan8_raw)
        self.ports[msg.port].updateValues(channels)

    def __createDefaultWidget(self):
        self.__defaultWidget = QWidget()
        l = QVBoxLayout()
        l.addWidget(QLabel('No RC channel value message has been received.'))
        self.__defaultWidget.setLayout(l)

class RadioControlTelemetryPanel(QWidget):
    def __init__(self, parent = None):
        super().__init__(parent)
        l = QGridLayout()
        self.__autoScaleSamples = DEFAULT_RC_AUTO_SCALE_SAMPLES
        self.channelValueRanges = [] # (min, max, samples)
        self.channelValueBars = []
        self.channelValueLabels = []
        for i in range(8):
            self.channelValueRanges.append((1000000, 0, 0))
            self.channelValueBars.append(QProgressBar(self))
            self.channelValueLabels.append(QLabel('0 ms', self))
            self.channelValueBars[i].setRange(1000, 2000)
            self.channelValueBars[i].setTextVisible(False)
            l.addWidget(QLabel('Channel {}'.format(i + 1)), i, 0, 1, 1)
            l.addWidget(self.channelValueBars[i], i, 1, 1, 1)
            l.addWidget(self.channelValueLabels[i], i, 2, 1, 1)
        l.setColumnStretch(1, 1)
        self.setLayout(l)

    def updateValues(self, values):
        for i in range(8):
            if values[i] < self.channelValueRanges[i][0]:
                self.channelValueRanges[i] = (values[i], self.channelValueRanges[i][1], self.channelValueRanges[i][2])
            if values[i] > self.channelValueRanges[i][1]:
                self.channelValueRanges[i] = (self.channelValueRanges[i][0], values[i], self.channelValueRanges[i][2])
            if self.channelValueRanges[i][1] > self.channelValueRanges[i][0]:
                if self.channelValueRanges[i][2] < self.__autoScaleSamples:
                    # First `self.__autoScaleSamples` samples will always be used to update scale
                    self.channelValueBars[i].setRange(self.channelValueRanges[i][0], self.channelValueRanges[i][1])
                    self.channelValueRanges[i] = (self.channelValueRanges[i][0], self.channelValueRanges[i][1], self.channelValueRanges[i][2] + 1)
                else:
                    # After that, only values exceeding current ranges will be updated
                    if self.channelValueRanges[i][0] < self.channelValueBars[i].minimum():
                        self.channelValueBars[i].setMinimum(self.channelValueRanges[i][0])
                    if self.channelValueRanges[i][1] > self.channelValueBars[i].maximum():
                        self.channelValueBars[i].setMaximum(self.channelValueRanges[i][1])
            self.channelValueBars[i].setValue(values[i])
            self.channelValueLabels[i].setText('{} ms'.format(values[i]))

class ConnectionEditWindow(QWidget):

    MAVLinkConnectedSignal = pyqtSignal(object)
    cancelConnectionSignal = pyqtSignal()

    def __init__(self, parent = None):
        super().__init__(parent)
        self.tabs = QTabWidget(self)
        self._createTabs()
        l = QVBoxLayout()
        l.setContentsMargins(0, 0, 0, 0)
        l.addWidget(self.tabs)
        l.addWidget(self.__createActionButtons())
        self.setLayout(l)

    def _createTabs(self):
        self.serialConnTab = SerialConnectionEditTab(parent=self)
        self.logReplayTab = LogFileReplayEditTab(self)
        self.tabs.addTab(self.serialConnTab, 'Serial Link')
        self.tabs.addTab(self.logReplayTab, 'Log File Replay')

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

    def closeEvent(self, event):
        self.cancelConnectionSignal.emit()
        super().closeEvent(event)

    def _doConnect(self):
        currTab = self.tabs.currentWidget()
        if hasattr(currTab, 'doConnect'):
            if currTab.doConnect():
                self.close()

class LogFileReplaySpeedControl(mavlogfile, QObject):

    replayCompleteSignal = pyqtSignal()


    def __init__(self, filename):
        mavlogfile.__init__(self, filename)
        QObject.__init__(self)
        self.replaySpeed = 1.0

    def pre_message(self):
        super().pre_message()
        if self._last_timestamp is not None and self.replaySpeed > 0:
            ts = abs(self._timestamp - self._last_timestamp) * self.replaySpeed
            sleep(ts)

    def recv(self,n=None):
        b = super().recv(n)
        if b == None or len(b) < n:
            self.replayCompleteSignal.emit()
        return b

    def write(self, buf):
        '''Log files will be open in read only mode. All write operations are ignored.'''
        pass

class LogFileReplayEditTab(QWidget):

    def __init__(self, parent):
        super().__init__(parent)
        self.MAVLinkConnectedSignal = parent.MAVLinkConnectedSignal
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
            connection = LogFileReplaySpeedControl(fileName)
            self.MAVLinkConnectedSignal.emit(connection)
            return True
        QMessageBox.critical(self.window(), 'Error', 'Invalid log file: {}'.format(fileName), QMessageBox.Ok)
        return False

    def __chooseLogFile(self):
        fileName = QFileDialog.getOpenFileName(self, 'Choose Log File')
        if fileName != None:
            self.logFilePathEdit.setText(fileName[0])

class SerialConnectionEditTab(QWidget):

    __autoBaudStartSignal = pyqtSignal(object)

    def __init__(self, initParams = None, parent = None):
        super().__init__(parent)
        self.portList = {}
        self.autoBaud = None
        self.MAVLinkConnectedSignal = parent.MAVLinkConnectedSignal
        self.MAVLinkConnectedSignal.connect(self.__recordLastConnection)
        self.__autoBaudStartSignal.connect(self.__autoBaud)
        self.listSerialPorts()
        if initParams == None:
            self.params = self.__getLastConnectionParameter()
        else:
            self.params = initParams
        l = QGridLayout()
        row = 0

        lbl, self.portsDropDown = self._createDropDown(
            'Serial Port', self.portList,
            UserData.getParameterValue(self.params, UD_TELEMETRY_LAST_CONNECTION_PORT_KEY))
        l.addWidget(lbl, row, 0, 1, 1, Qt.AlignRight)
        l.addWidget(self.portsDropDown, row, 1, 1, 3, Qt.AlignLeft)
        self.refreshButton = QPushButton('\u21BB')  # Unicode for clockwise open circle arrow
        self.refreshButton.setFixedSize(self.portsDropDown.height(), self.portsDropDown.height())
        l.addWidget(self.refreshButton, row, 4, 1, 1, Qt.AlignLeft)
        self.refreshButton.clicked.connect(lambda: self.listSerialPorts(self.portsDropDown))
        row += 1

        lbl, self.baudDropDown = self._createDropDown(
            'Baud Rate', BAUD_RATES,
            UserData.getParameterValue(self.params, UD_TELEMETRY_LAST_CONNECTION_BAUD_RATE_KEY))
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
        self.autoBaudMessageLabel = QLabel('')
        l.addWidget(self.autoBaudMessageLabel, row, 0, 1, 3)
        row += 1

        self.setLayout(l)

    def _createDropDown(self, label, data: dict, defaultValue = None):
        dropDown = QComboBox(self)
        dropDown.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        i = 0
        for key, val in data.items():
            dropDown.addItem(str(val), QVariant(key))
            if key == defaultValue:
                dropDown.setCurrentIndex(i)
            i += 1
        return QLabel(label), dropDown

    def listSerialPorts(self, dropDown = None):
        portsInfo = sorted(comports(False))
        cnts = 0
        self.portList.clear()
        for p in portsInfo:
            self.portList[p.device] = p
            cnts += 1
        if cnts == 0:
            self.portList['No ports available'] = 'No ports available'
        if dropDown != None:
            while dropDown.count() > 0:
                dropDown.removeItem(0)
            for key, val in self.portList.items():
                dropDown.addItem(str(val), QVariant(key))

    def doConnect(self):
        port = self.portsDropDown.currentData()
        baud = self.baudDropDown.currentData()
        if baud == 0:
            self.__autoBaudStartSignal.emit(port)
            return False  # Keep window open while auto bauding
        connection = mavutil.mavlink_connection(port, int(baud))
        self.MAVLinkConnectedSignal.emit(connection)
        return True

    def __autoBaud(self, port):
        self.autoBaud = AutoBaudThread(port, self)
        #                                   This Tab       QTabWidget     QWidget
        self.autoBaud.finished.connect(self.parentWidget().parentWidget().parentWidget().close)
        self.autoBaud.autoBaudStatusUpdateSignal.connect(self.autoBaudMessageLabel.setText)
        self.autoBaud.start()

    def __recordLastConnection(self, conn):
        self.params[UD_TELEMETRY_LAST_CONNECTION_PORT_KEY] = conn.device
        self.params[UD_TELEMETRY_LAST_CONNECTION_BAUD_RATE_KEY] = conn.baud

    def __getLastConnectionParameter(self):
        pParam = UserData.getInstance().getUserDataEntry(UD_TELEMETRY_KEY, {})
        return UserData.getParameterValue(pParam, UD_TELEMETRY_LAST_CONNECTION_KEY, {})

class AutoBaudThread(QThread):

    autoBaudStatusUpdateSignal = pyqtSignal(object)

    def __init__(self, port, parent):
        super().__init__(parent)
        self.MAVLinkConnectedSignal = parent.MAVLinkConnectedSignal
        self.port = port

    def run(self):
        for b in BAUD_RATES:
            if b >= self.__minimumBaudRate():
                self.autoBaudStatusUpdateSignal.emit('AutoBaud: try baud rate {}'.format(b))
                conn = mavutil.mavlink_connection(self.port, b)
                hb = conn.wait_heartbeat(timeout=2.0)  # set timeout to 2 second
                if hb == None:
                    self.autoBaudStatusUpdateSignal.emit('AutoBaud: timeout for baud rate {}'.format(b))
                    # Reset environment variables after a failed attempt
                    # Otherwise mavutil.auto_mavlink_version may result in
                    # unexpected behaviour
                    if 'MAVLINK09' in os.environ:
                        del os.environ['MAVLINK09']
                    if 'MAVLINK20' in os.environ:
                        del os.environ['MAVLINK20']
                    conn.close()
                else:
                    self.autoBaudStatusUpdateSignal.emit('AutoBaud: correct baud rate is {}'.format(b))
                    self.MAVLinkConnectedSignal.emit(conn)
                    return
        # Fail back to default mavlink baud rate
        self.autoBaudStatusUpdateSignal.emit('AutoBaud: default 57600')
        self.MAVLinkConnectedSignal.emit(mavutil.mavlink_connection(self.port, 57600))

    def __minimumBaudRate(self):
        return 4800

class MAVLinkConnection(QThread):

    externalMessageHandler = pyqtSignal(object)  # pass any types of message to an external handler

    connectionEstablishedSignal = pyqtSignal()
    onboardWaypointsReceivedSignal = pyqtSignal(object)  # pass the list of waypoints as parameter
    newTextMessageSignal = pyqtSignal(object)
    messageTimeoutSignal = pyqtSignal(float)  # pass number of seconds without receiving any messages
    heartbeatTimeoutSignal = pyqtSignal()

    DEFAULT_MESSAGE_TIMEOUT_THRESHOLD = 2.0
    DEFAULT_HEARTBEAT_TIMEOUT= 5.0

    def __init__(self, connection, replayMode = False, enableLog = True):
        super().__init__()
        self.internalHandlerLookup = {}
        self.mavStatus = {MavStsKeys.AP_SYS_ID : 1}
        self.isConnected = False
        self.paramList = []
        self.paramPanel = None

        self.txLock = QMutex()  # uplink lock
        self.txResponseCond = QWaitCondition()
        self.txTimeoutTimer = QTimer()
        self.finalWPSent = False

        self.wpLoader = MAVWPLoader()
        self.onboardWPCount = 0
        self.numberOfonboardWP = 0
        self.onboardWP = []

        self.mavlinkLogFile = None
        self.lastMessageReceivedTimestamp = 0.0
        self.lastMessages = {} # type = (msg, timestamp)

        self.param = UserData.getInstance().getUserDataEntry(UD_TELEMETRY_KEY, {})
        self.messageTimeoutThreshold = UserData.getParameterValue(self.param,
                                                                  UD_TELEMETRY_TIMEOUT_THRESHOLD_KEY,
                                                                  MAVLinkConnection.DEFAULT_MESSAGE_TIMEOUT_THRESHOLD)
        self.txTimeoutmsec = self.messageTimeoutThreshold * 1000000
        # timeout for wait initial heartbeat signal
        self.initHeartbeatTimeout = UserData.getParameterValue(self.param,
                                                               UD_TELEMETRY_HEARTBEAT_TIMEOUT_KEY,
                                                               MAVLinkConnection.DEFAULT_HEARTBEAT_TIMEOUT)
        self.txMessageQueue = deque()
        self.running = True
        self.connection = connection
        self.replayMode = replayMode
        self.enableLog = enableLog
        self.uas = None
        if replayMode:
            self.enableLog = False
            connection.replayCompleteSignal.connect(self.requestExit)
        self.internalHandlerLookup['PARAM_VALUE'] = self.receiveOnboardParameter
        self.internalHandlerLookup['MISSION_REQUEST'] = self.receiveMissionRequest
        self.internalHandlerLookup['MISSION_ACK'] = self.receiveMissionAcknowledge
        self.internalHandlerLookup['MISSION_COUNT'] = self.receiveMissionItemCount
        self.internalHandlerLookup['MISSION_ITEM'] = self.receiveMissionItem
        self.internalHandlerLookup['DATA_STREAM'] = self.receiveDataStream
        self.internalHandlerLookup['PARAM_SET'] = self.receiveParameterSet

        self.txTimeoutTimer.timeout.connect(self._timerTimeout)
        self.txTimeoutTimer.setSingleShot(True)
        # print('waiting for heart beat...')
        # self._establishConnection()

    def requestExit(self):
        # print('exit conn thread...')
        self.running = False

    def run(self):
        while self.running:
            msg = self.connection.recv_match(blocking=False)
            if msg != None:
                msgType = msg.get_type()
                if msgType != 'BAD_DATA':
                    # exclude BAD_DATA from any other messages
                    self.lastMessageReceivedTimestamp = time()
                    self.lastMessages[msgType] = (msg, self.lastMessageReceivedTimestamp)
                    if self.enableLog:
                        ts = int(time() * 1.0e6) & ~3
                        self.mavlinkLogFile.write(struct.pack('>Q', ts) + msg.get_msgbuf())
                    # 1. send message to external destination
                    self.externalMessageHandler.emit(msg)
                    # 2. process message with internal UASInterface
                    self.uas.receiveMAVLinkMessage(msg)
                    # 3. process message with other internal handlers
                    if msgType in self.internalHandlerLookup:
                        self.internalHandlerLookup[msgType](msg)
                else:
                    # TODO handle BAD_DATA?
                    print('BAD_DATA:', msg)
            rs = time() - self.lastMessageReceivedTimestamp
            if (rs > self.messageTimeoutThreshold):
                print('Message timeout:', rs)
                self.messageTimeoutSignal.emit(rs)
            try:
                txMsg = self.txMessageQueue.popleft()
                print('sending mavlink msg:', txMsg)
                self.connection.mav.send(txMsg)
            except IndexError:
                pass
        self.__doDisconnect()

    def __doDisconnect(self, txtmsg = 'Disconnected'):
        self.connection.close()
        if self.enableLog and self.mavlinkLogFile != None:
            self.mavlinkLogFile.close()
        self.newTextMessageSignal.emit(txtmsg)

    def establishConnection(self):
        hb = self.connection.wait_heartbeat(timeout=self.initHeartbeatTimeout)
        if hb == None:
            self.running = False
            self.__doDisconnect('Connection timeout')
            self.heartbeatTimeoutSignal.emit()
            return
        self.lastMessageReceivedTimestamp = time()
        self.__createLogFile()
        self.__setMavlinkDialect(hb.autopilot)
        self.mavStatus[MavStsKeys.VEHICLE_TYPE] = hb.type
        self.mavStatus[MavStsKeys.AP_TYPE] = hb.autopilot
        self.mavStatus[MavStsKeys.AP_MODE] = hb.base_mode
        self.mavStatus[MavStsKeys.CUSTOM_AP_MODE] = hb.custom_mode
        self.mavStatus[MavStsKeys.AP_SYS_STS] = hb.system_status
        self.mavStatus[MavStsKeys.MAVLINK_VER] = hb.mavlink_version
        # request all parameters
        if self.replayMode:
            self.newTextMessageSignal.emit('Conneced in log file replay mode')
            self.isConnected = True
            self.connectionEstablishedSignal.emit()
        else:
            self.newTextMessageSignal.emit('Conneced to AP:{}'.format(self.mavStatus[MavStsKeys.AP_TYPE]))
            self.connection.param_fetch_all()

    def receiveOnboardParameter(self, msg):
        self.paramList.append(msg)
        self.newTextMessageSignal.emit('Param: {} = {}'.format(msg.param_id, msg.param_value))
        if msg.param_index + 1 == msg.param_count:
            self.isConnected = True
            self.newTextMessageSignal.emit('{} parameters received'.format(msg.param_count))
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
            self.newTextMessageSignal.emit('Total {} waypoint(s) onboard'.format(len(self.onboardWP)))
            self.onboardWaypointsReceivedSignal.emit(self.onboardWP)  # all done, send signal

    def receiveMissionItemCount(self, msg):
        self.onboardWPCount = msg.count
        if self.onboardWPCount > 0:
            self.connection.waypoint_request_send(0)  # start reading onboard waypoints

    def receiveMissionRequest(self, msg):
        # print('missionRequest:', msg)
        self.sendMavlinkMessage(self.wpLoader.wp(msg.seq))

    def receiveMissionAcknowledge(self, msg):
        print('missionRequestAck:', msg)
        self.txResponseCond.wakeAll()

    def receiveDataStream(self, msg):
        # DATA_STREAM {stream_id : 10, message_rate : 0, on_off : 0}
        print(msg)

    def receiveParameterSet(self, msg):
        # PARAM_SET {target_system : 81, target_component : 50, param_id : BFLOW_GYRO_COM, param_value : 0.0, param_type : 9}
        print(msg)

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
        self.sendMavlinkMessage(item)

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

    def sendMavlinkMessage(self, msg):
        ''' Add a mavlink message to the tx queue '''
        if msg.target_system == 255:
            msg.target_system = self.connection.target_system
        if msg.target_component == 255:
            msg.target_component = self.connection.target_component
        self.txMessageQueue.append(msg)

    def _timerTimeout(self):
        print('Timeout')
        self.txResponseCond.wakeAll()

    def navigateToWaypoint(self, wp: Waypoint):
        item = mavutil.mavlink.MAVLink_mission_item_message(self.connection.target_system, self.connection.target_component, 0,
                                                            mavlink.MAV_FRAME_GLOBAL, mavlink.MAV_CMD_NAV_WAYPOINT, 1,
                                                            1,  # Auto continue to next waypoint
                                                            0, 0, 0, 0, wp.latitude, wp.longitude, wp.altitude)
        self.sendMavlinkMessage(item)

    def initializeReturnToHome(self):
        self.connection.set_mode_rtl()

    def uploadNewParametersEvent(self, params):
        # the params from UI are MAVLink_param_value_message,
        # which are required to be consistent with all parameters
        # download upon connection. They will be converted to
        # MAVLink_param_set_message before sending to UAV
        for param in params:
            paramSet = mavutil.mavlink.MAVLink_param_set_message(self.connection.target_system,
                                                                 self.connection.target_component,
                                                                 param.param_id.encode('utf-8'),
                                                                 param.param_value, param.param_type)
            self.sendMavlinkMessage(paramSet)

    def __createLogFile(self):
        if self.enableLog:
            name = 'MAV_{}.bin'.format(int(time() * 1000))
            self.mavlinkLogFile = open(os.path.join(self.param[UD_TELEMETRY_LOG_FOLDER_KEY], name), 'wb')

    def __setMavlinkDialect(self, ap):
        mavutil.mavlink = None  # reset previous dialect
        self.uas = UASInterfaceFactory.getUASInterface(ap)
        self.uas.mavlinkMessageTxSignal.connect(self.sendMavlinkMessage)
        if ap in MAVLINK_DIALECTS:
            print('Set dialect to: {} ({})'.format(MAVLINK_DIALECTS[ap], ap))
            mavutil.set_dialect(MAVLINK_DIALECTS[ap])
        elif ap != mavlink.MAV_AUTOPILOT_INVALID:
            # default to common
            print('Set dialect to common for unknown AP type:', ap)
            mavutil.set_dialect(MAVLINK_DIALECTS[mavlink.MAV_AUTOPILOT_GENERIC])
        # Hot patch after setting mavlink dialect on the fly
        self.connection.mav = mavutil.mavlink.MAVLink(self.connection,
                                                      srcSystem=self.connection.source_system,
                                                      srcComponent=self.connection.source_component)
        self.connection.mav.robust_parsing = self.connection.robust_parsing
        self.connection.WIRE_PROTOCOL_VERSION = mavutil.mavlink.WIRE_PROTOCOL_VERSION
