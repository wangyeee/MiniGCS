from PyQt5.QtWidgets import QApplication, QHBoxLayout, QWidget, QComboBox, QLabel, QGridLayout, QPushButton, QSizePolicy
from PyQt5.QtCore import Qt, QVariant, QObject, pyqtSignal, QThread
# from pymavlink.dialects.v10 import common as mavlink
from pymavlink import mavutil
from serial import Serial
from serial.tools.list_ports import comports

import sys, time

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

class ConnectionEditWindow(QWidget):

    portList = {}

    connectToMAVLink = pyqtSignal(object, object)
    cancelConnectionSignal = pyqtSignal()  # test only

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
        print(self.portList)

    def cancelConnection(self):
        self.cancelConnectionSignal.emit()
        self.close()

    def connectSerialPort(self):
        port = self.portsDropDown.currentData()
        baud = self.baudDropDown.currentData()
        # print('{} -- {}'.format(port, baud))
        connection = mavutil.mavlink_connection(port, int(baud))
        self.connectToMAVLink.emit(self, connection)

class MAVLinkConnection(QThread):

    handlerLookup = {}

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

    def gpsRawIntHandler(self, msg):
        print('GPS_RAW_INT:', msg)

    def localPositionNEDHandler(self, msg):
        print('LOCAL_POSITION_NED:', msg)

    def navControllerOutputHandler(self, msg):
        print('NAV_CONTROLLER_OUTPUT:', msg)

    def radioStatusHandler(self, msg):
        print('RADIO_STATUS:', msg)

    def rcChannelRawHandler(self, msg):
        print('RC_CHANNELS_RAW:', msg)

    def scaledIMUHandler(self, msg):
        print('SCALED_IMU:', msg)

    def scaledPressureHandler(self, msg):
        print('SCALED_PRESSURE:', msg)

    def heartBeatHandler(self, msg):
        mode = mavutil.mode_string_v10(msg)
        isArmed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        isEnabled = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED
        print('HB event: mode = {}, armed = {}, enabled = {}'.format(mode, isArmed, isEnabled))

    def altitudeHandler(self, msg):
        # print('roll = {}, pitch = {}, yaw = {}, rollspeed = {}, pitchspeed = {}, yawspeed = {}'.format(msg.roll, msg.pitch, msg.yaw, msg.rollspeed, msg.pitchspeed, msg.yawspeed))
        print('ALTITUDE:', msg)

    def systemStatusHandler(self, msg):
        print('SYS_STATUS:', msg)

    def requestExit(self):
        print('exit conn thread...')
        self.running = False

    def run(self):
        print('waiting for heart beat...')
        self.connection.wait_heartbeat()

        while self.running:
            msg = self.connection.recv_match(blocking=False)
            if msg != None:
                msgType = msg.get_type()
                if msgType in self.handlerLookup:
                    self.handlerLookup[msgType](msg)

class TestMav(QObject):

    def createConnection(self, edit: ConnectionEditWindow, conn):
        self.mav = MAVLinkConnection(conn)
        edit.cancelConnectionSignal.connect(self.mav.requestExit)
        self.mav.start()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    tele = ConnectionEditWindow()
    test = TestMav()
    tele.connectToMAVLink.connect(test.createConnection)
    tele.show()
    sys.exit(app.exec_())
