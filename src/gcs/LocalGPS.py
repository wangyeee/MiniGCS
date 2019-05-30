from pynmea2 import NMEAStreamReader
from pynmea2.types.talker import GGA
from PyQt5.QtCore import pyqtSignal, QThread
from PyQt5.QtPositioning import QGeoCoordinate
from serial import Serial
from telemetry import ConnectionEditWindow, SerialConnectionEditTab

class NMEAReceiver(QThread):

    locationUpdate = pyqtSignal(object)

    def __init__(self, port = None, baud = 9600):
        super().__init__()
        self.serialPort = Serial()
        self.running = False
        self.configureSerialPort(port, baud)

    def configureSerialPort(self, port, baud):
        self.serialPort.baudrate = baud
        self.serialPort.port = port

    def connect(self):
        self.serialPort.open()
        if self.serialPort.isOpen():
            self.running = True
            self.start()

    def disconnect(self):
        self.running = False

    def run(self):
        streamreader = NMEAStreamReader()
        while self.running:
            data = self.serialPort.readline().decode('ASCII')
            if data.startswith('$GP'):
                for msg in streamreader.next(data):
                    # print('{}  {}  {}'.format(msg_cnt, type(msg), msg))
                    if type(msg) is GGA:
                        # print('location update: {}, {}'.format(msg.latitude, msg.longitude))
                        self.locationUpdate.emit(QGeoCoordinate(msg.latitude, msg.longitude))
        if self.serialPort.isOpen():
            self.serialPort.close()

class GPSConfigurationWindow(ConnectionEditWindow):

    def __init__(self, parent = None):
        super().__init__(parent)
        self.connection = NMEAReceiver()
        self.setWindowTitle('GPS Configuration')

    def _createTabs(self):
        self.serialConnTab = SerialConnectionEditTab(initParams={}, parent=self)
        self.tabs.addTab(self.serialConnTab, 'Serial GPS Receiver')

    def _doConnect(self):
        currTab = self.tabs.currentWidget()
        port = currTab.portsDropDown.currentData()
        baud = currTab.baudDropDown.currentData()
        self.connection.configureSerialPort(port, int(baud))
        self.connection.connect()
        self.close()
