from pynmea2 import NMEAStreamReader
from pynmea2.types.talker import GGA
from PyQt5.QtCore import pyqtSignal, QThread
from PyQt5.QtPositioning import QGeoCoordinate
from serial import Serial
from telemetry import ConnectionEditWindow

from PyQt5.QtWidgets import QApplication
import sys

class NMEAReceiver(QThread):

    locationUpdate = pyqtSignal(object)

    serialPort = Serial()
    running = False

    def __init__(self, port = None, baud = 9600):
        super().__init__()
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
                        print('location update: {}, {}'.format(msg.latitude, msg.longitude))
                        self.locationUpdate.emit(QGeoCoordinate(msg.latitude, msg.longitude))
        if self.serialPort.isOpen():
            self.serialPort.close()

class GPSConfigurationWindow(ConnectionEditWindow):

    connection = NMEAReceiver()

    def __init__(self, parent = None):
        super().__init__(parent)
        self.setWindowTitle('GPS Configuration')

    def connectSerialPort(self):
        print('connect to gps...')
        port = self.portsDropDown.currentData()
        baud = self.baudDropDown.currentData()
        print('{} -- {}'.format(port, baud))
        self.connection.configureSerialPort(port, int(baud))
        self.connection.connect()
        # self.close()

    # test only
    def cancelConnection(self):
        self.connection.disconnect()
        self.close()

# test only
if __name__ == "__main__":
    app = QApplication(sys.argv)
    receiver = GPSConfigurationWindow()
    receiver.show()
    sys.exit(app.exec_())
