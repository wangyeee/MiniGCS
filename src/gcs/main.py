import math
import os
import sys

from PyQt5.QtWidgets import (QApplication, QHBoxLayout, QMainWindow,
                             QSizePolicy, QVBoxLayout, QWidget)

from map import MapWidget
from pfd import PrimaryFlightDisplay
from statusPanel import SystemStatusPanel
from telemetry import ConnectionEditWindow, MAVLinkConnection
from LocalGPS import GPSConfigurationWindow

class MiniGCS(QMainWindow):

    mav = None

    def __init__(self, parent = None):
        super().__init__(parent)
        current_path = os.path.abspath(os.path.dirname(__file__))
        qmlFile = os.path.join(current_path, 'map.qml')
        self.setWindowTitle('Mini GCS')
        self.teleWindow = ConnectionEditWindow()
        self.teleWindow.connectToMAVLink.connect(self.createConnection)
        self.window = QWidget()
        mainLayout = QHBoxLayout()
        leftLayout = QVBoxLayout()
        self.left = QWidget()
        self.pfd = PrimaryFlightDisplay(self.window)
        self.sts = SystemStatusPanel(self.window)
        self.sts.connectToMAVLink.connect(self.teleWindow.show)
        self.sts.disconnectFromMAVLink.connect(self.disconnect)
        leftLayout.addWidget(self.pfd)
        leftLayout.addWidget(self.sts)
        self.left.setLayout(leftLayout)
        spLeft = QSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)
        spLeft.setHorizontalStretch(1)
        self.left.setSizePolicy(spLeft)
        self.m = MapWidget(qmlFile)
        spRight = QSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)
        spRight.setHorizontalStretch(2)
        self.m.setSizePolicy(spRight)
        mainLayout.addWidget(self.left)
        mainLayout.addWidget(self.m)
        self.localGPSWindow = GPSConfigurationWindow()
        # TODO configurable behavior
        self.localGPSWindow.connection.locationUpdate.connect(self.m.updateHomeLocationEvent)
        self.sts.connectToLocalGPS.connect(self.localGPSWindow.show)
        self.sts.disconnectFromLocalGPS.connect(self.localGPSWindow.connection.disconnect)
        self.window.setLayout(mainLayout)
        self.setCentralWidget(self.window)

    def createConnection(self, conn):
        self.mav = MAVLinkConnection(conn)
        self.mav.gpsRawIntHandler.connect(self.droneLocationHandler)
        self.mav.altitudeHandler.connect(self.droneAttitudeHandler)
        self.mav.systemStatusHandler.connect(self.droneStatusHandler)
        self.mav.start()

    def droneStatusHandler(self, msg):
        # mV mA -> V A
        self.sts.updateBatteryStatus(msg.voltage_battery / 1000.0, msg.current_battery / 1000.0, msg.battery_remaining)

    def droneLocationHandler(self, msg):
        scale = 1E7
        self.m.mapView.updateDroneLocation(msg.lat / scale, msg.lon / scale, msg.eph / 100, msg.epv / 100)
        self.sts.updateGPSFixStatus(msg.fix_type)
        self.pfd.updateGPSAltitude(0, msg.time_usec, msg.alt / 1000.0) # mm -> meter

    def droneAttitudeHandler(self, msg):
        scale = 180 / math.pi
        self.pfd.updateAttitude(0, msg.time_boot_ms, msg.roll * scale, msg.pitch * scale, msg.yaw * scale)
        self.pfd.updateAttitudeSpeed(0, msg.time_boot_ms, msg.rollspeed * scale, msg.pitchspeed * scale, msg.yawspeed * scale)

    def disconnect(self):
        self.mav.requestExit()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    frame = MiniGCS()
    frame.show()
    sys.exit(app.exec_())
