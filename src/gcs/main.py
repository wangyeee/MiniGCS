import math
import os
import sys
import pymavlink

from PyQt5.QtWidgets import (QApplication, QHBoxLayout, QMainWindow,
                             QSizePolicy, QVBoxLayout, QWidget)

from map import MapWidget
from pfd import PrimaryFlightDisplay
from statusPanel import SystemStatusPanel
from telemetry import ConnectionEditWindow, MAVLinkConnection
from LocalGPS import GPSConfigurationWindow
from UserData import UserData

class MiniGCS(QMainWindow):

    mav = None

    def __init__(self, parent = None):
        super().__init__(parent)
        current_path = os.path.abspath(os.path.dirname(__file__))
        qmlFile = os.path.join(current_path, 'map.qml')
        self.setWindowTitle('Mini GCS')
        self.teleWindow = ConnectionEditWindow()
        self.teleWindow.MAVLinkConnectedSignal.connect(self.createConnection)
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
        self.map = MapWidget(qmlFile)
        spRight = QSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)
        spRight.setHorizontalStretch(2)
        self.map.setSizePolicy(spRight)
        mainLayout.addWidget(self.left)
        mainLayout.addWidget(self.map)
        self.localGPSWindow = GPSConfigurationWindow()
        # TODO configurable behavior
        self.localGPSWindow.connection.locationUpdate.connect(self.map.updateHomeLocationEvent)
        self.sts.connectToLocalGPS.connect(self.localGPSWindow.show)
        self.sts.disconnectFromLocalGPS.connect(self.localGPSWindow.connection.disconnect)
        self.teleWindow.MAVLinkConnectedSignal.connect(lambda: self.sts.statusPanel.toggleButtonLabel(True))
        self.teleWindow.cancelConnectionSignal.connect(lambda: self.sts.statusPanel.connectButton.setEnabled(True))
        self.window.setLayout(mainLayout)
        self.setCentralWidget(self.window)

    def createConnection(self, conn):
        self.mav = MAVLinkConnection(conn, isinstance(conn, pymavlink.mavutil.mavlogfile))
        self.map.waypointList.requestReturnToHome.connect(self.mav.initializeReturnToHome)
        self.map.uploadWaypointsToUAVEvent.connect(self.mav.uploadWaypoints)
        self.map.downloadWaypointsFromUAVSignal.connect(self.mav.downloadWaypoints)
        self.mav.gpsRawIntHandler.connect(self.droneLocationHandler)
        self.mav.altitudeHandler.connect(self.droneAttitudeHandler)
        self.mav.systemStatusHandler.connect(self.droneStatusHandler)
        self.sts.statusPanel.editParameterButton.clicked.connect(self.mav.showParameterEditWindow)
        self.mav.connectionEstablishedSignal.connect(lambda: self.sts.statusPanel.editParameterButton.setEnabled(True))
        self.mav.onboardWaypointsReceivedSignal.connect(self.map.setAllWaypoints)
        self.mav.newTextMessageSignal.connect(self.map.displayTextMessage)
        self.mav.start()

    def droneStatusHandler(self, msg):
        # mV mA -> V A
        self.sts.statusPanel.updateBatteryStatus(msg.voltage_battery / 1000.0, msg.current_battery / 1000.0, msg.battery_remaining)

    def droneLocationHandler(self, msg):
        scale = 1E7
        self.map.mapView.updateDroneLocation(msg.lat / scale, msg.lon / scale, msg.eph / 100, msg.epv / 100)
        self.sts.statusPanel.updateGPSFixStatus(msg.fix_type)
        self.pfd.updateGPSAltitude(0, msg.time_usec, msg.alt / 1000.0) # mm -> meter

    def droneAttitudeHandler(self, msg):
        scale = 180 / math.pi
        self.pfd.updateAttitude(0, msg.time_boot_ms, msg.roll * scale, msg.pitch * scale, msg.yaw * scale)
        self.pfd.updateAttitudeSpeed(0, msg.time_boot_ms, msg.rollspeed * scale, msg.pitchspeed * scale, msg.yawspeed * scale)
        self.sts.compassPanel.setHeading(msg.yaw * scale)

    def disconnect(self):
        self.mav.requestExit()

    def closeEvent(self, event):
        print('[MAIN] closeEvent')
        ud = UserData.getInstance()
        if self.map != None:
            ps = self.map.getParametersToSave(True)
            for p in ps:
                ud.setUserDataEntry(p[0], p[1])
        try:
            ud.saveGCSConfiguration()
            print('GCS Conf saved.')
        except IOError:
            pass
        super().closeEvent(event)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    try:
        UserData.getInstance().loadGCSConfiguration()
    except IOError:
        sys.exit(1)
    frame = MiniGCS()
    frame.show()
    sys.exit(app.exec_())
