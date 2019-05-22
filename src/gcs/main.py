import math
import os
import sys
import pymavlink

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication, QMainWindow, QSizePolicy, QSplitter

from LocalGPS import GPSConfigurationWindow
from map import MapWidget
from pfd import PrimaryFlightDisplay
from statusPanel import SystemStatusPanel
from telemetry import ConnectionEditWindow, MAVLinkConnection
from UserData import UserData
from HUD import HUDWindow

UD_MAIN_WINDOW_KEY = 'MAIN'
UD_MAIN_WINDOW_HEIGHT_KEY = 'WINDOW_HEIGHT'
UD_MAIN_WINDOW_WIDTH_KEY = 'WINDOW_WIDTH'

class MiniGCS(QMainWindow):

    mav = None

    def __init__(self, parent = None):
        super().__init__(parent)
        self.param = UserData.getInstance().getUserDataEntry(UD_MAIN_WINDOW_KEY, {})
        current_path = os.path.abspath(os.path.dirname(__file__))
        qmlFile = os.path.join(current_path, 'map.qml')
        self.setWindowTitle('Mini GCS')
        if UD_MAIN_WINDOW_HEIGHT_KEY in self.param and UD_MAIN_WINDOW_WIDTH_KEY in self.param:
            self.resize(self.param[UD_MAIN_WINDOW_WIDTH_KEY], self.param[UD_MAIN_WINDOW_HEIGHT_KEY])
        self.teleWindow = ConnectionEditWindow()
        self.teleWindow.MAVLinkConnectedSignal.connect(self.createConnection)
        self.window = QSplitter()
        self.left = QSplitter(Qt.Vertical, self.window)
        self.pfd = PrimaryFlightDisplay(self.window)
        self.sts = SystemStatusPanel(self.window)
        self.hudWindow = HUDWindow()
        self.hud = self.hudWindow.hud
        self.sts.connectToMAVLink.connect(self.teleWindow.show)
        self.sts.disconnectFromMAVLink.connect(self.disconnect)
        self.left.addWidget(self.pfd)
        self.left.addWidget(self.sts)
        spLeft = QSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)
        spLeft.setHorizontalStretch(1)
        self.left.setSizePolicy(spLeft)
        self.map = MapWidget(qmlFile)
        spRight = QSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)
        spRight.setHorizontalStretch(2)
        self.map.setSizePolicy(spRight)
        self.window.addWidget(self.left)
        self.window.addWidget(self.map)
        self.localGPSWindow = GPSConfigurationWindow()
        # TODO configurable behavior
        self.localGPSWindow.connection.locationUpdate.connect(self.map.updateHomeLocationEvent)
        self.sts.connectToLocalGPS.connect(self.localGPSWindow.show)
        self.sts.disconnectFromLocalGPS.connect(self.localGPSWindow.connection.disconnect)
        self.sts.statusPanel.showHUDButton.clicked.connect(self.hudWindow.show)
        self.teleWindow.MAVLinkConnectedSignal.connect(lambda: self.sts.statusPanel.toggleButtonLabel(True))
        self.teleWindow.cancelConnectionSignal.connect(lambda: self.sts.statusPanel.connectButton.setEnabled(True))
        self.setCentralWidget(self.window)

    def createConnection(self, conn):
        self.mav = MAVLinkConnection(conn, isinstance(conn, pymavlink.mavutil.mavlogfile))
        self.map.waypointList.requestReturnToHome.connect(self.mav.initializeReturnToHome)
        self.map.uploadWaypointsToUAVEvent.connect(self.mav.uploadWaypoints)
        self.map.downloadWaypointsFromUAVSignal.connect(self.mav.downloadWaypoints)

        self.mav.connectionEstablishedSignal.connect(lambda: self.sts.statusPanel.editParameterButton.setEnabled(True))
        self.mav.connectedToAPTypeSignal.connect(self.sts.addAPControlPanel)
        self.mav.newTextMessageSignal.connect(self.map.displayTextMessage)
        self.mav.onboardWaypointsReceivedSignal.connect(self.map.setAllWaypoints)

        self.mav.gpsRawIntHandler.connect(self.droneLocationHandler)
        self.mav.altitudeHandler.connect(self.droneAttitudeHandler)
        self.mav.scaledPressureHandler.connect(self.droneAltitudeHandler)
        self.mav.systemStatusHandler.connect(self.droneStatusHandler)

        self.pfd.setActiveUAS(self.mav.uas)
        self.hud.setActiveUAS(self.mav.uas)

        self.sts.statusPanel.editParameterButton.clicked.connect(self.mav.showParameterEditWindow)
        self.sts.initializaMavlinkForControlPanels(self.mav)
        self.mav.start()

    def droneStatusHandler(self, msg):
        # mV mA -> V A
        self.sts.statusPanel.updateBatteryStatus(msg.voltage_battery / 1000.0, msg.current_battery / 1000.0, msg.battery_remaining)

    def droneLocationHandler(self, msg):
        scale = 1E7
        self.map.mapView.updateDroneLocation(msg.lat / scale, msg.lon / scale, msg.eph / 100, msg.epv / 100)
        self.sts.statusPanel.updateGPSFixStatus(msg.fix_type)
        self.pfd.updateGPSReception(0, msg.time_usec, msg.fix_type, msg.satellites_visible)

    def droneAltitudeHandler(self, msg):
        self.sts.barometerPanel.setBarometer(msg.press_abs)

    def droneAttitudeHandler(self, msg):
        self.sts.compassPanel.setHeading(msg.yaw)

    def disconnect(self):
        self.mav.requestExit()

    def closeEvent(self, event):
        print('[MAIN] closeEvent')
        ud = UserData.getInstance()
        s = self.size()
        self.param[UD_MAIN_WINDOW_HEIGHT_KEY] = s.height()
        self.param[UD_MAIN_WINDOW_WIDTH_KEY] = s.width()
        ud.setUserDataEntry(UD_MAIN_WINDOW_KEY, self.param)
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

if __name__ == '__main__':
    app = QApplication(sys.argv)
    try:
        UserData.getInstance().loadGCSConfiguration()
    except IOError:
        sys.exit(1)
    frame = MiniGCS()
    frame.show()
    sys.exit(app.exec_())
