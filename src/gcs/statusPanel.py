from PyQt5.QtWidgets import QWidget, QGridLayout, QLabel, QApplication, QProgressBar, QPushButton
from PyQt5.QtCore import Qt, pyqtSignal
from pymavlink.dialects.v10 import common as mavlink

import sys

GPS_FIX_LABELS = {
    mavlink.GPS_FIX_TYPE_NO_GPS : 'No GPS',
    mavlink.GPS_FIX_TYPE_NO_FIX : 'No fix',
    mavlink.GPS_FIX_TYPE_2D_FIX : '2D fix',
    mavlink.GPS_FIX_TYPE_3D_FIX : '3D fix',
    mavlink.GPS_FIX_TYPE_DGPS : 'DGPS/SBAS aided 3D fix',
    mavlink.GPS_FIX_TYPE_RTK_FLOAT : 'RTK 3D float',
    mavlink.GPS_FIX_TYPE_RTK_FIXED : 'RTK 3D fix',
    mavlink.GPS_FIX_TYPE_STATIC : 'Static fix',
    mavlink.GPS_FIX_TYPE_PPP : 'PPP 3D fix'
}

class SystemStatusPanel(QWidget):

    connectToMAVLink = pyqtSignal()
    disconnectFromMAVLink = pyqtSignal()

    def __init__(self, parent = None):
        super().__init__(parent)
        l = QGridLayout()
        row = 0
        self.sysNameLbl = QLabel('System Status')
        l.addWidget(self.sysNameLbl, row, 0, 1, 3, Qt.AlignLeft)
        row += 1

        self.armedLbl = QLabel('Disarmed')
        self.modeLbl = QLabel('Standby')
        self.gpsLbl = QLabel(GPS_FIX_LABELS[mavlink.GPS_FIX_TYPE_NO_GPS])
        l.addWidget(self.armedLbl, row, 0, 1, 1, Qt.AlignLeft)
        l.addWidget(self.modeLbl, row, 1, 1, 1, Qt.AlignLeft)
        l.addWidget(self.gpsLbl, row, 2, 1, 1, Qt.AlignLeft)
        row += 1

        l.addWidget(QLabel('Battery'), row, 0, 1, 1, Qt.AlignLeft)
        self.battBar = QProgressBar(self)
        self.battBar.setValue(0)
        l.addWidget(self.battBar, row, 1, 1, 1, Qt.AlignLeft)
        self.battVoltLabel = QLabel('0.0V')
        l.addWidget(self.battVoltLabel, row, 2, 1, 1, Qt.AlignLeft)
        row += 1
        self.radioBar = QProgressBar(self)
        self.radioBar.setValue(0)
        l.addWidget(QLabel('Radio'), row, 0, 1, 1, Qt.AlignLeft)
        l.addWidget(self.radioBar, row, 1, 1, 2, Qt.AlignLeft)
        row += 1

        self.connectButton = QPushButton('Connect')
        self.connectLabelShown = True
        self.connectButton.clicked.connect(self.toggleButtonLabel)
        l.addWidget(self.connectButton, row, 2, 1, 1, Qt.AlignLeft)

        self.setLayout(l)

    def updateBatteryStatus(self, voltage, current, remaining):
        self.battVoltLabel.setText('{}V'.format(voltage))
        self.battBar.setValue(remaining)

    def updateGPSFixStatus(self, fix):
        if fix in GPS_FIX_LABELS:
            self.gpsLbl.setText(GPS_FIX_LABELS[fix])
        else:
            self.gpsLbl.setText(GPS_FIX_LABELS[mavlink.GPS_FIX_TYPE_NO_FIX])

    def toggleButtonLabel(self):
        if self.connectLabelShown:
            self.connectToMAVLink.emit()
            self.connectLabelShown = False
            self.connectButton.setText('Disconnect')
        else:
            self.disconnectFromMAVLink.emit()
            self.connectLabelShown = True
            self.connectButton.setText('Connect')


# test
if __name__ == "__main__":
    app = QApplication(sys.argv)
    ssp = SystemStatusPanel()
    ssp.show()
    sys.exit(app.exec_())
