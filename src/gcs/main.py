import sys, time, os

from PyQt5.QtWidgets import QApplication, QHBoxLayout, QWidget, QMainWindow
from PyQt5.QtCore import pyqtSignal, QThread

from pfd import PrimaryFlightDisplay
from map import MapWidget

#from compass import Compass
#from barometer import Barometer

class UpdateThread(QThread):
    updateHeading = pyqtSignal(float, name='hdr')
    updateBaro = pyqtSignal(float, name='hbar')

    def run(self):
        count = 0.0
        baro = 950.0
        while True:
            time.sleep(1)
            count += 5.0
            baro += 5.0
            if count > 360.0:
                count = count - 360.0
            if baro > 1050.0:
                baro = 950.0
            self.updateHeading.emit(count)
            self.updateBaro.emit(baro)

if __name__ == "__main__":
    app = QApplication(sys.argv)

    current_path = os.path.abspath(os.path.dirname(__file__))
    qmlFile = os.path.join(current_path, 'map.qml')

    frame = QMainWindow()

    window = QWidget()
    layout = QHBoxLayout()
    pfd = PrimaryFlightDisplay(window)
    layout.addWidget(pfd)
    m = MapWidget(qmlFile)
    layout.addWidget(m)
    window.setLayout(layout)
    frame.setCentralWidget(window)
    frame.showMaximized()
    # window.showMaximized()

    # thread = UpdateThread()
    # thread.updateHeading.connect(comp.setHeading)
    # thread.updateBaro.connect(pfd.setBarometer)
    # thread.start()
    sys.exit(app.exec_())
