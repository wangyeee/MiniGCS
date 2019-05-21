from abc import abstractmethod
from PyQt5.QtCore import QObject, pyqtSignal

class UASInterface(QObject):

    updateAttitudeSignal = pyqtSignal(object, int, float, float, float) # uas, timestamp, roll, pitch, yaw
    updateBatterySignal = pyqtSignal(object, int, float, float, float) # uas, timestamp, voltage, current, percent
    updateGlobalPositionSignal = pyqtSignal(object, int, float, float, float) # uas, timestamp, lat, lng, altitude
    updateAirSpeedSignal = pyqtSignal(object, int, float) # uas, timestamp, speed
    updateGroundSpeedSignal = pyqtSignal(object, int, float) # uas, timestamp, speed
    updateVelocitySignal = pyqtSignal(object, int, float, float, float) # uas, timestamp, x, y, z

    def __init__(self, name, parent = None):
        super().__init__(parent)
        self.uasName = name

    @abstractmethod
    def uasStatusHandler(self, msg):
        pass

    @abstractmethod
    def uasLocationHandler(self, msg):
        pass

    @abstractmethod
    def uasAltitudeHandler(self, msg):
        pass

    @abstractmethod
    def uasAttitudeHandler(self, msg):
        pass

