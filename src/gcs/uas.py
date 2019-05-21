from abc import abstractmethod
from PyQt5.QtCore import QObject

class UASInterface(QObject):

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

