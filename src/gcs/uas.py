from abc import abstractmethod
from math import log
from PyQt5.QtCore import QObject, pyqtSignal
from pymavlink.dialects.v10 import common as mavlink

UINT16_MAX = 0xFFFF
UNIVERSAL_GAS_CONSTANT = 8.3144598 # J/(mol·K)
MOLAR_MASS = 0.0289644 # kg/mol
gravity = 9.80665 # m/s2

class UASInterface(QObject):

    updateAttitudeSignal = pyqtSignal(object, int, float, float, float) # uas, timestamp, roll, pitch, yaw
    updateBatterySignal = pyqtSignal(object, int, float, float, float) # uas, timestamp, voltage, current, percent
    updateGlobalPositionSignal = pyqtSignal(object, int, float, float, float) # uas, timestamp, lat, lng, altitude
    updateGPSStatusSignal = pyqtSignal(object, int, int, int, int, int, int, int, int, int) # uas, timestamp, fix type, HDOP, VDOP, satellites visible, h acc, v acc, vel acc, hdg acc
    updateAirSpeedSignal = pyqtSignal(object, int, float) # uas, timestamp, speed
    updateGroundSpeedSignal = pyqtSignal(object, int, float) # uas, timestamp, speed
    updateVelocitySignal = pyqtSignal(object, int, float, float, float) # uas, timestamp, x, y, z
    updatePrimaryAltitudeSignal = pyqtSignal(object, int, float) # uas, timestamp, altitude
    updateGPSAltitudeSignal = pyqtSignal(object, int, float) # uas, timestamp, altitude
    updateAirPressureSignal = pyqtSignal(object, int, float, float, float) # uas, timestamp, abs press, diff press, temperature

    def __init__(self, name, parent = None):
        super().__init__(parent)
        self.uasName = name
        self.messageHandlers = {}
        self.messageHandlers['SYS_STATUS'] = self.uasStatusHandler
        self.messageHandlers['GPS_RAW_INT'] = self.uasLocationHandler
        self.messageHandlers['SCALED_PRESSURE'] = self.uasAltitudeHandler
        self.messageHandlers['ATTITUDE'] = self.uasAttitudeHandler
        self.altitudeReference = 0.0  # meter
        self.pressureReference = 101325.0  # Pa

    def receiveMAVLinkMessage(self, msg):
        tp = msg.get_type()
        if tp in self.messageHandlers:
            self.messageHandlers[tp](msg)

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

    def getPressureAltitude(self, pressure, temperature):
        kelvin = temperature + 273.0
        altitude = self.altitudeReference - (UNIVERSAL_GAS_CONSTANT * kelvin) * log(pressure / self.pressureReference) / (gravity * MOLAR_MASS)
        return altitude

class StandardMAVLinkInterface(UASInterface):

    def uasStatusHandler(self, msg):
        self.updateBatterySignal.emit(self, 0, msg.voltage_battery / 1000.0, msg.current_battery / 1000.0, msg.battery_remaining)

    def uasLocationHandler(self, msg):
        scale = 1E7
        self.updateGlobalPositionSignal.emit(self, msg.time_usec, msg.lat / scale, msg.lon / scale, msg.alt / 1000.0)
        self.updateGPSAltitudeSignal.emit(self, msg.time_usec, msg.alt / 1000.0) # mm -> meter
        self.updateGPSStatusSignal.emit(self, msg.fix_type, msg.time_usec, msg.eph, msg.epv, msg.satellites_visible, 0, 0, 0, 0)
        if msg.vel != UINT16_MAX:
            self.updateGroundSpeedSignal.emit(self, msg.time_usec, msg.vel / 100 * 3.6)  # cm/s to km/h

    def uasAltitudeHandler(self, msg):
        self.updateAirPressureSignal.emit(self, msg.time_usec, msg.press_abs, msg.press_diff, msg.temperature)
        alt = self.getPressureAltitude(msg.press_abs, msg.temperature)
        self.updatePrimaryAltitudeSignal.emit(self, msg.time_usec, alt)

    def uasAttitudeHandler(self, msg):
        self.updateAttitudeSignal.emit(self, msg.time_boot_ms, msg.roll, msg.pitch, msg.yaw)

class AutoQuadMAVLinkInterface(StandardMAVLinkInterface):

    def uasLocationHandler(self, msg):
        scale = 1E7
        self.updateGlobalPositionSignal.emit(self, msg.time_usec, msg.lat / scale, msg.lon / scale, msg.alt / 1000.0)
        self.updateGPSAltitudeSignal.emit(self, msg.time_usec, msg.alt / 1000.0) # mm -> meter
        self.updateGPSStatusSignal.emit(self, msg.fix_type, msg.time_usec, UINT16_MAX, UINT16_MAX, msg.satellites_visible, int(msg.eph / 100), int(msg.epv / 100), 0, 0)
        if msg.vel != UINT16_MAX:
            self.updateGroundSpeedSignal.emit(self, msg.time_usec, msg.vel / 100 * 3.6)  # cm/s to km/h

class UASInterfaceFactory:
    UAS_INTERFACES = {
        mavlink.MAV_AUTOPILOT_GENERIC : StandardMAVLinkInterface('Generic MAVLink Interface'),
        mavlink.MAV_AUTOPILOT_AUTOQUAD : AutoQuadMAVLinkInterface('AutoQuad MAVLink Interface')
    }

    @staticmethod
    def getUASInterface(dialect):
        if dialect in UASInterfaceFactory.UAS_INTERFACES:
            return UASInterfaceFactory.UAS_INTERFACES[dialect]
        return UASInterfaceFactory.UAS_INTERFACES[mavlink.MAV_AUTOPILOT_GENERIC]
