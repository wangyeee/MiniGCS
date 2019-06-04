from abc import abstractmethod
from math import log
from PyQt5.QtCore import QObject, pyqtSignal
from pymavlink import mavutil
from pymavlink.dialects.v10 import common as mavlink
from utils import unused

UINT16_MAX = 0xFFFF
UNIVERSAL_GAS_CONSTANT = 8.3144598 # J/(mol·K)
MOLAR_MASS = 0.0289644 # kg/mol
gravity = 9.80665 # m/s2
DEFAULT_ALTITUDE_REFERENCE = 0.0  # METER
DEFAULT_PRESSURE_REFERENCE = 101325.0  # PA
ZERO_KELVIN = -273.15 # degree

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
    updateRCStatusSignal = pyqtSignal(object, int, int, int, int) # uas, type(local, remote), rssi, noise, errors
    # uas, desiredRoll, desiredPitch, desiredHeading, targetBearing, wpDist
    updateNavigationControllerOutputSignal = pyqtSignal(object, float, float, float, float, float)
    mavlinkMessageTxSignal = pyqtSignal(object) # mavlink message object

    def __init__(self, name, parent = None):
        super().__init__(parent)
        self.uasName = name
        self.autopilotClass = mavlink.MAV_AUTOPILOT_GENERIC
        self.messageHandlers = {}
        self.messageHandlers['SYS_STATUS'] = self.uasStatusHandler
        self.messageHandlers['GPS_RAW_INT'] = self.uasLocationHandler
        self.messageHandlers['GLOBAL_POSITION_INT'] = self.uasFilteredLocationHandler
        self.messageHandlers['SCALED_PRESSURE'] = self.uasAltitudeHandler
        self.messageHandlers['ATTITUDE'] = self.uasAttitudeHandler

        self.messageHandlers['RADIO_STATUS'] = self.uasRadioStatusHandler
        self.messageHandlers['LOCAL_POSITION_NED'] = self.uasDefaultMessageHandler
        self.messageHandlers['NAV_CONTROLLER_OUTPUT'] = self.uasNavigationControllerOutputHandler
        self.messageHandlers['PARAM_VALUE'] = self.uasDefaultMessageHandler
        self.messageHandlers['HEARTBEAT'] = self.uasDefaultMessageHandler
        self.messageHandlers['ATTITUDE_QUATERNION'] = self.uasDefaultMessageHandler
        self.messageHandlers['SYSTEM_TIME'] = self.uasDefaultMessageHandler
        self.messageHandlers['VFR_HUD'] = self.uasDefaultMessageHandler
        self.messageHandlers['AUTOPILOT_VERSION'] = self.uasDefaultMessageHandler
        self.messageHandlers['BATTERY_STATUS'] = self.uasDefaultMessageHandler
        self.altitudeReference = DEFAULT_ALTITUDE_REFERENCE
        self.pressureReference = DEFAULT_PRESSURE_REFERENCE
        self.signingKey = None
        self.initialTimestamp = 0

    def receiveMAVLinkMessage(self, msg):
        tp = msg.get_type()
        if tp in self.messageHandlers:
            self.messageHandlers[tp](msg)
        else:
            print('UNKNOWN MSG:', msg)

    def setPressureAltitudeReference(self, presRef, altiRef):
        self.altitudeReference = altiRef
        self.pressureReference = presRef

    @abstractmethod
    def uasStatusHandler(self, msg):
        pass

    @abstractmethod
    def uasLocationHandler(self, msg):
        pass

    @abstractmethod
    def uasFilteredLocationHandler(self, msg):
        pass

    @abstractmethod
    def uasAltitudeHandler(self, msg):
        pass

    @abstractmethod
    def uasAttitudeHandler(self, msg):
        pass

    @abstractmethod
    def uasRadioStatusHandler(self, msg):
        pass

    @abstractmethod
    def uasNavigationControllerOutputHandler(self, msg):
        pass

    def uasDefaultMessageHandler(self, msg):
        pass

    def getPressureAltitude(self, pressure, temperature):
        kelvin = temperature - ZERO_KELVIN
        altitude = self.altitudeReference - (UNIVERSAL_GAS_CONSTANT * kelvin) * log(pressure / self.pressureReference) / (gravity * MOLAR_MASS)
        return altitude

    def acceptMessageSigningKey(self, key, ts):
        key0 = self.signingKey
        ts0 = self.initialTimestamp
        try:
            key0 = bytes.fromhex(key)
        except ValueError:
            pass
        try:
            ts0 = int(ts)
        except ValueError:
            pass
        self.signingKey = key0
        if ts0 > self.initialTimestamp:
            self.initialTimestamp = ts0
        if mavutil.mavlink.WIRE_PROTOCOL_VERSION == '2.0':
            from pymavlink.dialects.v20 import common as mavlinkv2
            msg = mavlinkv2.MAVLink_setup_signing_message(255, 255, self.signingKey, self.initialTimestamp)
            self.mavlinkMessageTxSignal.emit(msg)

    def allowUnsignedCallback(self, mav, msgId):
        unused(mav, msgId)
        return True

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

    def uasFilteredLocationHandler(self, msg):
        scale = 1E7
        self.updateGlobalPositionSignal.emit(self, msg.time_boot_ms, msg.lat / scale, msg.lon / scale, msg.alt / 1000.0)
        self.updateGPSAltitudeSignal.emit(self, msg.time_boot_ms, msg.alt / 1000.0) # mm -> meter

    def uasAltitudeHandler(self, msg):
        self.updateAirPressureSignal.emit(self, msg.time_usec, msg.press_abs, msg.press_diff, msg.temperature)
        alt = self.getPressureAltitude(msg.press_abs, msg.temperature)
        self.updatePrimaryAltitudeSignal.emit(self, msg.time_usec, alt)

    def uasAttitudeHandler(self, msg):
        self.updateAttitudeSignal.emit(self, msg.time_boot_ms, msg.roll, msg.pitch, msg.yaw)

    def uasRadioStatusHandler(self, msg):
        self.updateRCStatusSignal.emit(self, 0, msg.rssi, msg.noise, msg.rxerrors)

    def uasNavigationControllerOutputHandler(self, msg):
        self.updateNavigationControllerOutputSignal.emit(self, msg.nav_roll, msg.nav_pitch, msg.nav_bearing, msg.target_bearing, msg.wp_dist)

class AutoQuadMAVLinkInterface(StandardMAVLinkInterface):

    def __init__(self, name, parent = None):
        super().__init__(name, parent)
        self.autopilotClass = mavlink.MAV_AUTOPILOT_AUTOQUAD

    def uasLocationHandler(self, msg):
        scale = 1E7
        self.updateGlobalPositionSignal.emit(self, msg.time_usec, msg.lat / scale, msg.lon / scale, msg.alt / 1000.0)
        self.updateGPSAltitudeSignal.emit(self, msg.time_usec, msg.alt / 1000.0) # mm -> meter
        self.updateGPSStatusSignal.emit(self, msg.time_usec, msg.fix_type, UINT16_MAX, UINT16_MAX, msg.satellites_visible, int(msg.eph / 100), int(msg.epv / 100), 0, 0)
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
