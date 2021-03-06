from abc import abstractmethod
from math import log, sqrt
from PyQt5.QtCore import QObject, pyqtSignal
from pymavlink.mavutil import mavlink
from utils import unused
from UserData import UserData

UINT16_MAX = 0xFFFF
MAVLINK_LXTITUDE_SCALE = 1E7
UNIVERSAL_GAS_CONSTANT = 8.3144598 # J/(mol·K)
MOLAR_MASS = 0.0289644 # kg/mol
gravity = 9.80665 # m/s2
DEFAULT_ALTITUDE_REFERENCE = 0.0  # METER
DEFAULT_PRESSURE_REFERENCE = 101325.0  # PA
ZERO_KELVIN = -273.15 # degree

UD_UAS_CONF_KEY = 'UAS'
UD_UAS_CONF_GPS_SRC_KEY = 'GPS_SRC'

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
    updateRCChannelsSignal = pyqtSignal(object, int, int, object) # uas, timestamp, rssi, channels
    mavlinkMessageTxSignal = pyqtSignal(object) # mavlink message object

    def __init__(self, name, parent = None):
        super().__init__(parent)
        self.uasName = name
        self.autopilotClass = mavlink.MAV_AUTOPILOT_GENERIC
        self.param = UserData.getInstance().getUserDataEntry(UD_UAS_CONF_KEY, {})
        self.onboardParameters = []
        self.oldOnboardParameters = []
        self.onboardParamNotReceived = 0
        self.messageHandlers = {}
        self.messageHandlers['SYS_STATUS'] = self.uasStatusHandler
        self.messageHandlers['GPS_RAW_INT'] = self.uasLocationHandler
        self.messageHandlers['GLOBAL_POSITION_INT'] = self.uasFilteredLocationHandler
        self.messageHandlers['SCALED_PRESSURE'] = self.uasAltitudeHandler
        self.messageHandlers['ATTITUDE'] = self.uasAttitudeHandler
        self.messageHandlers['GPS_STATUS'] = self.uasGPSStatusHandler
        self.messageHandlers['RADIO_STATUS'] = self.uasRadioStatusHandler
        self.messageHandlers['RC_CHANNELS'] = self.uasRCChannelsHandler
        self.messageHandlers['NAV_CONTROLLER_OUTPUT'] = self.uasNavigationControllerOutputHandler
        self.messageHandlers['LOCAL_POSITION_NED'] = self.uasDefaultMessageHandler
        self.messageHandlers['PARAM_VALUE'] = self.uasDefaultMessageHandler
        self.messageHandlers['HEARTBEAT'] = self.uasDefaultMessageHandler
        self.messageHandlers['ATTITUDE_QUATERNION'] = self.uasDefaultMessageHandler
        self.messageHandlers['SYSTEM_TIME'] = self.uasDefaultMessageHandler
        self.messageHandlers['VFR_HUD'] = self.uasDefaultMessageHandler
        self.messageHandlers['AUTOPILOT_VERSION'] = self.uasDefaultMessageHandler
        self.messageHandlers['BATTERY_STATUS'] = self.uasDefaultMessageHandler
        self.messageHandlers['SCALED_IMU'] = self.uasDefaultMessageHandler
        self.messageHandlers['RAW_IMU'] = self.uasDefaultMessageHandler
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
    def uasGPSStatusHandler(self, msg):
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
    def uasRCChannelsHandler(self, msg):
        pass

    @abstractmethod
    def uasNavigationControllerOutputHandler(self, msg):
        pass

    @abstractmethod
    def fetchAllOnboardParameters(self):
        pass

    def uasDefaultMessageHandler(self, msg):
        pass

    def __backupOnboardParameters(self, dest, src):
        '''
        Copy parameters from `src` to `dest`,
        any existing parameters in `dest` will be removed.
        Parameters in `src` will also be removed after the backup.
        '''
        while len(dest) > 0:
            del dest[0]
        while len(src) > 0:
            dest.append(src[0])
            del src[0]

    def resetOnboardParameterList(self):
        # copy onboardParameters to oldOnboardParameters
        self.__backupOnboardParameters(self.oldOnboardParameters, self.onboardParameters)

    def restoreToOldOnboardParameterList(self):
        # copy oldOnboardParameters to onboardParameters
        self.__backupOnboardParameters(self.onboardParameters, self.oldOnboardParameters)

    def getPressureAltitude(self, pressure, temperature):
        try:
            kelvin = temperature - ZERO_KELVIN
            altitude = self.altitudeReference - (UNIVERSAL_GAS_CONSTANT * kelvin) * log(pressure / self.pressureReference) / (gravity * MOLAR_MASS)
            return altitude
        except ValueError:
            return 0

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
        if mavlink.WIRE_PROTOCOL_VERSION == '2.0':
            msg = mavlink.MAVLink_setup_signing_message(255, 255, self.signingKey, self.initialTimestamp)
            self.mavlinkMessageTxSignal.emit(msg)

    def allowUnsignedCallback(self, mav, msgId):
        unused(mav, msgId)
        return True

class StandardMAVLinkInterface(UASInterface):

    DEFAULT_GPS_SRC = 'GPS_RAW_INT'

    def __init__(self, name, parent = None):
        super().__init__(name, parent)
        self.gpsSrc = UserData.getParameterValue(self.param, UD_UAS_CONF_GPS_SRC_KEY, StandardMAVLinkInterface.DEFAULT_GPS_SRC)

    def uasStatusHandler(self, msg):
        self.updateBatterySignal.emit(self, 0, msg.voltage_battery / 1000.0, msg.current_battery / 1000.0, msg.battery_remaining)

    def uasLocationHandler(self, msg):
        if (self.gpsSrc == msg.get_type()):
            self.updateGlobalPositionSignal.emit(self, msg.time_usec, msg.lat / MAVLINK_LXTITUDE_SCALE, msg.lon / MAVLINK_LXTITUDE_SCALE, msg.alt / 1000.0)
            self.updateGPSAltitudeSignal.emit(self, msg.time_usec, msg.alt / 1000.0) # mm -> meter
            if msg.vel != UINT16_MAX:
                self.updateGroundSpeedSignal.emit(self, msg.time_usec, msg.vel / 100 * 3.6)  # cm/s to km/h
        self.updateGPSStatusSignal.emit(self, msg.time_usec, msg.fix_type, msg.eph, msg.epv, msg.satellites_visible, 0, 0, 0, 0)

    def uasFilteredLocationHandler(self, msg):
        if (self.gpsSrc == msg.get_type()):
            self.updateGlobalPositionSignal.emit(self, msg.time_boot_ms, msg.lat / MAVLINK_LXTITUDE_SCALE, msg.lon / MAVLINK_LXTITUDE_SCALE, msg.alt / 1000.0)
            self.updateGPSAltitudeSignal.emit(self, msg.time_boot_ms, msg.alt / 1000.0) # mm -> meter
            vel = msg.vx * msg.vx
            vel += msg.vy * msg.vy
            vel += msg.vz * msg.vz
            self.updateGroundSpeedSignal.emit(self, msg.time_boot_ms, sqrt(vel) / 100 * 3.6)  # cm/s to km/h

    def uasAltitudeHandler(self, msg):
        self.updateAirPressureSignal.emit(self, msg.time_boot_ms, msg.press_abs, msg.press_diff, msg.temperature)
        alt = self.getPressureAltitude(msg.press_abs * 100, msg.temperature)  # hPa to Pa
        self.updatePrimaryAltitudeSignal.emit(self, msg.time_boot_ms, alt)

    def uasAttitudeHandler(self, msg):
        self.updateAttitudeSignal.emit(self, msg.time_boot_ms, msg.roll, msg.pitch, msg.yaw)

    def uasRadioStatusHandler(self, msg):
        self.updateRCStatusSignal.emit(self, 0, msg.rssi, msg.noise, msg.rxerrors)

    def uasRCChannelsHandler(self, msg):
        rcChannels = {}
        for i in range(msg.chancount):
            ch = 'chan{}_raw'.format(i + 1)
            rcChannels[i + 1] = getattr(msg, ch)
        self.updateRCChannelsSignal.emit(self, msg.time_boot_ms, msg.rssi, rcChannels)

    def uasGPSStatusHandler(self, msg):
        # can be used to view gps SNR
        pass

    def acceptOnboardParameter(self, msg):
        self.onboardParameters.append(msg)
        self.onboardParamNotReceived = msg.param_count - msg.param_index - 1

    def fetchAllOnboardParameters(self):
        self.resetOnboardParameterList()
        self.mavlinkMessageTxSignal.emit(mavlink.MAVLink_param_request_list_message(255, 0))

    def uasNavigationControllerOutputHandler(self, msg):
        self.updateNavigationControllerOutputSignal.emit(self, msg.nav_roll, msg.nav_pitch, msg.nav_bearing, msg.target_bearing, msg.wp_dist)

class AutoQuadMAVLinkInterface(StandardMAVLinkInterface):

    def __init__(self, name, parent = None):
        super().__init__(name, parent)
        self.autopilotClass = mavlink.MAV_AUTOPILOT_AUTOQUAD

    def uasLocationHandler(self, msg):
        self.updateGlobalPositionSignal.emit(self, msg.time_usec, msg.lat / MAVLINK_LXTITUDE_SCALE, msg.lon / MAVLINK_LXTITUDE_SCALE, msg.alt / 1000.0)
        self.updateGPSAltitudeSignal.emit(self, msg.time_usec, msg.alt / 1000.0) # mm -> meter
        self.updateGPSStatusSignal.emit(self, msg.time_usec, msg.fix_type, UINT16_MAX, UINT16_MAX, msg.satellites_visible, int(msg.eph / 100), int(msg.epv / 100), 0, 0)
        if msg.vel != UINT16_MAX:
            self.updateGroundSpeedSignal.emit(self, msg.time_usec, msg.vel / 100 * 3.6)  # cm/s to km/h

class UASInterfaceFactory:
    UAS_INTERFACES = {}

    @staticmethod
    def __initUas():
        UASInterfaceFactory.UAS_INTERFACES[mavlink.MAV_AUTOPILOT_GENERIC] = StandardMAVLinkInterface('Generic MAVLink Interface')
        UASInterfaceFactory.UAS_INTERFACES[mavlink.MAV_AUTOPILOT_AUTOQUAD] = AutoQuadMAVLinkInterface('AutoQuad MAVLink Interface')

    @staticmethod
    def getUASInterface(dialect):
        if len(UASInterfaceFactory.UAS_INTERFACES) == 0:
            UASInterfaceFactory.__initUas()
        if dialect in UASInterfaceFactory.UAS_INTERFACES:
            return UASInterfaceFactory.UAS_INTERFACES[dialect]
        inst = UASInterfaceFactory.UAS_INTERFACES[mavlink.MAV_AUTOPILOT_GENERIC]
        inst.autopilotClass = dialect
        return inst
