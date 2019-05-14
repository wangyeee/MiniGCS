import socket
import time
import subprocess
import io
import os

from PyQt5.QtCore import (QAbstractListModel, QByteArray, QModelIndex, Qt,
                          QThread, QVariant, pyqtSignal)
from PyQt5.QtPositioning import QGeoCoordinate

# https://github.com/kanflo/ADS-B-funhouse
from sbs1 import SBS1Message

class AircraftsModel(QAbstractListModel):

    allAircrafts = []

    def __init__(self, parent = None):
        super().__init__(parent)
        self.positionRole = Qt.UserRole + 1
        self.headingRole = Qt.UserRole + 2
        self.callsignRole = Qt.UserRole + 3

    def rowCount(self, parent=QModelIndex()):
        return len(self.allAircrafts)

    def __findByICAO(self, icao):
        for i in range(self.rowCount()):
            if self.allAircrafts[i].icao24 == icao:
                return i
        return -1

    def updateAircraft(self, aircraft: SBS1Message):
        i = self.__findByICAO(aircraft.icao24)
        if i >= 0:
            idx = self.index(i)
            self.allAircrafts[i] = aircraft
            self.dataChanged.emit(idx, idx)

    def addAircraft(self, aircraft: SBS1Message):
        idx = self.rowCount()
        self.beginInsertRows(QModelIndex(), idx, idx)
        self.allAircrafts.append(aircraft)
        self.endInsertRows()

    def removeAircraft(self, aircraft: SBS1Message):
        i = self.__findByICAO(aircraft.icao24)
        if i >= 0:
            self.beginRemoveRows(QModelIndex(), i, i)
            del self.allAircrafts[i]
            self.endRemoveRows()

    def data(self, index, role=Qt.DisplayRole):
        idx = index.row()
        if idx < 0 or idx > len(self.allAircrafts) - 1:
            return QVariant()
        if role == self.positionRole:
            if self.allAircrafts[idx].lat and self.allAircrafts[idx].lon and self.allAircrafts[idx].altitude:
                # 3D position, altitude is displayed next to aircraft icon
                # Altitude from dump1090 is in feet, convert to meters by default
                altitudeInMeters = self.allAircrafts[idx].altitude * 0.3048
                return QVariant(QGeoCoordinate(self.allAircrafts[idx].lat, self.allAircrafts[idx].lon, altitudeInMeters))
            if self.allAircrafts[idx].lat and self.allAircrafts[idx].lon:
                # 2D position, 'Unknown Altitude' is displayed next to aircraft icon
                return QVariant(QGeoCoordinate(self.allAircrafts[idx].lat, self.allAircrafts[idx].lon))
            return QVariant(QGeoCoordinate())
        if role == self.headingRole:
            if self.allAircrafts[idx].lat and self.allAircrafts[idx].lon:
                if self.allAircrafts[idx].track:
                    return QVariant(self.allAircrafts[idx].track)
            return QVariant(0)
        if role == self.callsignRole:
            if self.allAircrafts[idx].lat and self.allAircrafts[idx].lon:
                if self.allAircrafts[idx].callsign:
                    return QVariant(self.allAircrafts[idx].callsign)
                # Display ICAO address when callsign is unavailable
                return QVariant('x{}'.format(self.allAircrafts[idx].icao24))
        return QVariant()

    def flags(self, index):
        return Qt.NoItemFlags

    def roleNames(self):
        return {
            self.positionRole : QByteArray(b'position'),
            self.headingRole : QByteArray(b'heading'),
            self.callsignRole : QByteArray(b'callsign')
        }

class ADSBSource(QThread):

    aircraftCreateSignal = pyqtSignal(object)
    aircraftUpdateSignal = pyqtSignal(object)
    aircraftDeleteSignal = pyqtSignal(object)

    running = True
    param = None

    def run(self):
        self.doLazyInit()
        while self.running:
            self.periodicalTask()
        self.cleanupTask()

    def stopADSB(self):
        self.running = False

    def lazyInit(self, param):
        self.param = param

    def doLazyInit(self):
        pass

    def periodicalTask(self):
        pass

    def cleanupTask(self):
        pass

    def getConfigurationParameterKey(self):
        return ''

    @staticmethod
    def getAvailableADSBSources():
        return [
            Dump1090NetClient(),
            Dump1090NetLocal()
        ]

class Dump1090NetClient(ADSBSource):

    DEFAULT_TIMEOUT = 10

    aircrafts = {}
    sckt = None
    timeout = DEFAULT_TIMEOUT  # default timeout, 10 seconds

    def __init__ (self, host = None, port = 0, parent = None):
        super().__init__(parent)
        self.lazyInit({'HOST' : host, 'PORT' : port, 'TIMEOUT' : self.DEFAULT_TIMEOUT})

    def doLazyInit(self):
        host = self.param['HOST']
        port = int(self.param['PORT'])
        if host != None and 0 < port < 65536:
            try:
                self.sckt = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sckt.connect((host, port))
                # The TIMEOUT parameter is optional
                self.timeout = int(self.param['TIMEOUT'])
                if self.timeout <= 0:
                    self.timeout = self.DEFAULT_TIMEOUT
            except ConnectionRefusedError:
                self.sckt = None
                print('Failed to connect to Dump1090 {}:{}'.format(host, port))
            except ValueError:
                pass

    def periodicalTask(self):
        if self.sckt != None:
            self.__receiveLatestData()
            self.__removeInactiveData()

    def __receiveLatestData(self):
        line = self.sckt.recv(1024)
        if line != b'':
            msg = SBS1Message(line)
            if msg.isValid:
                msg.loggedDate = time.time()  # re-use loggedDate field
                if msg.icao24 in self.aircrafts:
                    self.__updateMessageFields(self.aircrafts[msg.icao24], msg)
                    self.aircraftUpdateSignal.emit(self.aircrafts[msg.icao24])
                else:
                    self.aircrafts[msg.icao24] = msg
                    self.aircraftCreateSignal.emit(self.aircrafts[msg.icao24])

    def __removeInactiveData(self):
        timeNow = time.time()
        toRemove = []
        for icao, aircraft in self.aircrafts.items():
            if timeNow - self.timeout > aircraft.loggedDate:
                toRemove.append(icao)
                self.aircraftDeleteSignal.emit(aircraft)
        for rm in toRemove:
            del self.aircrafts[rm]

    def __updateMessageFields(self, currMsg, newMsg):
        currMsg.messageType = currMsg.messageType if newMsg.messageType == None else newMsg.messageType
        currMsg.transmissionType = currMsg.transmissionType if newMsg.transmissionType == None else newMsg.transmissionType
        currMsg.sessionID = currMsg.sessionID if newMsg.sessionID == None else newMsg.sessionID
        currMsg.aircraftID = currMsg.aircraftID if newMsg.aircraftID == None else newMsg.aircraftID
        currMsg.flightID = currMsg.flightID if newMsg.flightID == None else newMsg.flightID
        currMsg.generatedDate = currMsg.generatedDate if newMsg.generatedDate == None else newMsg.generatedDate
        currMsg.loggedDate = currMsg.loggedDate if newMsg.loggedDate == None else newMsg.loggedDate
        currMsg.callsign = currMsg.callsign if newMsg.callsign == None else newMsg.callsign
        currMsg.altitude = currMsg.altitude if newMsg.altitude == None else newMsg.altitude
        currMsg.groundSpeed = currMsg.groundSpeed if newMsg.groundSpeed == None else newMsg.groundSpeed
        currMsg.track = currMsg.track if newMsg.track == None else newMsg.track
        currMsg.lat = currMsg.lat if newMsg.lat == None else newMsg.lat
        currMsg.lon = currMsg.lon if newMsg.lon == None else newMsg.lon
        currMsg.verticalRate = currMsg.verticalRate if newMsg.verticalRate == None else newMsg.verticalRate
        currMsg.squawk = currMsg.squawk if newMsg.squawk == None else newMsg.squawk
        currMsg.alert = currMsg.alert if newMsg.alert == None else newMsg.alert
        currMsg.emergency = currMsg.emergency if newMsg.emergency == None else newMsg.emergency
        currMsg.spi = currMsg.spi if newMsg.spi == None else newMsg.spi
        currMsg.onGround = currMsg.onGround if newMsg.onGround == None else newMsg.onGround

    def cleanupTask(self):
        self.sckt.close()

    def getConfigurationParameterKey(self):
        return 'DUMP1090SBS1'

class Dump1090NetLocal(Dump1090NetClient):

    isBiasTeeSupported = False
    binCheckPass = False
    receiverProcess = None

    def __init__(self, dump1090BinPath = None, parent = None):
        super().__init__(parent)
        self.lazyInit({
            'DUMP1090_BIN' : dump1090BinPath,
            'DEVICE_IDX' : 0,
            'SBS_PORT' : 30003,
            'TIMEOUT' : self.DEFAULT_TIMEOUT,
            'BIAS_TEE' : False
        })

    def doLazyInit(self):
        print(self.param)
        self.__checkDump1090Binary()
        if self.binCheckPass:
            self.__startDump1090()
            self.param['PORT'] = self.param['SBS_PORT']
            self.param['HOST'] = 'localhost'
            super().doLazyInit()

    def __checkDump1090Binary(self):
        try:
            ps = subprocess.run([self.param['DUMP1090_BIN'], '--help'],
                                 stdout = subprocess.PIPE,
                                 stderr = subprocess.STDOUT)
            if ps.returncode == 0:
                self.binCheckPass = True
                texts = io.StringIO(ps.stdout.decode('utf-8'))
                for text in texts:
                    if text.startswith('--enable-bias-tee'):
                        # Check if dump1090 supports setting bias T in RTL-SDR
                        # Enabling bias T with supported RTL-SDR receiver and external
                        # LNA will significantly increase range
                        # Source: https://github.com/wangyeee/dump1090/tree/minigcs_integration
                        # Receiver: https://www.rtl-sdr.com/rtl-sdr-blog-v-3-dongles-user-guide
                        # LNA: https://www.rtl-sdr.com/new-product-rtl-sdr-blog-1090-mhz-ads-b-lna
                        self.isBiasTeeSupported = True
        except FileNotFoundError:
            print('DUMP1090 not installed.')

    def __startDump1090(self):
        # dump1090 --device-index <DEVICE_IDX> --net --net-sbs-port <SBS_PORT> --enable-bias-tee
        cmds = [self.param['DUMP1090_BIN']]
        cmds.append('--device-index')
        cmds.append(str(self.param['DEVICE_IDX']))
        cmds.append('--net')
        cmds.append('--net-sbs-port')
        cmds.append(str(self.param['SBS_PORT']))
        if self.isBiasTeeSupported and self.param['BIAS_TEE']:
            cmds.append('--enable-bias-tee')
        self.receiverProcess = subprocess.Popen(cmds,
                                                stdout = subprocess.DEVNULL,
                                                stderr = subprocess.DEVNULL,
                                                cwd = os.path.dirname(self.param['DUMP1090_BIN']))

    def getConfigurationParameterKey(self):
        return 'DUMP1090SBS1_LOCAL'

    def cleanupTask(self):
        super().cleanupTask()
        if self.receiverProcess != None:
            self.receiverProcess.kill()
