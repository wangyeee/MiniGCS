import socket
import time

from PyQt5.QtPositioning import QGeoCoordinate
from PyQt5.QtCore import (QAbstractListModel, QByteArray, QModelIndex, QSize,
                          Qt, QUrl, QVariant, pyqtSignal, pyqtSlot, QThread)

# https://github.com/kanflo/ADS-B-funhouse
from sbs1 import SBS1Message

# TODO remove this class
class Aircraft0:
    icao24 = None
    loggedDate = None
    callsign = None
    altitude = -1.0
    groundSpeed = -1.0
    track = 0
    latitude = 0.0
    longitude = 0.0
    verticalRate = 0.0

    fieldUpdates = {}

    def __init__(self, icao24):
        self.icao24 = icao24
        self.fieldUpdates['CALLSIGN'] = False
        self.fieldUpdates['ALT'] = False
        self.fieldUpdates['GS'] = False
        self.fieldUpdates['VS'] = False
        self.fieldUpdates['TRK'] = False
        self.fieldUpdates['LAT'] = False
        self.fieldUpdates['LNG'] = False
        self.fieldUpdates['LUT'] = time.time()

    def has3DPositionUpdate(self):
        return self.__anyFields(['LAT', 'LNG', 'ALT'])

    def has2DPositionUpdate(self):
        return self.__anyFields(['LAT', 'LNG'])

    def has2DVelocityUpdate(self):
        return self.__anyFields(['GS', 'TRK'])

    def has3DVelocityUpdate(self):
        return self.__anyFields(['VS', 'GS', 'TRK'])

    def __anyFields(self, fields):
        for f in fields:
            if self.fieldUpdates[f] == False:
                return False
        return True

    def getCoordinate(self):
        if self.has2DPositionUpdate():
            return QGeoCoordinate(self.latitude, self.longitude)
        return None

    def update(self, msg: SBS1Message):
        if msg.icao24 and self.icao24 == msg.icao24:
            if msg.callsign:
                self.callsign = msg.callsign
                self.fieldUpdates['CALLSIGN'] = True
            if msg.altitude:
                self.altitude = msg.altitude
                self.fieldUpdates['ALT'] = True
            if msg.groundSpeed:
                self.groundSpeed = msg.groundSpeed
                self.fieldUpdates['GS'] = True
            if msg.track:
                self.track = msg.track
                self.fieldUpdates['TRK'] = True
            if msg.lat:
                self.latitude = msg.lat
                self.fieldUpdates['LAT'] = True
            if msg.lon:
                self.longitude = msg.lon
                self.fieldUpdates['LNG'] = True
            if msg.verticalRate:
                self.verticalRate = msg.verticalRate
                self.fieldUpdates['VS'] = True
            self.fieldUpdates['LUT'] = time.time()
            # if self.has2DPositionUpdate():
            print(str({
                'ICAO' : self.icao24,
                'CALLSIGN' : self.callsign,
                'ALT' : self.altitude,
                'GS' : self.groundSpeed,
                'VS' : self.verticalRate,
                'LAT' : self.latitude,
                'LNG' : self.longitude,
                'TRK' : self.track,
            }))

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
        if i > 0:
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
        if i > 0:
            self.beginRemoveRows(QModelIndex(), i, i)
            del self.allAircrafts[i]
            self.endRemoveRows()

    def data(self, index, role=Qt.DisplayRole):
        idx = index.row()
        if idx < 0 or idx > len(self.allAircrafts) - 1:
            return QVariant()
        if role == self.positionRole:
            if self.allAircrafts[idx].lat and self.allAircrafts[idx].lon:
                # print('Position update#{} at {} => {}, {}'.format(idx, self.allAircrafts[idx].loggedDate, self.allAircrafts[idx].lat, self.allAircrafts[idx].lon))
                return QVariant(QGeoCoordinate(self.allAircrafts[idx].lat, self.allAircrafts[idx].lon))
            return QVariant(QGeoCoordinate(1000, 1000))
        if role == self.headingRole:
            if self.allAircrafts[idx].lat and self.allAircrafts[idx].lon:
                if self.allAircrafts[idx].track:
                    # print('Heading update#{} => {}'.format(idx, self.allAircrafts[idx].track))
                    return QVariant(self.allAircrafts[idx].track)
            return QVariant(0)
        if role == self.callsignRole:
            if self.allAircrafts[idx].lat and self.allAircrafts[idx].lon:
                if self.allAircrafts[idx].callsign:
                    return QVariant(self.allAircrafts[idx].callsign)
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

    def run(self):
        while self.running:
            self.periodicalTask()
        self.cleanupTask()

    def stopADSB(self):
        self.running = False

    def periodicalTask(self):
        pass

    def cleanupTask(self):
        pass

    def getConfigurationParameterKey(self):
        return ''

    @staticmethod
    def getAvailableADSBSources():
        return [Dump1090NetClient()]

class Dump1090NetClient(ADSBSource):

    aircrafts = {}
    sckt = None
    timeout = 10  # default timeout, 10 seconds

    def __init__ (self, host = None, port = 0, parent = None):
        super().__init__(parent)
        self.lazyInit({'HOST' : host, 'PORT' : port})

    def lazyInit(self, param):
        host = param['HOST']
        port = int(param['PORT'])
        if host != None and 0 < port < 65536:
            self.sckt = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sckt.connect((host, port))
            try:
                self.timeout = int(param['TIMEOUT'])
            except ValueError:
                pass

    def periodicalTask(self):
        self.__receiveLatestData()
        self.__removeInactiveData()

    def __receiveLatestData(self):
        if self.sckt == None:
            return
        line = self.sckt.recv(1024)
        if line == b'':
            raise RuntimeError('socket connection broken')
        msg = SBS1Message(line)
        if msg.isValid:
            msg.loggedDate = time.time()  # re-use loggedDate field
            if msg.icao24 not in self.aircrafts:
                self.aircrafts[msg.icao24] = msg
                self.aircraftCreateSignal.emit(self.aircrafts[msg.icao24])
            else:
                self.__updateMessageFields(self.aircrafts[msg.icao24], msg)
                self.aircraftUpdateSignal.emit(self.aircrafts[msg.icao24])

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


# test codes
if __name__ == '__main__':
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect(('localhost', 30003))

    MSGLEN = 100
    dbg_cnt = 0
    running = False
    aircrafts = {}

    while running:
        line0 = s.recv(1024)
        if line0 == b'':
            raise RuntimeError('socket connection broken')
        # print(line0.decode('ascii'), end='')
        msg0 = SBS1Message(line0)
        if msg0.isValid:
            # msg.dump()
            if msg0.icao24 not in aircrafts:
                aircrafts[msg0.icao24] = Aircraft(msg0.icao24)
            aircrafts[msg0.icao24].update(msg0)

        dbg_cnt += 1
        if dbg_cnt == MSGLEN:
            print('{} msg received, exit.'.format(MSGLEN))
            running = False

    s.close()
