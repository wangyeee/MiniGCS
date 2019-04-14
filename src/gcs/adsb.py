import socket
import time

from PyQt5.QtPositioning import QGeoCoordinate
from PyQt5.QtCore import (QAbstractListModel, QByteArray, QModelIndex, QSize,
                          Qt, QUrl, QVariant, pyqtSignal, pyqtSlot, QThread)

# https://github.com/kanflo/ADS-B-funhouse
from sbs1 import SBS1Message

class Aircraft:

    icao24 = None
    loggedDate = None
    callsign = None
    altitude = -123.0
    groundSpeed = -1.0
    track = None
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
            if self.has3DPositionUpdate():
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

    def data(self, index, role=Qt.DisplayRole):
        idx = index.row()
        if idx < 0 or idx > len(self.allAircrafts) - 1:
            return QVariant()
        if role == self.positionRole:
            return QVariant(self.allAircrafts[idx].getCoordinate())
        if role == self.headingRole:
            return QVariant('green')
        if role == self.callsignRole:
            if self.allAircrafts[idx].callsign != None:
                return QVariant(self.allAircrafts[idx].callsign)
            return QVariant('x{}'.format(self.allAircrafts[idx].icao24))
        return QVariant()

    def flags(self, index):
        return Qt.NoItemFlags

    def roleNames(self):
        return {
            self.positionRole : QByteArray(b'position'),
            self.headingRole : QByteArray(b'headingRole'),
            self.callsignRole : QByteArray(b'callsignRole')
        }

# test codes
if __name__ == '__main__':
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect(('localhost', 30003))

    MSGLEN = 100
    dbg_cnt = 0
    running = True
    aircrafts = {}

    while running:
        line = s.recv(1024)
        if line == b'':
            raise RuntimeError('socket connection broken')
        # print(line.decode('ascii'), end='')
        msg = SBS1Message(line)
        if msg.isValid:
            # msg.dump()
            if msg.icao24 not in aircrafts:
                aircrafts[msg.icao24] = Aircraft(msg.icao24)
            aircrafts[msg.icao24].update(msg)

        dbg_cnt += 1
        if dbg_cnt == MSGLEN:
            print('{} msg received, exit.'.format(MSGLEN))
            running = False

    s.close()
