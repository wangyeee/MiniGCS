'''
The flight map implementation.
'''
import math
import os
import sys
import time

from pymavlink.dialects.v10 import common as mavlink
from PyQt5.QtCore import (QAbstractListModel, QByteArray, QModelIndex, QSize,
                          Qt, QThread, QUrl, QVariant, pyqtSignal, pyqtSlot)
from PyQt5.QtGui import QCursor
from PyQt5.QtPositioning import QGeoCoordinate
from PyQt5.QtQml import qmlRegisterType
from PyQt5.QtQuick import QQuickItem, QQuickView
from PyQt5.QtWidgets import (QApplication, QHBoxLayout, QLabel, QMessageBox,
                             QPushButton, QSplitter, QVBoxLayout, QWidget)

from adsb import ADSBSource, AircraftsModel
from telemetry import UD_TELEMETRY_KEY, UD_TELEMETRY_LOG_FOLDER_KEY
from UserData import UserData
from utils import unused
from waypoint import (MAVWaypointParameter, Waypoint,
                      WaypointEditWindowFactory, WaypointList)

DEFAULT_LATITUDE = 0.0
DEFAULT_LONGITUDE = 0.0
DEFAULT_ZOOM = 1

MIN_ZOOM = 0
MAX_ZOOM = 19
MIN_DRAG_TIME = 0.1
TILE_SIZE = 256

UD_MAP_KEY = 'MAP'
UD_MAP_INIT_LATITUDE_KEY = 'INIT_LATITUDE'
UD_MAP_INIT_LONGITUDE_KEY = 'INIT_LONGITUDE'
UD_MAP_INIT_ZOOM_KEY = 'INIT_ZOOM'

class MapItem(QQuickItem):

    updateCoordinate = pyqtSignal(float, float, arguments=['lat', 'lng'])
    updateHomeCoordinate = pyqtSignal(float, float, arguments=['lat', 'lng'])
    updateZoomLevel = pyqtSignal(int, arguments=['zoom'])
    updateDroneLocation = pyqtSignal(float, float, arguments=['lat', 'lng'])
    updateDroneLocationUncertainty = pyqtSignal(float, float, arguments=['hacc', 'vacc'])

    waypointRemoved = pyqtSignal(int, arguments=['wpNumber'])  # signal sent to qml to remove wp in polyline, wpNumber starts from 1 (0 is resvered for home)
    waypointChanged = pyqtSignal(int, float, float, arguments=['wpNumber', 'latitude', 'longitude'])  # signal sent to qml to update wp in polyline
    waypointChangedInt = pyqtSignal(int, int, int, arguments=['wpNumber', 'x', 'y'])  # signal sent to qml to update wp by cursor position in polyline
    waypointCreated = pyqtSignal(float, float, arguments=['lat', 'lng'])
    allPolylineRemoved = pyqtSignal()  # signal to remove existing polyline

    def __init__(self, parent=None):
        super().__init__(parent)
        self.lat = 0.0
        self.lng = 0.0
        self.zoomLevel = DEFAULT_ZOOM

    def moveMapToCoordinate(self, lat, lng):
        self.lat = lat
        self.lng = lng
        self.updateCoordinate.emit(lat, lng)

    def moveHomeToCoordinate(self, lat, lng):
        self.updateHomeCoordinate.emit(lat, lng)

    def getZoomLevel(self):
        return self.zoomLevel

    def zoomMapDelta(self, delta):
        self.zoomMap(self.zoomLevel + delta)

    def zoomMap(self, absZoom):
        if MIN_ZOOM <= absZoom <= MAX_ZOOM:
            self.zoomLevel = absZoom
            self.updateZoomLevel.emit(self.zoomLevel)

    def moveMap(self, deltaLat, deltaLng):
        lat = self.lat + deltaLat
        lng = self.lng + deltaLng
        if -90.0 <= lat <= 90.0 and -180.0 <= lng <= 180.0:
            self.moveMapToCoordinate(lat, lng)

    def moveDroneLocation(self, lat, lng, hacc, vacc):
        self.updateDroneLocation.emit(lat, lng, hacc, vacc)

class WaypointsModel(QAbstractListModel):

    positionRole = Qt.UserRole + 1
    dotColorRole = Qt.UserRole + 2
    rowNumberRole = Qt.UserRole + 3
    loiterRadiusRole = Qt.UserRole + 4
    createWaypointAction = pyqtSignal(object)

    def __init__(self, parent = None):
        super().__init__(parent)
        self.allWaypoints = [] # [Waypoint(0, 0, 0, 0)]
        self.redWPIdx = -1

    def markWaypoint(self, wp: Waypoint):
        idx = self.index(wp.rowNumber)
        self.redWPIdx = wp.rowNumber
        self.dataChanged.emit(idx, idx)

    def unmarkWaypoint(self, wp: Waypoint):
        idx = self.index(wp.rowNumber)
        self.redWPIdx = -1
        self.dataChanged.emit(idx, idx)

    @pyqtSlot(float, float)
    def createWaypoint(self, lat, lng):
        idx = self.rowCount()
        self.beginInsertRows(QModelIndex(), idx, idx)
        mrk = Waypoint(idx, lat, lng, 10.0)
        self.allWaypoints.append(mrk)
        self.endInsertRows()
        self.createWaypointAction.emit(mrk)

    def rowCount(self, parent=QModelIndex()):
        unused(parent)
        return len(self.allWaypoints)

    def data(self, index, role=Qt.DisplayRole):
        idx = index.row()
        if idx < 0 or idx > len(self.allWaypoints) - 1:
            return QVariant()
        if role == WaypointsModel.positionRole:
            return QVariant(self.allWaypoints[idx].getCoordinate())
        if role == WaypointsModel.dotColorRole:
            if self.allWaypoints[idx].rowNumber == self.redWPIdx:
                self.redWPIdx = -1
                return QVariant('red')
            return QVariant('green')
        if role == WaypointsModel.rowNumberRole:
            return QVariant(self.allWaypoints[idx].rowNumber)
        if role == WaypointsModel.loiterRadiusRole:
            if self.allWaypoints[idx].waypointType in (mavlink.MAV_CMD_NAV_LOITER_TIME, mavlink.MAV_CMD_NAV_LOITER_TURNS,
                                                       mavlink.MAV_CMD_NAV_LOITER_UNLIM, mavlink.MAV_CMD_NAV_LOITER_TO_ALT):
                return QVariant(self.allWaypoints[idx].mavlinkParameters[MAVWaypointParameter.PARAM3])
            return QVariant(0.0)
        return QVariant()

    def flags(self, index):
        if index.row() < 0 or index.row() >= len(self.allWaypoints):
            return Qt.NoItemFlags
        return Qt.ItemIsDragEnabled | Qt.ItemIsDropEnabled

    def roleNames(self):
        return {
            WaypointsModel.positionRole : QByteArray(b'position'),
            WaypointsModel.dotColorRole : QByteArray(b'dotColor'),
            WaypointsModel.rowNumberRole : QByteArray(b'rowNumber'),
            WaypointsModel.loiterRadiusRole : QByteArray(b'loiterRadius')
        }

    def updateWaypointCoordinate(self, rowNumber, newCoordinate: QGeoCoordinate):
        if 0 <= rowNumber < len(self.allWaypoints):
            # self._debug_dump_wp_list(rowNumber)
            wp = self.allWaypoints[rowNumber]
            # print('[MAP] move WP#{4} ({0}, {1}) to ({2}, {3})'.format(wp.latitude, wp.longitude,
            #                                                 newCoordinate.latitude(), newCoordinate.longitude(),
            #                                                 wp.rowNumber))
            wp.latitude = newCoordinate.latitude()
            wp.longitude = newCoordinate.longitude()
            self.refreshWaypoint(wp)
            # self._debug_dump_wp_list(rowNumber)

    def refreshWaypoint(self, wp: Waypoint):
        idx = self.index(wp.rowNumber)
        self.dataChanged.emit(idx, idx)

    def removeWaypoint(self, wp: Waypoint):
        tgtWp = wp.rowNumber
        # self._debug_dump_wp_list(tgtWp)
        i = tgtWp + 1
        while i < len(self.allWaypoints):
            self.allWaypoints[i].rowNumber -= 1
            i += 1
        self.beginRemoveRows(QModelIndex(), tgtWp, tgtWp)
        del self.allWaypoints[tgtWp]
        self.endRemoveRows()
        # self._debug_dump_wp_list()
        # send signal to update WP# displayed on the map unless the last WP is removed
        if i > tgtWp + 1:
            txtStart = self.index(tgtWp)
            txtEnd = self.index(len(self.allWaypoints) - 1)
            self.dataChanged.emit(txtStart, txtEnd)

    def removeAllWaypoint(self):
        self.beginRemoveRows(QModelIndex(), 0, self.rowCount())
        self.allWaypoints.clear()
        self.endRemoveRows()

    def _debug_dump_wp_list(self, star = -1):
        for wp in self.allWaypoints:
            if star == wp.rowNumber:
                print('*' + str(wp))
            else:
                print(str(wp))

class WaypointDragTracking(QThread):

    def __init__(self, mapView, hertz = 50, threshold = 2, parent = None):
        super().__init__(parent)
        self.wpIndex = 0
        self.prevX = 0
        self.prevY = 0
        self.mapView = mapView
        self.delay = 1 / hertz
        self.threshold = threshold

    def startTrackingWaypoint(self, index):
        self.wpIndex = index
        self.start()

    def run(self):
        while self.mapView.isBeingDragged():
            currentPos = QCursor.pos()
            if abs(currentPos.x() - self.prevX) > self.threshold or abs(currentPos.y() - self.prevY) > self.threshold:
                self.prevX = currentPos.x()
                self.prevY = currentPos.y()
                self.mapView.map.waypointChangedInt.emit(self.wpIndex, currentPos.x(), currentPos.y())  # Update ploylines in the map and the list
            time.sleep(self.delay)

class MapView(QQuickView):

    selectWaypointForAction = pyqtSignal(object)
    updateWaypointCoordinateEvent = pyqtSignal(int, object)  # WP row#, new coordinate
    moveHomeEvent = pyqtSignal(object)

    def __init__(self, qml):
        super().__init__()
        self.dragStart = False
        self.dragTracker = None
        self.mapConf = None
        qmlRegisterType(MapItem, 'MapItem', 1, 0, 'MapItem')
        self.setResizeMode(QQuickView.SizeRootObjectToView)
        self.wpModel = WaypointsModel()
        self.adsbModel = AircraftsModel()
        self.rootContext().setContextProperty('markerModel', self.wpModel)
        self.rootContext().setContextProperty('adsbModel', self.adsbModel)
        self.setSource(qml)
        if self.status() == QQuickView.Error:
            print('error loading qml file')
        else:
            self.map = self.rootObject()
            self.map.waypointSelected.connect(self.waypointEditEvent)
            self.map.mapDragEvent.connect(self.mapDragEvent)
            self.map.mapCenterChangedEvent.connect(self.mapCenterChangedEvent)
            self.map.mapZoomLevelChangedEvent.connect(self.mapZoomLevelChangedEvent)
            self.map.updateHomeLocation.connect(self.updateHomeEvent)
            self.dragTracker = WaypointDragTracking(self)
            self.adsbSources = ADSBSource.getAvailableADSBSources()
            for src in self.adsbSources:
                ud = UserData.getInstance()
                param = ud.getUserDataEntry(src.getConfigurationParameterKey())
                if param != None and param['ENABLE']:
                    src.lazyInit(param)
                    src.aircraftCreateSignal.connect(self.adsbModel.addAircraft)
                    src.aircraftUpdateSignal.connect(self.adsbModel.updateAircraft)
                    src.aircraftDeleteSignal.connect(self.adsbModel.removeAircraft)
                    src.start()

    def restorePreviousView(self):
        if self.map != None:
            lat0 = DEFAULT_LATITUDE
            lng0 = DEFAULT_LONGITUDE
            zoom0 = DEFAULT_ZOOM
            try:
                self.mapConf = UserData.getInstance().getUserDataEntry(UD_MAP_KEY)
                if self.mapConf != None:
                    lat0 = float(self.mapConf[UD_MAP_INIT_LATITUDE_KEY])
                    lng0 = float(self.mapConf[UD_MAP_INIT_LONGITUDE_KEY])
                    zoom0 = int(self.mapConf[UD_MAP_INIT_ZOOM_KEY])
                else:
                    self.mapConf = {}
            except ValueError:
                pass
            except TypeError:
                pass
            self.map.moveMapToCoordinate(lat0, lng0)
            self.map.zoomMap(zoom0)

    def pix2lat(self, pix, piy):
        scale = 1 << int(self.map.getZoomLevel())
        scale *= TILE_SIZE
        lng = 360 * pix / scale
        m = 4 * math.pi * piy / scale
        em = math.pow(math.e, m)
        lat = math.asin((em - 1) / (em + 1)) * 180 / math.pi
        return lng, lat

    def wheelEvent(self, event):
        # quicker response compared to MapGestureArea.FlickGesture
        self.map.zoomMapDelta(event.angleDelta().y() / 120)

    def keyPressEvent(self, event):
        key = event.key()
        deltaX, deltaY = self.pix2lat(10, 10)
        # print(key)
        if key == Qt.Key_Left:
            self.map.moveMap(0, -deltaY)
        elif key == Qt.Key_Right:
            self.map.moveMap(0, deltaY)
        elif key == Qt.Key_Up:
            self.map.moveMap(deltaX, 0)
        elif key == Qt.Key_Down:
            self.map.moveMap(-deltaX, 0)
        # elif key == Qt.Key_Home:
        # self.map.moveMapToCoordinate(LATITUDE, LONGITUDE)

    def waypointEditEvent(self, index):
        if 0 <= index < self.wpModel.rowCount():
            # print('select WP#', index)
            self.selectWaypointForAction.emit(self.wpModel.allWaypoints[index])

    def mapCenterChangedEvent(self, lat, lng):
        self.mapConf[UD_MAP_INIT_LATITUDE_KEY] = str(lat)
        self.mapConf[UD_MAP_INIT_LONGITUDE_KEY] = str(lng)
        UserData.getInstance().setUserDataEntry(UD_MAP_KEY, self.mapConf)

    def mapZoomLevelChangedEvent(self, zoom):
        self.mapConf[UD_MAP_INIT_ZOOM_KEY] = str(zoom)
        UserData.getInstance().setUserDataEntry(UD_MAP_KEY, self.mapConf)

    def mapDragEvent(self, index, lat, lng, actType):
        '''
        actType = 0, start of drag
        actType = 1, during drag
        actType = 2, end of drag
        '''
        if actType == 0:
            self.dragStart = True
            self.dragTracker.startTrackingWaypoint(index)
        elif actType == 1:
            toWp = QGeoCoordinate(lat, lng)
            self.updateWaypointCoordinateEvent.emit(index, toWp)
        elif actType == 2:
            if self.dragStart:
                toWp = QGeoCoordinate(lat, lng)
                self.updateWaypointCoordinateEvent.emit(index, toWp)
                self.dragStart = False

    def isBeingDragged(self):
        return self.dragStart

    def updateHomeEvent(self, lat, lng):
        # print('New home location: {0}, {1}'.format(lat, lng))
        self.moveHomeEvent.emit(QGeoCoordinate(lat, lng))

    def updateDroneLocation(self, sourceUAS, timestamp, latitude, longitude, altitude):
        unused(sourceUAS, timestamp, altitude)
        self.map.updateDroneLocation.emit(latitude, longitude)

    def updateDroneLocationUncertainty(self, sourceUAS, timestamp, fixtype, hdop, vdop, nosv, hacc, vacc, velacc, hdgacc):
        unused(sourceUAS, timestamp, fixtype, hdop, vdop, nosv, velacc, hdgacc)
        self.map.updateDroneLocationUncertainty.emit(hacc, vacc)

    def minimumSize(self):
        return QSize(600, 480)

class MapWidget(QSplitter):

    uploadWaypointsToUAVEvent = pyqtSignal(object)  # pass the waypoint list as parameter
    downloadWaypointsFromUAVSignal = pyqtSignal()

    def __init__(self, mapQmlFile, parent = None):
        super().__init__(Qt.Vertical, parent)
        self.mapView = MapView(QUrl.fromLocalFile(mapQmlFile))
        self.waypointList = WaypointList(self.mapView.wpModel.allWaypoints, parent)
        self.uas = None
        self.horizonLine = -1
        self.defaultLatitude = 30.0
        self.editWPpopup = []
        self.textMessageLogFile = None  # log of messages displayed on messageLabel

        container = QWidget.createWindowContainer(self.mapView)
        container.setMinimumSize(self.mapView.minimumSize())
        container.setMaximumSize(self.mapView.maximumSize())
        container.setFocusPolicy(Qt.TabFocus)
        container.sizePolicy().setVerticalStretch(3)
        self.mapView.restorePreviousView()
        self.mapView.wpModel.createWaypointAction.connect(self.createWaypointEvent)
        self.mapView.updateWaypointCoordinateEvent.connect(self.moveWaypointEvent)
        self.mapView.moveHomeEvent.connect(self.updateHomeLocationEvent)
        self.mapView.selectWaypointForAction.connect(self.waypointList.highlightWaypoint)
        self.waypointList.editWaypoint.connect(self.showEditWaypointWindow)
        self.waypointList.deleteWaypoint.connect(self.removeWaypoint)
        self.waypointList.preDeleteWaypoint.connect(self.markWaypointForRemoval)
        self.waypointList.cancelDeleteWaypoint.connect(self.unmarkWaypointNoRemoval)
        self.waypointList.afterWaypointEdited.connect(self.acceptWaypointEdit)

        self.actionPanel = QWidget(self)  # Upload/Refresh buttons
        self.loadWaypoints = QPushButton('Load from UAV')
        self.uploadWaypoints = QPushButton('Upload to UAV')
        self.messageLabel = QLabel('Disconnected')
        self.__setupTextMessageLogging()
        self.loadWaypoints.clicked.connect(self.loadWaypointsFromUAV)
        self.uploadWaypoints.clicked.connect(self.uploadWaypointsToUAV)
        panelLayput = QHBoxLayout()
        panelLayput.setSpacing(0)
        panelLayput.addWidget(self.messageLabel, 0, Qt.AlignLeft)
        panelLayput.addStretch(1)
        panelLayput.addWidget(self.loadWaypoints, 0, Qt.AlignRight)
        panelLayput.addWidget(self.uploadWaypoints, 0, Qt.AlignRight)
        self.actionPanel.setLayout(panelLayput)

        self.lowerPanel = QWidget(self)  # WP list and action panel
        lowerLayout = QVBoxLayout()
        lowerLayout.setSpacing(0)
        lowerLayout.addWidget(self.waypointList)
        lowerLayout.addWidget(self.actionPanel)
        self.lowerPanel.setLayout(lowerLayout)
        self.lowerPanel.sizePolicy().setVerticalStretch(1)

        self.addWidget(container)
        self.addWidget(self.lowerPanel)

    def setActiveUAS(self, uas):
        uas.updateGlobalPositionSignal.connect(self.mapView.updateDroneLocation)
        uas.updateGPSStatusSignal.connect(self.mapView.updateDroneLocationUncertainty)
        self.uas = uas

    def __setupTextMessageLogging(self):
        tconf = UserData.getInstance().getUserDataEntry(UD_TELEMETRY_KEY)
        if tconf != None:
            name = 'MAV_{}.log'.format(int(time.time() * 1000))
            self.textMessageLogFile = open(os.path.join(tconf[UD_TELEMETRY_LOG_FOLDER_KEY], name), 'w')

    def displayTextMessage(self, msg):
        if self.textMessageLogFile != None:
            self.textMessageLogFile.write('[{}] {}\n'.format(time.strftime('%d %b %Y %H:%M:%S', time.localtime()), msg))
        self.messageLabel.setText(msg)

    def uploadWaypointsToUAV(self):
        cfm = QMessageBox.question(self.window(),
                                   'Confirm upload',
                                   'Are you sure to send waypoints to UAV? All onboard waypoints will be replaced.',
                                   QMessageBox.Yes, QMessageBox.No)
        if cfm == QMessageBox.Yes:
            self.uploadWaypointsToUAVEvent.emit(self.mapView.wpModel.allWaypoints)

    def loadWaypointsFromUAV(self):
        cfm = QMessageBox.question(self.window(),
                                   'Confirm reload',
                                   'Are you sure to reload waypoints from UAV? All current waypoints will be replaced.',
                                   QMessageBox.Yes, QMessageBox.No)
        if cfm == QMessageBox.Yes:
            print('load WP from UAV')
            self.downloadWaypointsFromUAVSignal.emit()

    def moveWaypointEvent(self, wpIdx, toWp: QGeoCoordinate):
        # print('[WP] move {} to ({}, {})'.format(wpIdx, toWp.latitude(), toWp.longitude()))
        self.mapView.wpModel.updateWaypointCoordinate(wpIdx, toWp)
        self.mapView.map.waypointChanged.emit(wpIdx, toWp.latitude(), toWp.longitude())
        self.waypointList.moveWaypoint(wpIdx, toWp)

    def updateHomeLocationEvent(self, home: QGeoCoordinate):
        # print('update home loc: {}, {}'.format(home.latitude(), home.longitude()))
        self.mapView.map.moveHomeToCoordinate(home.latitude(), home.longitude())
        self.waypointList.updateHomeLocation(home)

    def showEditWaypointWindow(self, wp: Waypoint):
        # print('edit wp#{0}'.format(wp.rowNumber))
        popup = WaypointEditWindowFactory.createWaypointEditWindow(wp)
        self.editWPpopup.append(popup)
        popup.updateWaypoint.connect(self.acceptWaypointEdit)
        popup.show()

    def acceptWaypointEdit(self, wp: Waypoint):
        # print('edited WP:', wp)
        self.waypointList.updateWaypoint(wp)
        # sent new wp to map polyline
        self.mapView.wpModel.refreshWaypoint(wp)
        self.mapView.map.waypointChanged.emit(wp.rowNumber, wp.latitude, wp.longitude)

    def removeWaypoint(self, wp: Waypoint):
        # print('delete wp#{0}'.format(wp.rowNumber))
        self.mapView.wpModel.removeWaypoint(wp)
        self.mapView.map.waypointRemoved.emit(wp.rowNumber)

    def createWaypointEvent(self, wp: Waypoint):
        # print('[new] create WP: ({0}, {1}) at {2}'.format(wp.latitude, wp.longitude, wp.altitude))
        self.waypointList.addWaypoint(wp)

    def markWaypointForRemoval(self, wp: Waypoint):
        self.mapView.wpModel.markWaypoint(wp)

    def unmarkWaypointNoRemoval(self, wp: Waypoint):
        self.mapView.wpModel.unmarkWaypoint(wp)

    def setAllWaypoints(self, wpList):
        self.mapView.map.allPolylineRemoved.emit()
        self.waypointList.removeAllRows()
        self.mapView.wpModel.removeAllWaypoint()
        for wp in wpList:
            self.mapView.wpModel.createWaypoint(wp.latitude, wp.longitude)
            self.mapView.map.waypointCreated.emit(wp.latitude, wp.longitude)

#test only
if __name__ == "__main__":
    app = QApplication(sys.argv)
    current_path = os.path.abspath(os.path.dirname(__file__))
    qmlFile = os.path.join(current_path, 'map.qml')
    try:
        UserData.getInstance().loadGCSConfiguration()
    except IOError:
        sys.exit(1)
    w = MapWidget(qmlFile)
    w.show()
    sys.exit(app.exec_())
