'''
The flight map implementation.
'''
import math
import os
import sys

from PyQt5.QtCore import (QAbstractListModel, QByteArray, QModelIndex, QSize,
                          Qt, QUrl, QVariant, pyqtSignal, pyqtSlot)
from PyQt5.QtPositioning import QGeoCoordinate
from PyQt5.QtQml import qmlRegisterType
from PyQt5.QtQuick import QQuickItem, QQuickView
from PyQt5.QtWidgets import QApplication, QSplitter, QWidget, QPushButton, QHBoxLayout, QVBoxLayout, QMessageBox

from waypoint import Waypoint, WaypointEditWindowFactory, WaypointList, MAVWaypointParameter
from pymavlink.dialects.v10 import common as mavlink

LATITUDE = -36.88
LONGITUDE = 174.75
ZOOM = 15
MIN_ZOOM = 0
MAX_ZOOM = 19
MIN_DRAG_TIME = 0.1
TILE_SIZE = 256

class MapItem(QQuickItem):

    updateCoordinate = pyqtSignal(float, float, arguments=['lat', 'lng'])
    updateHomeCoordinate = pyqtSignal(float, float, arguments=['lat', 'lng'])
    updateZoomLevel = pyqtSignal(int, arguments=['zoom'])
    updateDroneLocation = pyqtSignal(float, float, float, float, arguments=['lat', 'lng', 'hacc', 'vacc'])

    waypointRemoved = pyqtSignal(int, arguments=['wpNumber'])  # signal sent to qml to remove wp in polyline, wpNumber starts from 1 (0 is resvered for home)
    waypointChanged = pyqtSignal(int, float, float, arguments=['wpNumber', 'latitude', 'longitude'])  # signal sent to qml to update wp in polyline
    waypointCreated = pyqtSignal(float, float, arguments=['lat', 'lng'])
    allPolylineRemoved = pyqtSignal()  # signal to remove existing polyline

    def __init__(self, parent=None):
        super().__init__(parent)
        self.lat = 0.0
        self.lng = 0.0
        self.zoomLevel = 5

    def moveMapToCoordinate(self, lat, lng):
        self.lat = lat
        self.lng = lng
        self.updateCoordinate.emit(lat, lng)

    def moveHomeToCoordinate(self, lat, lng):
        self.updateHomeCoordinate.emit(lat, lng)

    def getZoomLevel(self):
        return self.zoomLevel

    def zoomMap(self, delta):
        zoom = self.zoomLevel + delta
        if MIN_ZOOM <= zoom <= MAX_ZOOM:
            self.zoomLevel = zoom
            self.updateZoomLevel.emit(self.zoomLevel)

    def moveMap(self, deltaLat, deltaLng):
        lat = self.lat + deltaLat
        lng = self.lng + deltaLng
        if -90.0 <= lat <= 90.0 and 0.0 <= lng <= 180.0:
            self.moveMapToCoordinate(lat, lng)

    def moveDroneLocation(self, lat, lng, hacc, vacc):
        self.updateDroneLocation.emit(lat, lng, hacc, vacc)

class WaypointsModel(QAbstractListModel):

    createWaypointAction = pyqtSignal(object)

    allWaypoints = [] # [Waypoint(0, 0, 0, 0)]
    redWPIdx = -1

    def __init__(self, parent = None):
        super().__init__(parent)
        self.positionRole = Qt.UserRole + 1
        self.dotColorRole = Qt.UserRole + 2
        self.rowNumberRole = Qt.UserRole + 3
        self.loiterRadiusRole = Qt.UserRole + 4

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
        return len(self.allWaypoints)

    def data(self, index, role=Qt.DisplayRole):
        idx = index.row()
        if idx < 0 or idx > len(self.allWaypoints) - 1:
            return QVariant()
        if role == self.positionRole:
            return QVariant(self.allWaypoints[idx].getCoordinate())
        if role == self.dotColorRole:
            if self.allWaypoints[idx].rowNumber == self.redWPIdx:
                self.redWPIdx = -1
                return QVariant('red')
            return QVariant('green')
        if role == self.rowNumberRole:
            return QVariant(self.allWaypoints[idx].rowNumber)
        if role == self.loiterRadiusRole:
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
            self.positionRole : QByteArray(b'position'),
            self.dotColorRole : QByteArray(b'dotColor'),
            self.rowNumberRole : QByteArray(b'rowNumber'),
            self.loiterRadiusRole : QByteArray(b'loiterRadius')
        }

    def updateWaypointCoordinate(self, rowNumber, newCoordinate: QGeoCoordinate):
        if 0 <= rowNumber < len(self.allWaypoints):
            # self._debug_dump_wp_list(rowNumber)
            wp = self.allWaypoints[rowNumber]
            print('[MAP] move WP#{4} ({0}, {1}) to ({2}, {3})'.format(wp.latitude, wp.longitude,
                                                            newCoordinate.latitude(), newCoordinate.longitude(),
                                                            wp.rowNumber))
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

class MapView(QQuickView):

    selectWaypointForAction = pyqtSignal(object)
    updateWaypointCoordinateEvent = pyqtSignal(int, object)  # WP row#, new coordinate
    moveHomeEvent = pyqtSignal(object)

    dragStart = False
    map = None

    def __init__(self, qml):
        super().__init__()
        qmlRegisterType(MapItem, 'MapItem', 1, 0, 'MapItem')
        self.setResizeMode(QQuickView.SizeRootObjectToView)
        self.wpModel = WaypointsModel()
        self.rootContext().setContextProperty('markerModel', self.wpModel)
        self.setSource(qml)
        if self.status() == QQuickView.Error:
            print('error loading qml file')
        else:
            self.map = self.rootObject()
            self.map.waypointSelected.connect(self.waypointEditEvent)
            self.map.mapDragEvent.connect(self.mapDragEvent)
            self.map.updateHomeLocation.connect(self.updateHomeEvent)

    def restorePreviousView(self):
        if self.map is not None:
            # TODO load previous coordinate  and zoom level from user configuration file
            self.map.moveMapToCoordinate(LATITUDE, LONGITUDE)
            self.map.zoomMap(6)

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
        self.map.zoomMap(event.angleDelta().y() / 120)

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
        elif key == Qt.Key_Home:
            # TODO update home location
            self.map.moveMapToCoordinate(LATITUDE, LONGITUDE)

    def waypointEditEvent(self, index):
        if 0 <= index < self.wpModel.rowCount():
            # print('select WP#', index)
            self.selectWaypointForAction.emit(self.wpModel.allWaypoints[index])

    def mapDragEvent(self, index, lat, lng, actType):
        '''
        actType = 0, start of drag; actType = 1, end of drag
        '''
        if actType == 0:
            # something to do on start of a drag
            self.dragStart = True
        elif actType == 1:
            if self.dragStart:
                toWp = QGeoCoordinate(lat, lng)
                self.updateWaypointCoordinateEvent.emit(index, toWp)
                self.dragStart = False

    def updateHomeEvent(self, lat, lng):
        # print('New home location: {0}, {1}'.format(lat, lng))
        self.moveHomeEvent.emit(QGeoCoordinate(lat, lng))

    def updateDroneLocation(self, lat, lng, hacc, vacc):
        # print('Drone location updated: {}, {}'.format(lat, lng))
        self.map.moveDroneLocation(lat, lng, hacc, vacc)

    def minimumSize(self):
        return QSize(600, 480)

class MapWidget(QSplitter):

    mapView = None
    waypointList = None
    horizonLine = -1
    defaultLatitude = 30.0
    editWPpopup = []

    uploadWaypointsToUAVEvent = pyqtSignal(object)  # pass the waypoint list as parameter
    downloadWaypointsFromUAVSignal = pyqtSignal()

    def __init__(self, mapQmlFile, parent = None):
        super().__init__(Qt.Vertical, parent)
        self.mapView = MapView(QUrl.fromLocalFile(mapQmlFile))
        self.waypointList = WaypointList(self.mapView.wpModel.allWaypoints, parent)

        container = QWidget.createWindowContainer(self.mapView)
        container.setMinimumSize(self.mapView.minimumSize())
        container.setMaximumSize(self.mapView.maximumSize())
        container.setFocusPolicy(Qt.TabFocus)
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
        panelLayput = QHBoxLayout()
        panelLayput.setSpacing(0)
        panelLayput.addStretch(1)
        self.loadWaypoints.clicked.connect(self.loadWaypointsFromUAV)
        self.uploadWaypoints.clicked.connect(self.uploadWaypointsToUAV)
        panelLayput.addWidget(self.loadWaypoints, 0, Qt.AlignRight)
        panelLayput.addWidget(self.uploadWaypoints, 0, Qt.AlignRight)
        self.actionPanel.setLayout(panelLayput)

        self.lowerPanel = QWidget(self)  # WP list and action panel
        lowerLayout = QVBoxLayout()
        lowerLayout.setSpacing(0)
        lowerLayout.addWidget(self.waypointList)
        lowerLayout.addWidget(self.actionPanel)
        self.lowerPanel.setLayout(lowerLayout)

        self.addWidget(container)
        self.addWidget(self.lowerPanel)

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
    w = MapWidget(qmlFile)
    w.show()
    sys.exit(app.exec_())
