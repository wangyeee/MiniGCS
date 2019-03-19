'''
The flight map implementation.
'''
import math
import os
import sys
# import time

from PyQt5.QtCore import Qt, QUrl, pyqtSignal, QAbstractListModel, QModelIndex, QVariant, pyqtSlot, QByteArray, QSize
from PyQt5.QtGui import QMouseEvent
from PyQt5.QtQml import qmlRegisterType
from PyQt5.QtQuick import QQuickItem, QQuickView
from PyQt5.QtPositioning import QGeoCoordinate
from PyQt5.QtWidgets import QVBoxLayout, QWidget, QApplication

from waypoint import WaypointList, Waypoint, WaypointEditWindow

LATITUDE = -36.88
LONGITUDE = 174.75
ZOOM = 15
MIN_ZOOM = 0
MAX_ZOOM = 19
MIN_DRAG_TIME = 0.1
TILE_SIZE = 256

class MapItem(QQuickItem):

    updateCoordinate = pyqtSignal(float, float, arguments=['lat', 'lng'])
    updateZoomLevel = pyqtSignal(int, arguments=['zoom'])
    updateDroneLocation = pyqtSignal(float, float, float, float, arguments=['lat', 'lng', 'hacc', 'vacc'])

    waypointRemoved = pyqtSignal(int, arguments=['wpNumber'])  # signal sent to qml to remove wp in polyline
    waypointChanged = pyqtSignal(int, float, float, arguments=['wpNumber', 'latitude', 'longitude'])  # signal sent to qml to update wp in polyline

    def __init__(self, parent=None):
        super(MapItem, self).__init__(parent)
        self.lat = 0.0
        self.lng = 0.0
        self.zoomLevel = 5

    def moveMapToCoordinate(self, lat, lng):
        self.lat = lat
        self.lng = lng
        self.updateCoordinate.emit(lat, lng)

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
    wpCoordRev = {}
    wpSequence = 1
    redWPIdx = -1

    def __init__(self, parent = None):
        super().__init__(parent)
        self.positionRole = Qt.UserRole + 1
        self.dotColorRole = Qt.UserRole + 2
        self.rowNumberRole = Qt.UserRole + 3

    def markWaypoint(self, wp: Waypoint):
        idx = self.index(wp.rowNumber)
        self.redWPIdx = wp.wpID
        self.dataChanged.emit(idx, idx)

    def unmarkWaypoint(self, wp: Waypoint):
        idx = self.index(wp.rowNumber)
        self.redWPIdx = -1
        self.dataChanged.emit(idx, idx)

    @pyqtSlot(float, float)
    def createWaypoint(self, lat, lng):
        self.beginInsertRows(QModelIndex(), self.rowCount(), self.rowCount())
        seq = self.wpSequence
        self.wpSequence += 1
        mrk = Waypoint(seq, lat, lng, 10.0)
        mrk.rowNumber = self.rowCount()
        # print('add marker#{0} on {1}, {2}, row: {3}'.format(mrk.rowNumber, lat, lng, seq))
        self.allWaypoints.append(mrk)
        self.wpCoordRev[QGeoCoordinate(lat, lng)] = mrk
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
            if self.allWaypoints[idx].wpID == self.redWPIdx:
                return QVariant('red')
            return QVariant('green')
        if role == self.rowNumberRole:
            return QVariant(self.allWaypoints[idx].rowNumber)
        return QVariant()

    def flags(self, index):
        if index.row() < 0 or index.row() >= len(self.allWaypoints):
            return Qt.NoItemFlags
        return Qt.ItemIsDragEnabled | Qt.ItemIsDropEnabled

    def roleNames(self):
        return {
            self.positionRole : QByteArray(b'position'),
            self.dotColorRole : QByteArray(b'dotColor'),
            self.rowNumberRole : QByteArray(b'rowNumber')
        }

    def updateWaypointCoordinate(self, oldCoordinate: QGeoCoordinate, newCoordinate: QGeoCoordinate):
        wp = self.wpCoordRev[oldCoordinate]
        if wp != None:
            print('[MAP] move WP#{4} ({0}, {1}) to ({2}, {3})'.format(oldCoordinate.latitude(), oldCoordinate.longitude(),
                                                               newCoordinate.latitude(), newCoordinate.longitude(),
                                                               wp.rowNumber))
            wp.latitude = newCoordinate.latitude()
            wp.longitude = newCoordinate.longitude()
            del self.wpCoordRev[oldCoordinate]
            self.wpCoordRev[newCoordinate] = wp
            return wp.rowNumber
        return -1

    def refreshWaypoint(self, wp: Waypoint):
        oldWp = self.allWaypoints[wp.rowNumber]
        idx = self.index(wp.rowNumber)
        oldCord = oldWp.getCoordinate()
        # remove coordinate -> WP map entry
        del self.wpCoordRev[oldCord]
        # replace with new WP
        self.allWaypoints[oldWp.rowNumber] = wp
        self.wpCoordRev[wp.getCoordinate()] = wp
        self.dataChanged.emit(idx, idx)

    def removeWaypoint(self, wp: Waypoint):
        tgtWp = wp.rowNumber
        i = tgtWp
        while i < len(self.allWaypoints):
            self.allWaypoints[i].rowNumber -= 1
            i += 1
        self.beginRemoveRows(QModelIndex(), tgtWp, tgtWp)
        del self.allWaypoints[tgtWp]
        del self.wpCoordRev[wp.getCoordinate()]
        self.endRemoveRows()
        # send signal to update WP# displayed on the map
        txtStart = self.index(tgtWp)
        txtEnd = self.index(len(self.allWaypoints) - 1)
        self.dataChanged.emit(txtStart, txtEnd)

class MapView(QQuickView):

    selectWaypointForAction = pyqtSignal(float, float, float, float)
    moveWaypointEvent = pyqtSignal(object, object)
    moveHomeEvent = pyqtSignal(object)

    dragWpLat = 0.0
    dragWpLng = 0.0
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

    def waypointEditEvent(self, x, y, clkLat, clkLng):
        self.selectWaypointForAction.emit(x, y, clkLat, clkLng)

    def mapDragEvent(self, x, y, lat, lng, actType):
        '''
        actType = 0, start of drag; actType = 1, end of drag
        '''
        # print('drag ({0}, {1}) on ({2}, {3}), type: {4}'.format(x, y, lat, lng, actType))
        if actType == 0:
            self.dragWpLat = lat
            self.dragWpLng = lng
            self.dragStart = True
        elif actType == 1:
            if self.dragStart:
                fromWp = QGeoCoordinate(self.dragWpLat, self.dragWpLng)
                toWp = QGeoCoordinate(lat, lng)
                self.moveWaypointEvent.emit(fromWp, toWp)
                self.dragStart = False

    def updateHomeEvent(self, lat, lng):
        # print('New home location: {0}, {1}'.format(lat, lng))
        self.moveHomeEvent.emit(QGeoCoordinate(lat, lng))

    def updateDroneLocation(self, lat, lng, hacc, vacc):
        # print('Drone location updated: {}, {}'.format(lat, lng))
        self.map.moveDroneLocation(lat, lng, hacc, vacc)

    def minimumSize(self):
        return QSize(600, 480)

    # def maximumSize(self):
    # return QSize(10000, 10000)

class MapWidget(QWidget):

    mapView = None
    waypointList = None
    horizonLine = -1
    defaultLatitude = 30.0
    editWPpopup = []

    def __init__(self, mapQmlFile, parent = None):
        super().__init__(parent)
        self.mapView = MapView(QUrl.fromLocalFile(mapQmlFile))
        self.waypointList = WaypointList(self.mapView.wpModel.allWaypoints, parent)

        container = QWidget.createWindowContainer(self.mapView)
        container.setMinimumSize(self.mapView.minimumSize())
        container.setMaximumSize(self.mapView.maximumSize())
        container.setFocusPolicy(Qt.TabFocus)
        self.mapView.restorePreviousView()
        self.mapView.wpModel.createWaypointAction.connect(self.createWaypointEvent)
        self.mapView.moveWaypointEvent.connect(self.moveWaypointEvent)
        self.mapView.moveHomeEvent.connect(self.updateHomeLocationEvent)
        self.waypointList.editWaypoint.connect(self.showEditWaypointWindow)
        self.waypointList.deleteWaypoint.connect(self.removeWaypoint)
        self.waypointList.preDeleteWaypoint.connect(self.markWaypointForRemoval)
        self.waypointList.cancelDeleteWaypoint.connect(self.unmarkWaypointNoRemoval)
        l = QVBoxLayout()
        l.addWidget(container)
        l.addWidget(self.waypointList)
        self.setLayout(l)

    def mouseMoveEvent(self, e: QMouseEvent):
        super().mouseMoveEvent(e)
        h = e.pos().y()
        d = h - self.horizonLine
        if self.horizonLine > 0:
            self.mapView.setHeight(self.mapView.height() + d)
            self.waypointList.resize(self.waypointList.width(), self.waypointList.height() - d)
            self.waypointList.move(self.waypointList.pos().x(), self.waypointList.pos().y() + d)
        self.horizonLine = h

    def moveWaypointEvent(self, fromWp: QGeoCoordinate, toWp: QGeoCoordinate):
        print('[WP] move ({0}, {1}) to ({2}, {3})'.format(fromWp.latitude(), fromWp.longitude(), toWp.latitude(), toWp.longitude()))
        wpId = self.mapView.wpModel.updateWaypointCoordinate(fromWp, toWp)
        if wpId >= 0:
            self.waypointList.moveWaypoint(wpId, toWp)

    def updateHomeLocationEvent(self, home: QGeoCoordinate):
        # print('home loc:', home)
        self.waypointList.updateHomeLocation(home)

    def showEditWaypointWindow(self, wp: Waypoint):
        print('edit wp#{0}'.format(wp.wpID))
        popup = WaypointEditWindow(wp)
        self.editWPpopup.append(popup)
        popup.updateWaypoint.connect(self.acceptWaypointEdit)
        popup.show()

    def acceptWaypointEdit(self, wp: Waypoint):
        # print('edited WP:', wp)
        self.waypointList.updateWaypoint(wp)
        # sent new wp to map polyline
        self.mapView.wpModel.refreshWaypoint(wp)
        self.mapView.map.waypointChanged.emit(wp.rowNumber + 1, wp.latitude, wp.longitude)

    def removeWaypoint(self, wp: Waypoint):
        # print('delete wp#{0}'.format(wp.wpID))
        wpIdx = wp.rowNumber + 1
        self.mapView.wpModel.removeWaypoint(wp)
        self.mapView.map.waypointRemoved.emit(wpIdx)

    def createWaypointEvent(self, wp: Waypoint):
        print('[new] create WP: ({0}, {1}) at {2}'.format(wp.latitude, wp.longitude, wp.altitude))
        self.waypointList.addWaypoint(wp)

    def markWaypointForRemoval(self, wp: Waypoint):
        self.mapView.wpModel.markWaypoint(wp)

    def unmarkWaypointNoRemoval(self, wp: Waypoint):
        self.mapView.wpModel.unmarkWaypoint(wp)

#test only
if __name__ == "__main__":
    app = QApplication(sys.argv)
    current_path = os.path.abspath(os.path.dirname(__file__))
    qmlFile = os.path.join(current_path, 'map.qml')
    w = MapWidget(qmlFile)
    w.show()
    sys.exit(app.exec_())
