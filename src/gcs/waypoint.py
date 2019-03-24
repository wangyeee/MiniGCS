from PyQt5.QtCore import QObject, QRect, Qt, QVariant, pyqtSignal
from PyQt5.QtPositioning import QGeoCoordinate
from PyQt5.QtWidgets import (QComboBox, QFormLayout, QHBoxLayout, QHeaderView,
                             QLabel, QLineEdit, QMessageBox, QPushButton,
                             QTableWidget, QTableWidgetItem, QWidget)
from PyQt5.QtGui import QValidator, QDoubleValidator

from pymavlink.dialects.v10 import common as mavlink

WP_TYPE_NAMES = {
    mavlink.MAV_CMD_NAV_WAYPOINT : 'Waypoint',
    mavlink.MAV_CMD_NAV_LOITER_UNLIM : 'Loiter Unlimited',
    mavlink.MAV_CMD_NAV_LOITER_TURNS : 'Loiter Turns',
    mavlink.MAV_CMD_NAV_LOITER_TIME : 'Loiter Time',
    mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH : 'Return to Launch',
    mavlink.MAV_CMD_NAV_LAND : 'Land',
    mavlink.MAV_CMD_NAV_TAKEOFF : 'Takeoff',
    mavlink.MAV_CMD_NAV_LAND_LOCAL : 'Land Local',
    mavlink.MAV_CMD_NAV_TAKEOFF_LOCAL : 'Takeoff Local',
    mavlink.MAV_CMD_NAV_FOLLOW : 'Follow',
    mavlink.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT : 'Change Altitude',
    mavlink.MAV_CMD_NAV_LOITER_TO_ALT : 'Loiter to Altitude',
}

class Waypoint(QObject):

    rowNumber = 0
    latitude = 0.0
    longitude = 0.0
    altitude = 0.0
    # default to "navigate to waypoint (16)"
    waypointType = mavlink.MAV_CMD_NAV_WAYPOINT

    def __init__(self, rowNumber, latitude, longitude, altitude, parent = None):
        super().__init__(parent)
        self.rowNumber = rowNumber
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude

    def __str__(self):
        return 'Waypoint#{0} type: {1}({2}) @ ({3}, {4}, {5})'.format(self.rowNumber,
                                                                      WP_TYPE_NAMES[self.waypointType],
                                                                      self.waypointType,
                                                                      self.latitude,
                                                                      self.longitude,
                                                                      self.altitude)

    def getCoordinate(self):
        return QGeoCoordinate(self.latitude, self.longitude)

    def copy(self):
        c = Waypoint(self.rowNumber, self.latitude, self.longitude, self.altitude, self.parent())
        c.waypointType = self.waypointType
        return c

    @staticmethod
    def decimalToDMS(decimal):
        n = 1.0
        if decimal < 0.0:
            decimal = 0.0 - decimal
            n = -1.0
        degrees = int(decimal)
        decimal -= degrees
        decimal *= 60
        minutes = int(decimal)
        decimal -= minutes
        decimal *= 60
        return degrees * n, minutes, decimal

    @staticmethod
    def decimalFromDMS(degrees, minutes, seconds):
        n = 1.0
        if degrees < 0.0:
            degrees = 0.0 - degrees
            n = -1.0
        return n * (degrees + minutes / 60 + seconds / 3600)

class WaypointEditPanel(QWidget):

    editBtn : QPushButton = None
    delBtn : QPushButton = None
    # dupBtn = None
    edtCb = None
    delCb = None

    def __init__(self, wp: Waypoint, edtLbl = 'Edit', delLbl = 'Remove', edtCb = None, delCb = None, parent = None):
        super().__init__(parent)
        self.waypoint = wp
        self.editBtn = QPushButton(edtLbl)
        self.delBtn = QPushButton(delLbl)
        # self.dupBtn = QPushButton(dupLbl)
        l = QHBoxLayout()
        l.addWidget(self.editBtn)
        l.addWidget(self.delBtn)
        # l.addWidget(self.dupBtn)
        if edtCb != None:
            self.edtCb = edtCb
            self.editBtn.clicked.connect(lambda: self.edtCb(self.waypoint, 0))
        if delCb != None:
            self.delCb = delCb
            self.delBtn.clicked.connect(lambda: self.delCb(self.waypoint, 1))

        l.setContentsMargins(0, 0, 0, 0)
        self.setLayout(l)

class WPDropDownPanel(QWidget):

    dropDownList = None

    def __init__(self, dropDownList: dict, currentSelection = 0, parent = None):
        super().__init__(parent)
        self.dropDown = QComboBox(self)
        self.dropDownList = dropDownList
        self.setSelection(currentSelection)
        l = QHBoxLayout()
        l.setContentsMargins(5, 0, 5, 0)
        l.addWidget(self.dropDown)
        self.setLayout(l)

    def setSelection(self, idx: QVariant):
        # print('select:', idx)
        i = 0
        for idVal, idName in self.dropDownList.items():
            self.dropDown.addItem(idName, QVariant(idVal))
            if idVal == idx:
                self.dropDown.setCurrentIndex(i)
            i += 1

    def getSelection(self):
        return self.dropDown.currentData()

class WPDegreePanel(QWidget):

    LATITUDE_TYPE = 'LAT'
    LONGITUDE_TYPE = 'LNG'

    decimalValue = 0.0

    valueChanged = pyqtSignal(object)
    dirType = None

    def __init__(self, decimalValue, ctype, parent = None):
        '''
        ctype=LAT/LNG
        [deg][min][sec][EW/NS]
        '''
        super().__init__(parent)
        self.decimalValue = decimalValue
        # Degrees Minutes Seconds
        d, m, s = Waypoint.decimalToDMS(self.decimalValue)

        self.dirType = ctype
        self.dirLabel = QLabel(self._getDirLabel(d, self.dirType))
        self.dirLabel.adjustSize()

        self.degreesField = QLineEdit('%3d' % (d if d > 0 else -d))
        self.degreesField.setFrame(False)
        self._setFieldWidth(self.degreesField, 3)
        self.degreesLabel = QLabel(u'\N{DEGREE SIGN}')
        self.degreesLabel.adjustSize()

        self.minutesField = QLineEdit('%2d' % m)
        self.minutesField.setFrame(False)
        self._setFieldWidth(self.minutesField, 2)
        self.minutesLabel = QLabel(chr(0x2019))
        self.minutesLabel.adjustSize()

        self.secondsField = QLineEdit('%.4f' % s)
        self.secondsField.setFrame(False)
        self._setFieldWidth(self.secondsField, 7)
        self.secondsLabel = QLabel(chr(0x201D))
        self.secondsLabel.adjustSize()

        l = QHBoxLayout()
        l.setContentsMargins(5, 0, 5, 0)
        l.addWidget(self.degreesField)
        l.addWidget(self.degreesLabel)
        l.addWidget(self.minutesField)
        l.addWidget(self.minutesLabel)
        l.addWidget(self.secondsField)
        l.addWidget(self.secondsLabel)
        l.addWidget(self.dirLabel)

        self.setLayout(l)

    def getValue(self):
        d = int(self.degreesField.text())
        m = int(self.minutesField.text())
        s = float(self.secondsField.text())
        decimal = Waypoint.decimalFromDMS(d, m, s)
        sym = self.dirLabel.text()
        if sym == 'S' or sym == 'W':
            decimal = 0 - decimal
        return decimal

    def setValue(self, val: float):
        self.decimalValue = val
        d, m, s = Waypoint.decimalToDMS(self.decimalValue)
        self.degreesField.setText('%3d' % (d if d > 0 else -d))
        self.minutesField.setText('%2d' % m)
        self.secondsField.setText('%.4f' % s)
        self.dirLabel.setText(self._getDirLabel(d, self.dirType))

    def _getDirLabel(self, deg, ctype):
        if ctype == 'LAT':
            t = 'N'
            if deg < 0:
                t = 'S'
            return t
        elif ctype == 'LNG':
            t = 'E'
            if deg < 0:
                t = 'W'
            return t
        return ' '

    def _setFieldWidth(self, field, length):
        fm = field.fontMetrics()
        m = field.textMargins()
        c = field.contentsMargins()
        w = length * fm.width('0') + m.left() + m.right() + c.left() + c.right()
        field.setMaximumWidth(w + 8)

class WPDecimalPanel(QWidget):

    value = 0.0

    valueChanged = pyqtSignal(object)

    def __init__(self, value, uom = None, validator: QValidator = None, parent = None):
        super().__init__(parent)
        self.value = value
        self.editField = QLineEdit(str(value))
        self.editField.returnPressed.connect(self.valueChangedEvent)
        if validator == None:
            self.editField.setValidator(QDoubleValidator())
        else:
            self.editField.setValidator(validator)
        l = QHBoxLayout()
        l.setContentsMargins(5, 0, 5, 0)
        l.addWidget(self.editField)
        if uom != None:
            self.uomLabel = QLabel(uom)
            l.addWidget(self.uomLabel)
        self.setLayout(l)

    def valueChangedEvent(self):
        self.value = float(self.editField.text())
        # print('new value:', self.value)
        self.valueChanged.emit(self)

    def getValue(self):
        return float(self.editField.text())

    def setValue(self, val: float):
        self.value = val
        self.editField.setText(str(self.value))

class WaypointList(QTableWidget):

    homeLocation = Waypoint(0, 0, 0, 0)
    wpList = None
    requestReturnToHome = pyqtSignal(object)  # pass current home location
    editWaypoint = pyqtSignal(object)  # show popup window to edit the waypoint
    deleteWaypoint = pyqtSignal(object)  # remove a waypoint
    preDeleteWaypoint = pyqtSignal(object)  # signal sent before removing a waypoint
    cancelDeleteWaypoint = pyqtSignal(object)  # signal sent for cancelled waypoint removal

    def __init__(self, wpList, parent = None):
        super().__init__(parent)
        # self.verticalHeader().setVisible(False)
        self.createTableHeader()
        self.wpList = wpList
        self.setRowCount(len(self.wpList) + 1)
        self.createHomeWaypointRow()

    def createTableHeader(self):
        '''
        Type, Latitude, Longitude, Altitude, Actions
        '''
        wpHdr = ['Type', 'Latitude', 'Longitude', 'Altitude', 'Actions']
        self.setColumnCount(len(wpHdr))
        self.setHorizontalHeaderLabels(wpHdr)
        self.resizeRowsToContents()
        self.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        # self.resizeColumnsToContents()

    def _setRowData(self, rowNumber, dataArray):
        i = 0
        for s in dataArray:
            if isinstance(s, QWidget):
                self.setCellWidget(rowNumber, i, s)
            elif isinstance(s, QTableWidgetItem):
                self.setItem(rowNumber, i, s)
            i += 1

    def updateWaypoint(self, newWp: Waypoint):
        wpIdx = newWp.rowNumber + 1
        widget = self.cellWidget(wpIdx, 0)  # Type (WPDropDownPanel)
        widget.setSelection(QVariant(newWp.waypointType))
        widget = self.cellWidget(wpIdx, 1)  # Latitude(WPDecimalPanel)
        widget.setValue(newWp.latitude)
        widget = self.cellWidget(wpIdx, 2)  # Longitude(WPDecimalPanel)
        widget.setValue(newWp.longitude)
        widget = self.cellWidget(wpIdx, 3)  # Altitude(WPDecimalPanel)
        widget.setValue(newWp.altitude)
        widget = self.cellWidget(wpIdx, 4)  # WaypointEditPanel
        widget.editBtn.clicked.disconnect()
        widget.delBtn.clicked.disconnect()
        widget.editBtn.clicked.connect(lambda: self.wpButtonEvent(newWp, 0))
        widget.delBtn.clicked.connect(lambda: self.wpButtonEvent(newWp, 1))

    def moveWaypoint(self, wpIdx, coord: QGeoCoordinate):
        # print('wpidx:', wpIdx)
        if 0 <= wpIdx < len(self.wpList):
            self.wpList[wpIdx].latitude = coord.latitude()
            self.wpList[wpIdx].longitude = coord.longitude()
            self.updateWaypoint(self.wpList[wpIdx])

    def updateHomeLocation(self, coord: QGeoCoordinate):
        self.homeLocation.latitude = coord.latitude()
        self.homeLocation.longitude = coord.longitude()
        self.setHomeLocation(self.homeLocation)

    def addWaypoint(self, wp: Waypoint):
        if wp == None:
            return
        data = []
        data.append(WPDropDownPanel(WP_TYPE_NAMES, wp.waypointType))
        data.append(WPDegreePanel(wp.latitude, WPDegreePanel.LATITUDE_TYPE))
        data.append(WPDegreePanel(wp.longitude, WPDegreePanel.LONGITUDE_TYPE))
        data.append(WPDecimalPanel(wp.altitude, 'M'))
        pnl = WaypointEditPanel(wp, 'Edit', 'Remove', self.wpButtonEvent, self.wpButtonEvent)
        data.append(pnl)
        self.setRowCount(len(self.wpList) + 1)
        self._setRowData(wp.rowNumber + 1, data)
        self.scrollToBottom()

    def wpButtonEvent(self, wp: Waypoint, act):
        ''' route event to other components '''
        if act == 0:  # update
            wpIdx = wp.rowNumber + 1
            widget = self.cellWidget(wpIdx, 0)  # Type (WPDropDownPanel)
            wp.waypointType = widget.getSelection()
            widget = self.cellWidget(wpIdx, 1)  # Latitude(WPDecimalPanel)
            wp.latitude = widget.getValue()
            widget = self.cellWidget(wpIdx, 2)  # Longitude(WPDecimalPanel)
            wp.longitude = widget.getValue()
            widget = self.cellWidget(wpIdx, 3)  # Altitude(WPDecimalPanel)
            wp.altitude = widget.getValue()
            self.editWaypoint.emit(wp)
        elif act == 1: # delete
            self.preDeleteWaypoint.emit(wp)
            cfm = QMessageBox.question(self.window(),
                                       'Confirm removal',
                                       'Are you sure to remove waypoint#{0} at ({1}, {2})?'.format(wp.rowNumber + 1, wp.latitude, wp.longitude),
                                       QMessageBox.Yes, QMessageBox.No)
            if cfm == QMessageBox.Yes:
                self.removeRow(wp.rowNumber + 1)
                self.update()
                self.deleteWaypoint.emit(wp)
            else:
                self.cancelDeleteWaypoint.emit(wp)

    def highlightWaypoint(self, wp: Waypoint):
        self.scrollTo(self.model().index(wp.rowNumber + 1, 0))
        self.selectRow(wp.rowNumber + 1)

    def setHomeLocation(self, h: Waypoint):
        wpIdx = 0
        widget = self.cellWidget(wpIdx, 1)  # Latitude(WPDecimalPanel)
        widget.setValue(h.latitude)
        widget = self.cellWidget(wpIdx, 2)  # Longitude(WPDecimalPanel)
        widget.setValue(h.longitude)
        widget = self.cellWidget(wpIdx, 3)  # Altitude(WPDecimalPanel)
        widget.setValue(h.altitude)

    def createHomeWaypointRow(self):
        ''' can only be called once '''
        data = []
        s = QTableWidgetItem('Home')
        s.setTextAlignment(Qt.AlignCenter)
        s.setFlags(s.flags() & (~Qt.ItemIsEditable))
        data.append(s)
        data.append(WPDegreePanel(self.homeLocation.latitude, WPDegreePanel.LATITUDE_TYPE))
        data.append(WPDegreePanel(self.homeLocation.longitude, WPDegreePanel.LONGITUDE_TYPE))
        data.append(WPDecimalPanel(self.homeLocation.altitude, 'M'))
        pnl = WaypointEditPanel(self.homeLocation, 'Edit', 'Return')
        pnl.editBtn.clicked.connect(self.editHomeLocation)
        pnl.delBtn.clicked.connect(self.requestReturnHome)
        data.append(pnl)
        self._setRowData(0, data)

    def editHomeLocation(self):
        print('Edit home location popup')
        # self.updateHomeLocation.emit(self.homeLocation)

    def requestReturnHome(self):
        print('RTL started: {0}, {1} at {2}'.format(self.homeLocation.latitude, self.homeLocation.longitude, self.homeLocation.altitude))
        self.requestReturnToHome.emit(self.homeLocation)

class WaypointEditWindow(QWidget):

    waypoint = None

    updateWaypoint = pyqtSignal(object)  # send value in popup window to application

    def __init__(self, wp: Waypoint, parent = None):
        super().__init__(parent)
        layout = QFormLayout()
        self.latField = WPDegreePanel(wp.latitude, WPDegreePanel.LATITUDE_TYPE) # QLineEdit(str(wp.latitude))
        self.lngField = WPDegreePanel(wp.longitude, WPDegreePanel.LONGITUDE_TYPE) # QLineEdit(str(wp.longitude))
        self.altField = WPDecimalPanel(wp.altitude, 'M')
        self.typeSel = WPDropDownPanel(WP_TYPE_NAMES, wp.waypointType)
        layout.addRow(QLabel('Latitude'), self.latField)
        layout.addRow(QLabel('Longitude'), self.lngField)
        layout.addRow(QLabel('Altitude'), self.altField)
        layout.addRow(QLabel('Type'), self.typeSel)
        self.actPanel = QWidget(self)
        self.okBtn = QPushButton('OK')
        self.cnlBtn = QPushButton('Cancel')
        pnlLay = QHBoxLayout()
        pnlLay.addWidget(self.okBtn)
        pnlLay.addWidget(self.cnlBtn)
        self.actPanel.setLayout(pnlLay)
        layout.addRow(self.actPanel)
        self.cnlBtn.clicked.connect(self.close)
        self.okBtn.clicked.connect(self.updateWaypointEvent)
        # self.latField.returnPressed.connect(self.okBtn.click)
        # self.lngField.returnPressed.connect(self.okBtn.click)
        self.altField.valueChanged.connect(self.okBtn.click)
        self.waypoint = wp
        self.setWindowTitle('Edit Waypoint#{0}'.format(wp.rowNumber))
        self.setLayout(layout)
        self.setGeometry(QRect(100, 100, 400, 200))

    def updateWaypointEvent(self):
        # TODO data validation
        self.waypoint.latitude = self.latField.getValue()
        self.waypoint.longitude = self.lngField.getValue()
        self.waypoint.altitude = self.altField.getValue()
        self.waypoint.waypointType = self.typeSel.getSelection()
        # print('new WP:', self.waypoint)
        self.updateWaypoint.emit(self.waypoint)
        self.close()

    def keyPressEvent(self, event):
        key = event.key()
        if key == Qt.Key_Escape:
            self.close()
