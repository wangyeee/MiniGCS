from pymavlink.mavutil import mavlink
from PyQt5.QtCore import QObject, QPoint, QRect, Qt, QVariant, pyqtSignal
from PyQt5.QtGui import (QCursor, QDoubleValidator, QIntValidator, QPalette,
                         QValidator)
from PyQt5.QtPositioning import QGeoCoordinate
from PyQt5.QtWidgets import (QTableWidget, QTableWidgetItem, QWidget,
    QButtonGroup, QComboBox, QFormLayout, QGridLayout, QHBoxLayout,
    QHeaderView, QLabel, QLineEdit, QMessageBox, QPushButton, QRadioButton)

from WaypointDefault import WP_DEFAULTS, WP_TYPE_NAMES, MAVWaypointParameter
from UserData import UserData

class Waypoint(QObject):

    def __init__(self, rowNumber, latitude, longitude, altitude, waypointType = mavlink.MAV_CMD_NAV_WAYPOINT, parent = None):
        super().__init__(parent)
        self.rowNumber = rowNumber
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude
        self.waypointType = waypointType
        self.mavlinkParameters = {}
        if self.waypointType in (mavlink.MAV_CMD_NAV_LAND_LOCAL, mavlink.MAV_CMD_NAV_TAKEOFF_LOCAL):
            self.mavlinkParameters[MAVWaypointParameter.PARAM5] = 0.0
            self.mavlinkParameters[MAVWaypointParameter.PARAM6] = 0.0
            self.mavlinkParameters[MAVWaypointParameter.PARAM7] = 0.0
        else:
            self.mavlinkParameters[MAVWaypointParameter.PARAM5] = self.latitude
            self.mavlinkParameters[MAVWaypointParameter.PARAM6] = self.longitude
            self.mavlinkParameters[MAVWaypointParameter.PARAM7] = self.altitude

    def __str__(self):
        return 'Waypoint#{0} type: {1}({2}) @ ({3}, {4}, {5}) -- {6}'.format(self.rowNumber,
                                                                             WP_TYPE_NAMES[self.waypointType],
                                                                             self.waypointType,
                                                                             self.latitude,
                                                                             self.longitude,
                                                                             self.altitude,
                                                                             str(self.mavlinkParameters))

    def getCoordinate(self):
        return QGeoCoordinate(self.latitude, self.longitude)

    def copy(self):
        c = Waypoint(self.rowNumber, self.latitude, self.longitude, self.altitude, self.parent())
        c.waypointType = self.waypointType
        return c

    def toMavlinkMessage(self, sysId, compId, seq, current = 0, autocontinue = 0):
        item = mavlink.MAVLink_mission_item_message(
            sysId, compId, seq, mavlink.MAV_FRAME_GLOBAL,
            self.waypointType, current, autocontinue,
            Waypoint.defaultParameterValue(self.waypointType, MAVWaypointParameter.PARAM1) \
                if MAVWaypointParameter.PARAM1 not in self.mavlinkParameters \
                else self.mavlinkParameters[MAVWaypointParameter.PARAM1],
            Waypoint.defaultParameterValue(self.waypointType, MAVWaypointParameter.PARAM2) \
                if MAVWaypointParameter.PARAM2 not in self.mavlinkParameters \
                else self.mavlinkParameters[MAVWaypointParameter.PARAM2],
            Waypoint.defaultParameterValue(self.waypointType, MAVWaypointParameter.PARAM3) \
                if MAVWaypointParameter.PARAM3 not in self.mavlinkParameters \
                else self.mavlinkParameters[MAVWaypointParameter.PARAM3],
            Waypoint.defaultParameterValue(self.waypointType, MAVWaypointParameter.PARAM4) \
                if MAVWaypointParameter.PARAM4 not in self.mavlinkParameters \
                else self.mavlinkParameters[MAVWaypointParameter.PARAM4],
            self.mavlinkParameters[MAVWaypointParameter.PARAM5],
            self.mavlinkParameters[MAVWaypointParameter.PARAM6],
            self.mavlinkParameters[MAVWaypointParameter.PARAM7])
        return item

    @staticmethod
    def defaultParameterValue(wpType, param):
        if wpType in WP_DEFAULTS:
            p = WP_DEFAULTS[wpType]
            if param in p:
                return p[param]
        return None

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

class WaypointListCell(QWidget):

    def __init__(self, moveCursorWhenFocus = False, parent = None):
        super().__init__(parent)
        self.moveCursorWhenFocus = False
        self.editEnable = True
        self.moveCursorWhenFocus = moveCursorWhenFocus

    def nextFocus(self):
        self._moveCursor()
        return 0

    def prevFocus(self):
        self._moveCursor()
        return 0

    def enableEdit(self):
        self.editEnable = True

    def disableEdit(self):
        self.editEnable = False

    def _setPaletteColor(self, base, text):
        p = self.palette()
        if p == None:
            p = QPalette()
        p.setColor(QPalette.Base, base)
        p.setColor(QPalette.Text, text)
        self.setPalette(p)

    def _moveCursor(self, field = None):
        if self.moveCursorWhenFocus:
            if field == None:
                field = self
            pos = field.mapToGlobal(QPoint(0,0))
            ctr = field.rect().center()
            QCursor.setPos(pos.x() + ctr.x(), pos.y() + ctr.y())
            field.setCursor(Qt.BlankCursor)

class WaypointEditPanel(QWidget):

    def __init__(self, wp: Waypoint, edtLbl = 'Edit', delLbl = 'Remove', edtCb = None, delCb = None, parent = None):
        super().__init__(parent)
        self.waypoint = wp
        self.editBtn = QPushButton(edtLbl)
        self.delBtn = QPushButton(delLbl)
        l = QHBoxLayout()
        l.addWidget(self.editBtn)
        l.addWidget(self.delBtn)
        if edtCb != None:
            self.edtCb = edtCb
            self.editBtn.clicked.connect(lambda: self.edtCb(self.waypoint, 0))
        if delCb != None:
            self.delCb = delCb
            self.delBtn.clicked.connect(lambda: self.delCb(self.waypoint, 1))

        l.setContentsMargins(0, 0, 0, 0)
        self.setLayout(l)

class WPDropDownPanel(WaypointListCell):

    def __init__(self, dropDownList: dict, currentSelection = 0, parent = None):
        super().__init__(True, parent)
        self.dropDown = QComboBox(self)
        self.dropDownList = dropDownList
        self.setSelection(currentSelection, True)
        l = QHBoxLayout()
        l.setContentsMargins(5, 0, 5, 0)
        l.addWidget(self.dropDown)
        self.setLayout(l)

    def setSelection(self, idx: QVariant, createOption = False):
        i = 0
        for idVal, idName in self.dropDownList.items():
            if createOption:
                self.dropDown.addItem(idName, QVariant(idVal))
            if idVal == idx:
                self.dropDown.setCurrentIndex(i)
            i += 1

    def getSelection(self):
        return self.dropDown.currentData()

    def getSelectionIndex(self):
        return self.dropDown.currentIndex()

    def nextFocus(self):
        super().nextFocus()
        self.dropDown.setFocus(Qt.OtherFocusReason)
        return 1

    def prevFocus(self):
        super().prevFocus()
        self.dropDown.setFocus(Qt.OtherFocusReason)
        return -1

class FocusLineEdit(QLineEdit):

    focusGainedSignal = pyqtSignal(object)
    focusLostSignal = pyqtSignal(object)
    valueOverflowSignal = pyqtSignal(float)

    def __init__(self, contents, step = 1.0, parent = None):
        super().__init__(contents, parent)
        self.start = 0
        self.end = 0
        self.valueValidator = None
        self.isBeingEdited = False
        self.step = step

    def setValueRange(self, start, end, decimals = 0):
        self.start = start
        self.end = end
        if self.valueValidator == None:
            self.valueValidator = QIntValidator(self.start, self.end) if decimals < 1 else QDoubleValidator(self.start, self.end, decimals)
        else:
            if decimals > 0:
                self.valueValidator.setRange(self.start, self.end, decimals)
            else:
                self.valueValidator.setRange(self.start, self.end)
        self.setValidator(self.valueValidator)

    def wheelEvent(self, e):
        if self.isBeingEdited and self.isReadOnly() == False:
            if type(self.validator()) == QDoubleValidator:
                self.addValue(self.step * e.angleDelta().y() / 120)
            elif type(self.validator()) == QIntValidator:
                self.addValue(int(self.step * e.angleDelta().y() / 120))

    def addValue(self, value):
        if type(self.validator()) == QDoubleValidator:
            v = float(self.text())
            v += value
            v = self.__constraintValue(v)
            self.setText('{:.4f}'.format(v))
        elif type(self.validator()) == QIntValidator:
            v = int(self.text())
            v += value
            v = self.__constraintValue(v)
            self.setText(str(int(v)))

    def __constraintValue(self, value):
        if value > self.end:
            self.valueOverflowSignal.emit(self.step)
            value -= self.end
        elif value < self.start:
            self.valueOverflowSignal.emit(-self.step)
            value += self.end
        return value

    def claimFocus(self):
        self.setFocus(Qt.OtherFocusReason)

    def focusInEvent(self, e):
        super().focusInEvent(e)
        self.isBeingEdited = True
        self.focusGainedSignal.emit(self)

    def focusOutEvent(self, e):
        super().focusOutEvent(e)
        self.isBeingEdited = False
        self.focusLostSignal.emit(self)

class WPDegreePanel(WaypointListCell):

    LATITUDE_TYPE = 'LAT'
    LONGITUDE_TYPE = 'LNG'

    valueChanged = pyqtSignal(object)
    focusInSignal = pyqtSignal(object)

    def __init__(self, decimalValue, ctype, cachedWP = None, parent = None):
        '''
        ctype=LAT/LNG
        [deg][min][sec][EW/NS]
        '''
        super().__init__(True, parent)
        self.cachedCellLocation = None
        self.currentInFocusEdit = None
        self.decimalValue = decimalValue
        self.cachedWP = cachedWP
        # Degrees Minutes Seconds
        d, m, s = Waypoint.decimalToDMS(self.decimalValue)

        self.dirType = ctype
        self.dirLabel = QLabel(self._getDirLabel(d, self.dirType))
        self.dirLabel.adjustSize()

        self.degreesField = FocusLineEdit('%3d' % (d if d > 0 else -d))
        self.degreesField.setFrame(False)
        delta = 0 if ctype == self.LATITUDE_TYPE else 90
        self.degreesField.setValueRange(-90 + delta, 90 + delta)
        self._setFieldWidth(self.degreesField, 3)
        self.degreesLabel = QLabel(u'\N{DEGREE SIGN}')
        self.degreesLabel.adjustSize()

        self.minutesField = FocusLineEdit('%2d' % m)
        self.minutesField.setFrame(False)
        self.minutesField.setValueRange(0, 60)
        self._setFieldWidth(self.minutesField, 2)
        self.minutesLabel = QLabel(chr(0x2019))
        self.minutesLabel.adjustSize()

        self.secondsField = FocusLineEdit('%.4f' % s)
        self.secondsField.setFrame(False)
        self.secondsField.setValueRange(0.0, 60.0, 4)
        self._setFieldWidth(self.secondsField, 7)
        self.secondsLabel = QLabel(chr(0x201D))
        self.secondsLabel.adjustSize()

        self.secondsField.valueOverflowSignal.connect(self.minutesField.addValue)
        self.minutesField.valueOverflowSignal.connect(self.degreesField.addValue)

        l = QHBoxLayout()
        l.setContentsMargins(5, 0, 5, 0)
        self.degreesField.focusLostSignal.connect(self.valueChangedEvent)
        self.minutesField.focusLostSignal.connect(self.valueChangedEvent)
        self.secondsField.focusLostSignal.connect(self.valueChangedEvent)
        self.degreesField.focusGainedSignal.connect(self.reportInFocusEvent)
        self.minutesField.focusGainedSignal.connect(self.reportInFocusEvent)
        self.secondsField.focusGainedSignal.connect(self.reportInFocusEvent)
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
        if sym in('S', 'W'):
            decimal = 0 - decimal
        return decimal

    def setValue(self, val: float):
        self.decimalValue = val
        d, m, s = Waypoint.decimalToDMS(self.decimalValue)
        self.degreesField.setText('%3d' % (d if d > 0 else -d))
        self.minutesField.setText('%2d' % m)
        self.secondsField.setText('%.4f' % s)
        self.dirLabel.setText(self._getDirLabel(d, self.dirType))

    def valueChangedEvent(self):
        val = self.getValue()
        if self.cachedWP != None:
            if self.dirType == self.LATITUDE_TYPE:
                self.cachedWP.latitude  = val
            elif self.dirType == self.LONGITUDE_TYPE:
                self.cachedWP.longitude = val
        self.valueChanged.emit(self)

    def reportInFocusEvent(self, edit):
        self.currentInFocusEdit = edit
        self.focusInSignal.emit(self.cachedCellLocation)

    def nextFocus(self):
        if self.currentInFocusEdit != None:
            if self.currentInFocusEdit == self.degreesField:
                self.currentInFocusEdit = self.minutesField
                self.minutesField.claimFocus()
                self._moveCursor(self.minutesField)
                return 0
            if self.currentInFocusEdit == self.minutesField:
                self.currentInFocusEdit = self.secondsField
                self.secondsField.claimFocus()
                self._moveCursor(self.secondsField)
                return 0
            if self.currentInFocusEdit == self.secondsField:
                self.currentInFocusEdit = None
                return 1
            return 0  # Not going to happen
        else:
            self.degreesField.claimFocus()
            self._moveCursor(self.degreesField)
            return 0

    def prevFocus(self):
        if self.currentInFocusEdit != None:
            if self.currentInFocusEdit == self.secondsField:
                self.currentInFocusEdit = self.minutesField
                self.minutesField.claimFocus()
                self._moveCursor(self.minutesField)
                return 0
            if self.currentInFocusEdit == self.minutesField:
                self.currentInFocusEdit = self.degreesField
                self.degreesField.claimFocus()
                self._moveCursor(self.degreesField)
                return 0
            if self.currentInFocusEdit == self.degreesField:
                self.currentInFocusEdit = None
                return -1
            return 0  # Not going to happen
        else:
            self.secondsField.claimFocus()
            self._moveCursor(self.secondsField)
            return 0

    def setCellLocation(self, cell):
        self.cachedCellLocation = cell

    def enableEdit(self):
        super().enableEdit()
        self._setPaletteColor(Qt.white, Qt.black)
        self.degreesField.setReadOnly(False)
        self.minutesField.setReadOnly(False)
        self.secondsField.setReadOnly(False)

    def disableEdit(self):
        super().disableEdit()
        self._setPaletteColor(Qt.gray, Qt.darkGray)
        self.degreesField.setReadOnly(True)
        self.minutesField.setReadOnly(True)
        self.secondsField.setReadOnly(True)

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

class WPNumberPanel(WaypointListCell):

    valueChanged = pyqtSignal(object)

    def __init__(self, value, isInteger = False, uom = None, validator: QValidator = None, parent = None):
        super().__init__(True, parent)
        self.value = value
        self.isInteger = isInteger
        self.editField = FocusLineEdit(str(value))
        self.editField.returnPressed.connect(self.valueChangedEvent)
        self.editField.focusLostSignal.connect(self.valueChangedEvent)
        if validator == None:
            if self.isInteger:
                self.editField.setValidator(QIntValidator())
            else:
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
        if self.isInteger:
            self.value = int(self.editField.text())
        else:
            self.value = float(self.editField.text())
        self.valueChanged.emit(self)

    def getValue(self):
        return int(self.editField.text()) if self.isInteger else float(self.editField.text())

    def setValue(self, val):
        if self.editField.isReadOnly() == False:
            self.value = val
            self.editField.setText(str(self.value))

    def nextFocus(self):
        super().nextFocus()
        self.editField.setFocus(Qt.OtherFocusReason)
        return 1

    def prevFocus(self):
        super().prevFocus()
        self.editField.setFocus(Qt.OtherFocusReason)
        return -1

    def enableEdit(self):
        super().enableEdit()
        self._setPaletteColor(Qt.white, Qt.black)
        self.editField.setReadOnly(False)

    def disableEdit(self):
        super().disableEdit()
        self._setPaletteColor(Qt.gray, Qt.darkGray)
        self.editField.setReadOnly(True)

class WaypointList(QTableWidget):

    requestReturnToHome = pyqtSignal()
    editWaypoint = pyqtSignal(object)  # show popup window to edit the waypoint
    deleteWaypoint = pyqtSignal(object)  # remove a waypoint
    preDeleteWaypoint = pyqtSignal(object)  # signal sent before removing a waypoint
    cancelDeleteWaypoint = pyqtSignal(object)  # signal sent for cancelled waypoint removal
    afterWaypointEdited = pyqtSignal(object)  # signal after waypoint has been edited in the list

    def __init__(self, wpList, parent = None):
        from instruments.map import UD_MAP_KEY, UD_MAP_INIT_LATITUDE_KEY, UD_MAP_INIT_LONGITUDE_KEY
        super().__init__(parent)
        lat0 = 0.0
        lng0 = 0.0
        try:
            mapConf = UserData.getInstance().getUserDataEntry(UD_MAP_KEY)
            if mapConf != None:
                lat0 = float(mapConf[UD_MAP_INIT_LATITUDE_KEY])
                lng0 = float(mapConf[UD_MAP_INIT_LONGITUDE_KEY])
        except (TypeError, ValueError):
            pass
        self.homeLocation = Waypoint(0, lat0, lng0, 0, mavlink.MAV_CMD_DO_SET_HOME)
        self.currentInFocusCell = None
        self.manualSetHomeLocationSource = False
        # self.verticalHeader().setVisible(False)
        self.createTableHeader()
        self.wpList = wpList
        self.setRowCount(len(self.wpList) + 1)
        self.createHomeWaypointRow()
        # HomeEditWindow.HOME_SRC_UAV (1) -> use current drone location
        # HomeEditWindow.HOME_SRC_MAP (0) -> use the location of home icon on map
        self.homeLocation.mavlinkParameters[MAVWaypointParameter.PARAM1] = HomeEditWindow.HOME_SRC_UAV
        self.homeEditWindow = HomeEditWindow()
        self.homeEditWindow.updateHomeLocationSourceSignal.connect(self.updateHomeLocationSourceEvent)

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
                if hasattr(s, 'setCellLocation'):
                    s.setCellLocation((rowNumber, i))
                self.setCellWidget(rowNumber, i, s)
            elif isinstance(s, QTableWidgetItem):
                self.setItem(rowNumber, i, s)
            i += 1

    def updateWaypoint(self, newWp: Waypoint):
        wpIdx = newWp.rowNumber + 1
        widget = self.cellWidget(wpIdx, 0)  # Type (WPDropDownPanel)
        widget.setSelection(QVariant(newWp.waypointType))
        widget = self.cellWidget(wpIdx, 1)  # Latitude(WPNumberPanel)
        widget.setValue(newWp.latitude)
        widget = self.cellWidget(wpIdx, 2)  # Longitude(WPNumberPanel)
        widget.setValue(newWp.longitude)
        widget = self.cellWidget(wpIdx, 3)  # Altitude(WPNumberPanel)
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
        if coord.type() == QGeoCoordinate.Coordinate3D:
            self.homeLocation.altitude = coord.altitude()
        self.setHomeLocation(self.homeLocation)

    def addWaypoint(self, wp: Waypoint):
        if wp == None:
            return
        data = []
        data.append(WPDropDownPanel(WP_TYPE_NAMES, wp.waypointType))
        latpanel = WPDegreePanel(wp.latitude, WPDegreePanel.LATITUDE_TYPE, wp)
        latpanel.valueChanged.connect(self.processWaypointOutfocusUpdate)
        latpanel.focusInSignal.connect(self.cellFocusChangedEvent)
        data.append(latpanel)
        lngpanel = WPDegreePanel(wp.longitude, WPDegreePanel.LONGITUDE_TYPE, wp)
        lngpanel.valueChanged.connect(self.processWaypointOutfocusUpdate)
        lngpanel.focusInSignal.connect(self.cellFocusChangedEvent)
        data.append(lngpanel)
        data.append(WPNumberPanel(wp.altitude, uom='M'))
        pnl = WaypointEditPanel(wp, 'Edit', 'Remove', self.wpButtonEvent, self.wpButtonEvent)
        data.append(pnl)
        self.setRowCount(len(self.wpList) + 1)
        self._setRowData(wp.rowNumber + 1, data)
        self.scrollToBottom()

    def processWaypointOutfocusUpdate(self, panel):
        wp = panel.cachedWP
        if wp != None:
            self.afterWaypointEdited.emit(wp)

    def wpButtonEvent(self, wp: Waypoint, act):
        ''' route event to other components '''
        if act == 0:  # update
            wpIdx = wp.rowNumber + 1
            widget = self.cellWidget(wpIdx, 0)  # Type (WPDropDownPanel)
            wp.waypointType = widget.getSelection()
            widget = self.cellWidget(wpIdx, 1)  # Latitude(WPNumberPanel)
            wp.latitude = widget.getValue()
            widget = self.cellWidget(wpIdx, 2)  # Longitude(WPNumberPanel)
            wp.longitude = widget.getValue()
            widget = self.cellWidget(wpIdx, 3)  # Altitude(WPNumberPanel)
            wp.altitude = widget.getValue()
            self.editWaypoint.emit(wp)
        elif act == 1: # delete
            self.preDeleteWaypoint.emit(wp)
            cfm = QMessageBox.question(self.window(),
                                       'Confirm removal',
                                       'Are you sure to remove waypoint#{0} at {1}?'.format(wp.rowNumber + 1, wp.getCoordinate().toString()),
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
        widget = self.cellWidget(wpIdx, 1)  # Latitude(WPNumberPanel)
        widget.setValue(h.latitude)
        widget = self.cellWidget(wpIdx, 2)  # Longitude(WPNumberPanel)
        widget.setValue(h.longitude)
        widget = self.cellWidget(wpIdx, 3)  # Altitude(WPNumberPanel)
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
        data.append(WPNumberPanel(self.homeLocation.altitude, uom='M'))
        pnl = WaypointEditPanel(self.homeLocation, 'Edit', 'Return')
        pnl.editBtn.clicked.connect(self.editHomeLocation)
        pnl.delBtn.clicked.connect(self.requestReturnHome)
        data.append(pnl)
        self._setRowData(0, data)

    def editHomeLocation(self):
        src = self.homeLocation.mavlinkParameters[MAVWaypointParameter.PARAM1]
        if self.manualSetHomeLocationSource and src == HomeEditWindow.HOME_SRC_MAP:
            src = HomeEditWindow.HOME_SRC_MANUAL
        self.homeEditWindow.setInitialHomeLocationSource(src, self.homeLocation)
        self.homeEditWindow.show()

    def requestReturnHome(self):
        print('RTL started: {0}, {1} at {2}'.format(self.homeLocation.latitude, self.homeLocation.longitude, self.homeLocation.altitude))
        cfm = QMessageBox.question(self.window(),
                                   'Confirm RTH',
                                   'Start return to home {}?'.format(self.homeLocation.getCoordinate().toString()),
                                   QMessageBox.Yes, QMessageBox.No)
        if cfm == QMessageBox.Yes:
            self.requestReturnToHome.emit()

    def removeAllRows(self):
        while self.rowCount() > 1:  # the first row is home, which will be kept
            self.removeRow(1)

    def keyPressEvent(self, event):
        key = event.key()
        if key in (Qt.Key_Backtab, Qt.Key_Tab):
            # Tab/ShiftTab keys will be used to navigate between defferent line edits
            if self.currentInFocusCell != None:
                # 1. Check if current cess has multiple line edits
                # 2. Move to another cell
                row = self.currentInFocusCell[0]
                col = self.currentInFocusCell[1]
                if key == Qt.Key_Tab:
                    ret = self.cellWidget(row, col).nextFocus()
                    if ret == 1:
                        if col < self.columnCount() - 2:  # Skip last column
                            col += 1
                        elif row < self.rowCount() - 1:
                            row += 1
                            col = 0
                        self.currentInFocusCell = (row, col)
                        self.cellWidget(row, col).nextFocus()
                elif key == Qt.Key_Backtab:
                    ret = self.cellWidget(row, col).prevFocus()
                    if ret == -1:
                        if col > 0:
                            col -= 1
                        elif row > 1: # skip home row
                            row -= 1
                            col = self.columnCount() - 2 # Skip last column
                        self.currentInFocusCell = (row, col)
                        self.cellWidget(row, col).prevFocus()
        else: # key not in (Qt.Key_Backtab, Qt.Key_Tab)
            super().keyPressEvent(event)

    def cellFocusChangedEvent(self, cell):
        self.currentInFocusCell = cell

    def updateHomeLocationSourceEvent(self, src, coord):
        self.manualSetHomeLocationSource = False
        if src == HomeEditWindow.HOME_SRC_UAV:
            self.homeLocation.mavlinkParameters[MAVWaypointParameter.PARAM1] = HomeEditWindow.HOME_SRC_UAV
        elif src in (HomeEditWindow.HOME_SRC_MAP, HomeEditWindow.HOME_SRC_MANUAL):
            self.homeLocation.mavlinkParameters[MAVWaypointParameter.PARAM1] = HomeEditWindow.HOME_SRC_MAP
        if coord != None and src == HomeEditWindow.HOME_SRC_MANUAL:
            self.manualSetHomeLocationSource = True
            self.updateHomeLocation(coord)

class HomeEditWindow(QWidget):

    # 1st param, src: could be HOME_SRC_MAP, HOME_SRC_UAV, or HOME_SRC_MANUAL
    # 2st param, coord: QGeoCoordinate object if src == HOME_SRC_MANUAL, else None
    updateHomeLocationSourceSignal = pyqtSignal(int, object)

    HOME_SRC_MAP = 0  # Must be 0 to conform mavlink protocol
    HOME_SRC_UAV = 1  # Must be 1 to conform mavlink protocol
    HOME_SRC_MANUAL = 2

    def __init__(self, parent = None):
        super().__init__(parent)
        self.setWindowTitle('Edit Home Location')
        self.homeSrcGrp = QButtonGroup(self)
        l = QGridLayout()
        row = 0
        column = 0
        rowSpan = 1
        columnSpan = 3
        l.addWidget(QLabel('Choose data source'), row, column, rowSpan, columnSpan, Qt.AlignLeft)
        self.srcDroneHomeLocation = QRadioButton('Home location in the UAV', self)
        self.homeSrcGrp.addButton(self.srcDroneHomeLocation)
        row += 1
        l.addWidget(self.srcDroneHomeLocation, row, column, rowSpan, columnSpan, Qt.AlignLeft)

        self.srcMapHomeLocation = QRadioButton('Home location on the map', self)
        self.homeSrcGrp.addButton(self.srcMapHomeLocation)
        row += 1
        l.addWidget(self.srcMapHomeLocation, row, column, rowSpan, columnSpan, Qt.AlignLeft)

        self.srcInputHomeLocation = QRadioButton('Manual input', self)
        self.homeSrcGrp.addButton(self.srcInputHomeLocation)
        row += 1
        l.addWidget(self.srcInputHomeLocation, row, column, rowSpan, columnSpan, Qt.AlignLeft)
        self.srcInputHomeLocation.toggled.connect(self.__enableManualInputs)

        row += 1
        columnSpan = 1
        l.addWidget(QLabel('Latitude'), row, column, rowSpan, columnSpan, Qt.AlignLeft)
        self.latitudeField = WPDegreePanel(0.0, WPDegreePanel.LATITUDE_TYPE)
        columnSpan = 2
        l.addWidget(self.latitudeField, row, column + 1, rowSpan, columnSpan, Qt.AlignLeft)

        row += 1
        columnSpan = 1
        l.addWidget(QLabel('Longitude'), row, column, rowSpan, columnSpan, Qt.AlignLeft)
        self.longitudeField = WPDegreePanel(0.0, WPDegreePanel.LONGITUDE_TYPE)
        columnSpan = 2
        l.addWidget(self.longitudeField, row, column + 1, rowSpan, columnSpan, Qt.AlignLeft)

        row += 1
        columnSpan = 1
        l.addWidget(QLabel('Altitude'), row, column, rowSpan, columnSpan, Qt.AlignLeft)
        self.altitudeField = WPNumberPanel(0.0, uom='m')
        columnSpan = 2
        l.addWidget(self.altitudeField, row, column + 1, rowSpan, columnSpan, Qt.AlignLeft)

        row += 1
        columnSpan = 1
        self.okButton = QPushButton('OK', self)
        self.okButton.clicked.connect(self.__setHomeLocationSource)
        l.addWidget(self.okButton, row, column + 1, rowSpan, columnSpan, Qt.AlignRight)
        self.cancelButton = QPushButton('Cancel', self)
        self.cancelButton.clicked.connect(self.close)
        l.addWidget(self.cancelButton, row, column + 2, rowSpan, columnSpan, Qt.AlignRight)

        self.setInitialHomeLocationSource(HomeEditWindow.HOME_SRC_UAV)  # Default to home location reported by the UAV
        self.setLayout(l)

    def setInitialHomeLocationSource(self, src, wp: Waypoint = None):
        if src == HomeEditWindow.HOME_SRC_UAV:
            self.srcDroneHomeLocation.setChecked(True)
            self.__enableManualInputs(False)
        elif src == HomeEditWindow.HOME_SRC_MAP:
            self.srcMapHomeLocation.setChecked(True)
            self.__enableManualInputs(False)
        elif src == HomeEditWindow.HOME_SRC_MANUAL:
            self.srcInputHomeLocation.setChecked(True)
            self.__enableManualInputs(True)
        if wp != None:
            self.latitudeField.setValue(wp.latitude)
            self.longitudeField.setValue(wp.longitude)
            self.altitudeField.setValue(wp.altitude)

    def __setHomeLocationSource(self):
        if self.srcDroneHomeLocation.isChecked():
            self.updateHomeLocationSourceSignal.emit(HomeEditWindow.HOME_SRC_UAV, None)
        elif self.srcMapHomeLocation.isChecked():
            self.updateHomeLocationSourceSignal.emit(HomeEditWindow.HOME_SRC_MAP, None)
        elif self.srcInputHomeLocation.isChecked():
            coord = QGeoCoordinate(self.latitudeField.getValue(), self.longitudeField.getValue(), self.altitudeField.getValue())
            self.updateHomeLocationSourceSignal.emit(HomeEditWindow.HOME_SRC_MANUAL, coord)
        self.close()

    def __enableManualInputs(self, checked):
        if checked:
            self.latitudeField.enableEdit()
            self.longitudeField.enableEdit()
            self.altitudeField.enableEdit()
        else:
            self.latitudeField.disableEdit()
            self.longitudeField.disableEdit()
            self.altitudeField.disableEdit()

class WaypointEditWindowFactory:

    @staticmethod
    def createWaypointEditWindow(wp: Waypoint):
        if wp.waypointType in (mavlink.MAV_CMD_NAV_LOITER_TIME, mavlink.MAV_CMD_NAV_LOITER_TURNS,
                               mavlink.MAV_CMD_NAV_LOITER_UNLIM, mavlink.MAV_CMD_NAV_LOITER_TO_ALT):
            return LoiterWaypointEditWindow(wp)
        if wp.waypointType in (mavlink.MAV_CMD_NAV_LAND, mavlink.MAV_CMD_NAV_LAND_LOCAL):
            return LandWaypointEditWindow(wp)
        if wp.waypointType in (mavlink.MAV_CMD_NAV_TAKEOFF, mavlink.MAV_CMD_NAV_TAKEOFF_LOCAL):
            return TakeoffWaypointEditWindow(wp)
        if wp.waypointType == mavlink.MAV_CMD_NAV_FOLLOW:
            return FollowWaypointEditWindow(wp)
        if wp.waypointType == mavlink.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:
            return ContinueAndChangeAltitudeWaypointEditWindow(wp)
        return WaypointEditWindow(wp)

    @staticmethod
    def createYesNoDropdown(options = None, parent = None):
        if options is None:
            options = {0 : 'No', 1 : 'Yes'}
        dropDown = QComboBox(parent)
        for data, label in options.items():
            dropDown.addItem(label, QVariant(data))
        return dropDown

class WaypointEditWindow(QWidget):

    updateWaypoint = pyqtSignal(object)  # send value in popup window to application

    def __init__(self, wp: Waypoint, parent = None):
        super().__init__(parent)
        self.waypoint = wp
        layout = QFormLayout()
        self.latField = WPDegreePanel(wp.latitude, WPDegreePanel.LATITUDE_TYPE) # QLineEdit(str(wp.latitude))
        self.lngField = WPDegreePanel(wp.longitude, WPDegreePanel.LONGITUDE_TYPE) # QLineEdit(str(wp.longitude))
        self.altField = WPNumberPanel(wp.altitude, uom='M')
        self.typeSel = WPDropDownPanel(WP_TYPE_NAMES, wp.waypointType)
        layout.addRow(QLabel('Latitude'), self.latField)
        layout.addRow(QLabel('Longitude'), self.lngField)
        layout.addRow(QLabel('Altitude'), self.altField)
        layout.addRow(QLabel('Type'), self.typeSel)
        self.addAdditionalFields(layout)
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
        self.setWindowTitle('Edit Waypoint#{0}'.format(wp.rowNumber))
        self.setLayout(layout)
        self.setGeometry(QRect(100, 100, 400, 200))

    def addAdditionalFields(self, layout):
        if self.waypoint.waypointType == mavlink.MAV_CMD_NAV_WAYPOINT:
            self.holdTimeField = WPNumberPanel(self.getFieldValue(MAVWaypointParameter.PARAM1), uom='s', isInteger=True)
            layout.addRow(QLabel('Hold'), self.holdTimeField)
            self.acceptanceRadiusField = WPNumberPanel(self.getFieldValue(MAVWaypointParameter.PARAM2), uom='m')
            layout.addRow(QLabel('Accept Radius'), self.acceptanceRadiusField)
            self.passRadiusField = WPNumberPanel(self.getFieldValue(MAVWaypointParameter.PARAM3), uom='m')
            layout.addRow(QLabel('Pass Radius'), self.passRadiusField)
            self.yawAngleField = WPNumberPanel(self.getFieldValue(MAVWaypointParameter.PARAM4), uom=u'\N{DEGREE SIGN}')
            layout.addRow(QLabel('Yaw'), self.yawAngleField)

    def updateAdditionalFieldValues(self):
        if self.waypoint.waypointType == mavlink.MAV_CMD_NAV_WAYPOINT:
            self.waypoint.mavlinkParameters[MAVWaypointParameter.PARAM1] = self.holdTimeField.getValue()
            self.waypoint.mavlinkParameters[MAVWaypointParameter.PARAM2] = self.acceptanceRadiusField.getValue()
            self.waypoint.mavlinkParameters[MAVWaypointParameter.PARAM3] = self.passRadiusField.getValue()
            self.waypoint.mavlinkParameters[MAVWaypointParameter.PARAM4] = self.yawAngleField.getValue()

    def getFieldValue(self, param):
        if param in self.waypoint.mavlinkParameters:
            return self.waypoint.mavlinkParameters[param]
        return self.getDefaultParameterValue(param)

    def getDefaultParameterValue(self, param):
        if self.waypoint.waypointType == mavlink.MAV_CMD_NAV_WAYPOINT:
            if param == MAVWaypointParameter.PARAM1:
                return 0
            if param in (MAVWaypointParameter.PARAM2, MAVWaypointParameter.PARAM3):
                return 2.0
            if param == MAVWaypointParameter.PARAM4:
                return 0.0
        return 0

    def updateWaypointEvent(self):
        # TODO data validation
        self.waypoint.latitude = self.latField.getValue()
        self.waypoint.longitude = self.lngField.getValue()
        self.waypoint.altitude = self.altField.getValue()
        self.waypoint.waypointType = self.typeSel.getSelection()
        self.updateAdditionalFieldValues()
        print('new WP:', self.waypoint)
        self.updateWaypoint.emit(self.waypoint)
        self.close()

    def keyPressEvent(self, event):
        key = event.key()
        if key == Qt.Key_Escape:
            self.close()

class ContinueAndChangeAltitudeWaypointEditWindow(WaypointEditWindow):
    def __init__(self, wp: Waypoint, parent = None):
        super().__init__(wp, parent)
        self.setWindowTitle('Edit Continue and Change Altitude Waypoint#{}'.format(wp.rowNumber))

    def addAdditionalFields(self, layout):
        self.actionDropdown = WPDropDownPanel({
            0 : 'Neutral',
            1 : 'Climbing',
            2 : 'Descending'}, self.getFieldValue(MAVWaypointParameter.PARAM1), self)  # Param1
        layout.addRow(QLabel('Action'), self.actionDropdown)
        self.desiredAltitudeField = WPNumberPanel(self.getFieldValue(MAVWaypointParameter.PARAM7), uom='m')  # Param7
        layout.addRow(QLabel('Altitude'), self.desiredAltitudeField)

    def updateAdditionalFieldValues(self):
        self.waypoint.mavlinkParameters[MAVWaypointParameter.PARAM1] = self.actionDropdown.getSelectionIndex()
        self.waypoint.mavlinkParameters[MAVWaypointParameter.PARAM7] = self.desiredAltitudeField.getValue()

    # def getDefaultParameterValue(self, param):

class FollowWaypointEditWindow(WaypointEditWindow):
    def __init__(self, wp: Waypoint, parent = None):
        super().__init__(wp, parent)
        self.setWindowTitle('Edit Follow Waypoint#{}'.format(wp.rowNumber))

    def addAdditionalFields(self, layout):
        self.followingLogicField = WPNumberPanel(self.getFieldValue(MAVWaypointParameter.PARAM1), isInteger = True)  # Param1
        layout.addRow(QLabel('Following'), self.followingLogicField)
        self.groundSpeedField = WPNumberPanel(self.getFieldValue(MAVWaypointParameter.PARAM2), uom='m/s')  # Param2
        layout.addRow(QLabel('Ground Speed'), self.groundSpeedField)
        self.radiusField = WPNumberPanel(self.getFieldValue(MAVWaypointParameter.PARAM3), uom='m')  # Param3
        layout.addRow(QLabel('Radius'), self.radiusField)
        self.yawAngleField = WPNumberPanel(self.getFieldValue(MAVWaypointParameter.PARAM4), uom=u'\N{DEGREE SIGN}')  # Param4
        layout.addRow(QLabel('Yaw'), self.yawAngleField)

    def updateAdditionalFieldValues(self):
        self.waypoint.mavlinkParameters[MAVWaypointParameter.PARAM1] = self.followingLogicField.getValue()
        self.waypoint.mavlinkParameters[MAVWaypointParameter.PARAM2] = self.groundSpeedField.getValue()
        self.waypoint.mavlinkParameters[MAVWaypointParameter.PARAM3] = self.radiusField.getValue()
        self.waypoint.mavlinkParameters[MAVWaypointParameter.PARAM4] = self.yawAngleField.getValue()

    # def getDefaultParameterValue(self, param):

class TakeoffWaypointEditWindow(WaypointEditWindow):

    def __init__(self, wp: Waypoint, parent = None):
        super().__init__(wp, parent)
        self.setWindowTitle('Edit Takeoff Waypoint#{}'.format(wp.rowNumber))

    def addAdditionalFields(self, layout):
        self.minPitchField = WPNumberPanel(self.getFieldValue(MAVWaypointParameter.PARAM1), uom=u'\N{DEGREE SIGN}')  # Param1
        layout.addRow(QLabel('Pitch'), self.minPitchField)
        self.yawAngleField = WPNumberPanel(self.getFieldValue(MAVWaypointParameter.PARAM4), uom=u'\N{DEGREE SIGN}')  # Param4
        layout.addRow(QLabel('Yaw'), self.yawAngleField)
        if self.waypoint.waypointType == mavlink.MAV_CMD_NAV_TAKEOFF_LOCAL:
            self.ascendRateField = WPNumberPanel(self.getFieldValue(MAVWaypointParameter.PARAM3), uom='m/s')  # Param3
            layout.addRow(QLabel('Ascend Rate'), self.ascendRateField)
            self.xPositionField = WPNumberPanel(self.getFieldValue(MAVWaypointParameter.PARAM6), uom='m') # Param6
            layout.addRow(QLabel('X Position'), self.xPositionField)
            self.yPositionField = WPNumberPanel(self.getFieldValue(MAVWaypointParameter.PARAM5), uom='m') # Param5
            layout.addRow(QLabel('Y Position'), self.yPositionField)
            self.zPositionField = WPNumberPanel(self.getFieldValue(MAVWaypointParameter.PARAM7), uom='m') # Param7
            layout.addRow(QLabel('Z Position'), self.zPositionField)

    def updateAdditionalFieldValues(self):
        self.waypoint.mavlinkParameters[MAVWaypointParameter.PARAM1] = self.minPitchField.getValue()
        self.waypoint.mavlinkParameters[MAVWaypointParameter.PARAM4] = self.yawAngleField.getValue()
        if self.waypoint.waypointType == mavlink.MAV_CMD_NAV_TAKEOFF_LOCAL:
            self.waypoint.mavlinkParameters[MAVWaypointParameter.PARAM3] = self.ascendRateField.getValue()
            self.waypoint.mavlinkParameters[MAVWaypointParameter.PARAM6] = self.xPositionField.getValue()
            self.waypoint.mavlinkParameters[MAVWaypointParameter.PARAM5] = self.yPositionField.getValue()
            self.waypoint.mavlinkParameters[MAVWaypointParameter.PARAM7] = self.zPositionField.getValue()

    # def getDefaultParameterValue(self, param):

class LandWaypointEditWindow(WaypointEditWindow):

    def __init__(self, wp: Waypoint, parent = None):
        super().__init__(wp, parent)
        self.setWindowTitle('Edit Land Waypoint#{}'.format(wp.rowNumber))

    def addAdditionalFields(self, layout):
        self.yawAngleField = WPNumberPanel(self.getFieldValue(MAVWaypointParameter.PARAM4), uom=u'\N{DEGREE SIGN}')
        layout.addRow(QLabel('Yaw'), self.yawAngleField) # Param4
        if self.waypoint.waypointType == mavlink.MAV_CMD_NAV_LAND:
            self.abortAltitudeField = WPNumberPanel(self.getFieldValue(MAVWaypointParameter.PARAM1), uom='m')  # Param1
            layout.addRow(QLabel('Abort Alt'), self.abortAltitudeField)
            self.landModeDropdown = WPDropDownPanel({
                mavlink.PRECISION_LAND_MODE_DISABLED : 'Disabled',
                mavlink.PRECISION_LAND_MODE_OPPORTUNISTIC : 'Opportunistic',
                mavlink.PRECISION_LAND_MODE_REQUIRED : 'Required'}, self.getFieldValue(MAVWaypointParameter.PARAM2), self)  # Param2
            layout.addRow(QLabel('Land Mode'), self.landModeDropdown)
        elif self.waypoint.waypointType == mavlink.MAV_CMD_NAV_LAND_LOCAL:
            self.targetNumberField = WPNumberPanel(self.getFieldValue(MAVWaypointParameter.PARAM1), isInteger = True)
            layout.addRow(QLabel('Target'), self.targetNumberField)
            self.offsetField = WPNumberPanel(self.getFieldValue(MAVWaypointParameter.PARAM2), uom='m')
            layout.addRow(QLabel('Offset'), self.offsetField)
            self.descendRateField = WPNumberPanel(self.getFieldValue(MAVWaypointParameter.PARAM3), uom='m/s')
            layout.addRow(QLabel('Desend Rate'), self.descendRateField)
            self.xPositionField = WPNumberPanel(self.getFieldValue(MAVWaypointParameter.PARAM6), uom='m') # Param6
            layout.addRow(QLabel('X Position'), self.xPositionField)
            self.yPositionField = WPNumberPanel(self.getFieldValue(MAVWaypointParameter.PARAM5), uom='m') # Param5
            layout.addRow(QLabel('Y Position'), self.yPositionField)
            self.zPositionField = WPNumberPanel(self.getFieldValue(MAVWaypointParameter.PARAM7), uom='m') # Param7
            layout.addRow(QLabel('Z Position'), self.zPositionField)

    def updateAdditionalFieldValues(self):
        self.waypoint.mavlinkParameters[MAVWaypointParameter.PARAM4] = self.yawAngleField.getValue()
        if self.waypoint.waypointType == mavlink.MAV_CMD_NAV_LAND:
            self.waypoint.mavlinkParameters[MAVWaypointParameter.PARAM1] = self.abortAltitudeField.getValue()
            self.waypoint.mavlinkParameters[MAVWaypointParameter.PARAM2] = self.landModeDropdown.getSelectionIndex()
        elif self.waypoint.waypointType == mavlink.MAV_CMD_NAV_LAND_LOCAL:
            self.waypoint.mavlinkParameters[MAVWaypointParameter.PARAM1] = self.targetNumberField.getValue()
            self.waypoint.mavlinkParameters[MAVWaypointParameter.PARAM2] = self.offsetField.getValue()
            self.waypoint.mavlinkParameters[MAVWaypointParameter.PARAM3] = self.descendRateField.getValue()
            self.waypoint.mavlinkParameters[MAVWaypointParameter.PARAM6] = self.xPositionField.getValue()
            self.waypoint.mavlinkParameters[MAVWaypointParameter.PARAM5] = self.yPositionField.getValue()
            self.waypoint.mavlinkParameters[MAVWaypointParameter.PARAM7] = self.zPositionField.getValue()

    # def getDefaultParameterValue(self, param):

class LoiterWaypointEditWindow(WaypointEditWindow):

    def __init__(self, wp: Waypoint, parent = None):
        super().__init__(wp, parent)
        self.setWindowTitle('Edit Loiter Waypoint#{}'.format(wp.rowNumber))

    def addAdditionalFields(self, layout):
        self.loiterRadiusField = WPNumberPanel(self.getFieldValue(MAVWaypointParameter.PARAM3), uom='m')
        layout.addRow(QLabel('Radius'), self.loiterRadiusField)
        if self.waypoint.waypointType == mavlink.MAV_CMD_NAV_LOITER_TIME:
            self.loiterTimeField = WPNumberPanel(self.getFieldValue(MAVWaypointParameter.PARAM1), uom='s')
            layout.addRow(QLabel('Time'), self.loiterTimeField)
        elif self.waypoint.waypointType == mavlink.MAV_CMD_NAV_LOITER_TURNS:
            self.loiterTurnsField = WPNumberPanel(self.getFieldValue(MAVWaypointParameter.PARAM1), uom='turn')
            layout.addRow(QLabel('Turns'), self.loiterTurnsField)
        elif self.waypoint.waypointType == mavlink.MAV_CMD_NAV_LOITER_TO_ALT:
            self.headingRequiredDropDown = WaypointEditWindowFactory.createYesNoDropdown(parent=self)
            self.headingRequiredDropDown.setCurrentIndex(self.getFieldValue(MAVWaypointParameter.PARAM1))
            layout.addRow(QLabel('Heading Required'), self.headingRequiredDropDown)
            self.xtrackLocationDropDown = WaypointEditWindowFactory.createYesNoDropdown({0 : 'Center', 1 : 'Exit'}, self)
            self.xtrackLocationDropDown.setCurrentIndex(self.getFieldValue(MAVWaypointParameter.PARAM4))
            layout.addRow(QLabel('Xtrack Location'), self.xtrackLocationDropDown)

    def updateAdditionalFieldValues(self):
        self.waypoint.mavlinkParameters[MAVWaypointParameter.PARAM3] = self.loiterRadiusField.getValue()
        if self.waypoint.waypointType == mavlink.MAV_CMD_NAV_LOITER_TIME:
            self.waypoint.mavlinkParameters[MAVWaypointParameter.PARAM1] = self.loiterTimeField.getValue()
        elif self.waypoint.waypointType == mavlink.MAV_CMD_NAV_LOITER_TURNS:
            self.waypoint.mavlinkParameters[MAVWaypointParameter.PARAM1] = self.loiterTurnsField.getValue()
        elif self.waypoint.waypointType == mavlink.MAV_CMD_NAV_LOITER_TO_ALT:
            self.waypoint.mavlinkParameters[MAVWaypointParameter.PARAM1] = self.headingRequiredDropDown.currentIndex()
            self.waypoint.mavlinkParameters[MAVWaypointParameter.PARAM4] = self.xtrackLocationDropDown.currentIndex()

    # def getDefaultParameterValue(self, param):
