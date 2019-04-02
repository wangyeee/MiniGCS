from pymavlink.dialects.v10 import common as mavlink
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtWidgets import (QFileDialog, QHBoxLayout, QHeaderView,
                             QMessageBox, QPushButton, QTableWidget,
                             QTableWidgetItem, QVBoxLayout, QWidget)

PARAM_VALUE_TYPE_NAMES = {
    mavlink.MAV_PARAM_TYPE_UINT8 : '8-bit unsigned integer',
    mavlink.MAV_PARAM_TYPE_INT8 : '8-bit signed integer',
    mavlink.MAV_PARAM_TYPE_UINT16 : '16-bit unsigned integer',
    mavlink.MAV_PARAM_TYPE_INT16 : '16-bit signed integer',
    mavlink.MAV_PARAM_TYPE_UINT32 : '32-bit unsigned integer',
    mavlink.MAV_PARAM_TYPE_INT32 : '32-bit signed integer',
    mavlink.MAV_PARAM_TYPE_UINT64 : '64-bit unsigned integer',
    mavlink.MAV_PARAM_TYPE_INT64 : '64-bit signed integer',
    mavlink.MAV_PARAM_TYPE_REAL32 : '32-bit floating-point',
    mavlink.MAV_PARAM_TYPE_REAL64 : '64-bit floating-point'
}

PARAM_TYPE_RAW_ROLE = Qt.UserRole + 1

class ParameterList(QTableWidget):

    paramList = None
    paramNameIndexCache = {}

    allParams = {}
    changedParams = []

    def __init__(self, paramList, parent = None):
        super().__init__(parent)
        self.paramList = sorted(paramList, key = lambda p: p.param_id)
        print('Init param list with {} params.'.format(len(self.paramList)))
        self.createTableHeader()
        self.createTableBody()

    def createTableHeader(self):
        '''
        [param_index], param_id, param_value, param_type
        '''
        hdr = ['Name', 'Value', 'Type']
        self.setColumnCount(len(hdr))
        self.setHorizontalHeaderLabels(hdr)
        self.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)

    def createTableBody(self):
        rowNumber = 0
        self.setRowCount(len(self.paramList))
        for param in self.paramList:
            self.paramNameIndexCache[param.param_id] = rowNumber
            name = QTableWidgetItem(param.param_id)
            name.setFlags(name.flags() ^ Qt.ItemIsEditable)
            vstr = None
            if self._isIntegerType(param.param_type):
                vstr = str(int(param.param_value))
            elif self._isFloatType(param.param_type):
                vstr = str(float(param.param_value))
            else:
                vstr = str(param.param_value)
            value = QTableWidgetItem(vstr)
            pstr = None
            if param.param_type in PARAM_VALUE_TYPE_NAMES:
                pstr = PARAM_VALUE_TYPE_NAMES[param.param_type]
            else:
                pstr = 'UNKNOWN TYPE: {}'.format(param.param_type)
            ptype = QTableWidgetItem(pstr)
            ptype.setData(PARAM_TYPE_RAW_ROLE, int(param.param_type))
            ptype.setFlags(ptype.flags() ^ Qt.ItemIsEditable)
            self.setItem(rowNumber, 0, name)
            self.setItem(rowNumber, 1, value)
            self.setItem(rowNumber, 2, ptype)
            rowNumber += 1
        self.resizeRowsToContents()
        self.resizeColumnsToContents()

    def collectTableBody(self):
        self.changedParams.clear()
        self.allParams.clear()
        for rowNumber in range(self.rowCount()):
            name = self.item(rowNumber, 0).text()
            ptype = self.item(rowNumber, 2).data(PARAM_TYPE_RAW_ROLE)
            if self._isIntegerType(ptype):
                value = int(self.item(rowNumber, 1).text())
            elif self._isFloatType(ptype):
                value = float(self.item(rowNumber, 1).text())
            else:
                value = self.item(rowNumber, 1).text()
            # print('Row#{}: {} = {} ({})'.format(rowNumber, name, value, ptype))
            self.allParams[name] = value
            oldVal = self._initValue(name)
            if oldVal != value:
                self.changedParams.append((name, value, oldVal, ptype))
                # print('Value changed on row#{}: {} = {} -> {}'.format(rowNumber, name, oldVal, value))

    def _initValue(self, name):
        if name in self.paramNameIndexCache:
            return self.paramList[self.paramNameIndexCache[name]].param_value
        return 0.0

    def _isIntegerType(self, ptype):
        return ptype in (mavlink.MAV_PARAM_TYPE_UINT8,
                         mavlink.MAV_PARAM_TYPE_INT8,
                         mavlink.MAV_PARAM_TYPE_UINT16,
                         mavlink.MAV_PARAM_TYPE_INT16,
                         mavlink.MAV_PARAM_TYPE_UINT32,
                         mavlink.MAV_PARAM_TYPE_INT32,
                         mavlink.MAV_PARAM_TYPE_UINT64,
                         mavlink.MAV_PARAM_TYPE_INT64)

    def _isFloatType(self, ptype):
        return ptype in (mavlink.MAV_PARAM_TYPE_REAL32,
                         mavlink.MAV_PARAM_TYPE_REAL64)

class ParameterPanel(QWidget):

    uploadNewParametersSignal = pyqtSignal(object)  # dict[name] = value

    uploadUAVStep = 0

    def __init__(self, params, parent = None):
        super().__init__(parent)
        l = QVBoxLayout()
        self.paramList = ParameterList(params, parent)
        l.addWidget(self.paramList)
        self.actionPanel = QWidget()
        pLayout = QHBoxLayout()
        self.saveToFileButton = QPushButton('Save to file')
        pLayout.addWidget(self.saveToFileButton)
        self.saveToFileButton.clicked.connect(self.saveToFile)
        self.uploadButton = QPushButton('Upload to UAV >')
        pLayout.addWidget(self.uploadButton)
        self.uploadButton.clicked.connect(self.uploadToUAV)
        self.cancelButton = QPushButton('Cancel')
        pLayout.addWidget(self.cancelButton)
        self.cancelButton.clicked.connect(self.close)
        self.actionPanel.setLayout(pLayout)
        l.addWidget(self.actionPanel)
        self.setLayout(l)

    def uploadToUAV(self):
        if self.uploadUAVStep == 0:  ## 'Upload to UAV' clicked
            self.paramList.collectTableBody()
            if (len(self.paramList.changedParams) == 0):
                QMessageBox.warning(self.window(), 'Warning', 'No changes have been made.', QMessageBox.Ok)
                return
            print('Changed params: ', self.paramList.changedParams)
            self.uploadButton.setText('Confirm Upload')
            self.uploadUAVStep += 1
        elif self.uploadUAVStep == 1:  ## 'Confirm Upload' clicked
            print('Send parameters... ->', self.paramList.changedParams)
            self.uploadUAVStep = 0
            newVals = {}
            for p in self.paramList.changedParams:
                newVals[p[0]] = p[1]
            self.uploadNewParametersSignal.emit(newVals)
            self.close()

    def saveToFile(self):
        fileName = QFileDialog.getSaveFileName(self, 'Save Parameters', 'config.txt')
        self.paramList.collectTableBody()
        txt = open(fileName[0], 'w')
        for param, value in self.paramList.allParams.items():
            txt.write('{} = {}\n'.format(param, value))
        txt.close()
        self.close()
