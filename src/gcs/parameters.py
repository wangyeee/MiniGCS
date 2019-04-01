# from PyQt5.QtCore import QObject, QRect, Qt, QVariant, pyqtSignal
# from PyQt5.QtWidgets import (QComboBox, QFormLayout, QHBoxLayout, QHeaderView,
#                              QLabel, QLineEdit, QMessageBox, QPushButton,
#                              QTableWidget, QTableWidgetItem, QWidget)

from PyQt5.QtWidgets import QWidget, QTableWidget, QVBoxLayout, QHeaderView, QTableWidgetItem
from pymavlink.dialects.v10 import common as mavlink

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

class ParameterList(QTableWidget):

    paramList = None

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
            name = QTableWidgetItem(param.param_id)
            value = QTableWidgetItem(str(param.param_value))
            pstr = None
            if param.param_type in PARAM_VALUE_TYPE_NAMES:
                pstr = PARAM_VALUE_TYPE_NAMES[param.param_type]
            else:
                pstr = 'UNKNOWN TYPE: {}'.format(param.param_type)
            ptype = QTableWidgetItem(pstr)
            self.setItem(rowNumber, 0, name)
            self.setItem(rowNumber, 1, value)
            self.setItem(rowNumber, 2, ptype)
            rowNumber += 1
        self.resizeRowsToContents()

class ParameterPanel(QWidget):

    def __init__(self, params, parent = None):
        super().__init__(parent)
        l = QVBoxLayout()
        self.paramList = ParameterList(params, parent)
        l.addWidget(self.paramList)
        self.setLayout(l)
