# from PyQt5.QtCore import QObject, QRect, Qt, QVariant, pyqtSignal
# from PyQt5.QtWidgets import (QComboBox, QFormLayout, QHBoxLayout, QHeaderView,
#                              QLabel, QLineEdit, QMessageBox, QPushButton,
#                              QTableWidget, QTableWidgetItem, QWidget)

from PyQt5.QtWidgets import QWidget, QTableWidget, QVBoxLayout, QHeaderView, QTableWidgetItem

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
        self.resizeRowsToContents()
        self.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)

    def createTableBody(self):
        rowNumber = 0
        self.setRowCount(len(self.paramList))
        for param in self.paramList:
            # print(param)
            name = QTableWidgetItem(param.param_id)
            value = QTableWidgetItem(str(param.param_value))
            ptype = QTableWidgetItem(str(param.param_type))
            print('create row#{}: {} = {} ({})'.format(rowNumber, name.text(), value.text(), ptype.text()))
            self.setItem(rowNumber, 0, name)
            self.setItem(rowNumber, 1, value)
            self.setItem(rowNumber, 2, ptype)
            rowNumber += 1

class ParameterPanel(QWidget):

    def __init__(self, params, parent = None):
        super().__init__(parent)
        l = QVBoxLayout()
        self.paramList = ParameterList(params, parent)
        l.addWidget(self.paramList)
        self.setLayout(l)
