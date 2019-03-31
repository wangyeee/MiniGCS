# from PyQt5.QtCore import QObject, QRect, Qt, QVariant, pyqtSignal
# from PyQt5.QtWidgets import (QComboBox, QFormLayout, QHBoxLayout, QHeaderView,
#                              QLabel, QLineEdit, QMessageBox, QPushButton,
#                              QTableWidget, QTableWidgetItem, QWidget)

from PyQt5.QtWidgets import QWidget

class ParameterPanel(QWidget):

    parameters = []

    def __init__(self, params, parent = None):
        super().__init__(parent)
        self.parameters += params
