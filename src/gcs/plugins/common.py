from PyQt5.QtWidgets import QWidget, QGridLayout, QPushButton, QMessageBox, QPlainTextEdit
from PyQt5.QtCore import pyqtSignal, Qt
from pymavlink.mavutil import mavlink
import time

class AbstractControlPanel(QWidget):

    mavlinkTxSignal = pyqtSignal(object)  # pass a mavlink message object
    isConnected = False

    def tabName(self):
        return 'Tools'

    def processMavlinkMessage(self, msg):
        '''Will be invoked by other components.'''
        if msg != None:
            if msg.get_type() in self.registerMavlinkMessageListeners():
                self.mavlinkMessageReceived(msg)

    def registerMavlinkMessageListeners(self):
        '''Each sub-class should return list of message types
        which will be passed to __mavlinkMessageReceived method.'''
        return []

    def mavlinkMessageReceived(self, msg):
        # print('Error, sub-class must implement this method to process mavlink message:', msg)
        pass

class GenericControlPanel(AbstractControlPanel):

    autoPilotRebootSignal = pyqtSignal()
    textMessageConsole = None
    __updateTextConsoleSignal = pyqtSignal(object)

    def __init__(self, parent = None):
        super().__init__(parent)
        l = QGridLayout()
        row = 0
        self.rebootAutoPilotButton = QPushButton('Reboot AutoPilot')
        self.rebootAutoPilotButton.setStyleSheet('background-color: red')
        self.rebootAutoPilotButton.clicked.connect(self.__rebootAutoPilot)
        l.addWidget(self.rebootAutoPilotButton, row, 0, 1, 1, Qt.AlignLeft)
        row += 1
        self.textMessageConsole = QPlainTextEdit(self)
        self.textMessageConsole.setReadOnly(True)
        self.textMessageConsole.document().setMaximumBlockCount(100)
        self.__updateTextConsoleSignal.connect(self.__textMessagePrint)
        l.addWidget(self.textMessageConsole, row, 0, 5, 5)
        self.setLayout(l)

    def tabName(self):
        return 'Mavlink'

    def registerMavlinkMessageListeners(self):
        return ['STATUSTEXT']

    def mavlinkMessageReceived(self, msg):
        if msg.get_type() == 'STATUSTEXT':
            self.__updateTextConsoleSignal.emit(msg.text)

    def __textMessagePrint(self, msg: str):
        self.textMessageConsole.insertPlainText('{}: {}'.format(time.strftime('%H:%M:%S', time.localtime()), msg))
        bar = self.textMessageConsole.verticalScrollBar()
        bar.setValue(bar.maximum())

    def __rebootAutoPilot(self):
        if self.isConnected:
            cfm = QMessageBox.question(self.window(),
                                       'Reboot AutoPilot',
                                       'The autopilot will reboot, continue?',
                                       QMessageBox.Yes, QMessageBox.No)
            if cfm == QMessageBox.Yes:
                msg = mavlink.MAVLink_command_long_message(255, 0, mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0,
                                                           1, 0, 0, 0, 0, 0, 0)
                self.autoPilotRebootSignal.emit()  # signal other component the reboot event
                self.mavlinkTxSignal.emit(msg)
