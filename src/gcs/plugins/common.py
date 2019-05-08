from PyQt5.QtWidgets import QWidget
from PyQt5.QtCore import pyqtSignal

class AbstractControlPanel(QWidget):

    mavlinkTxSignal = pyqtSignal(object)  # pass a mavlink message object

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

    def tabName(self):
        return 'Mavlink'

    def registerMavlinkMessageListeners(self):
        return ['STATUSTEXT']

    def mavlinkMessageReceived(self, msg):
        if msg.get_type() == 'STATUSTEXT':
            print(msg)
