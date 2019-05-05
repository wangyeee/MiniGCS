from PyQt5.QtCore import Qt  # , pyqtSignal
from PyQt5.QtWidgets import QGridLayout, QLabel
from plugins.common import AbstractControlPanel

class AutoQuadControlPanel(AbstractControlPanel):

    def __init__(self, parent = None):
        super().__init__(parent)
        l = QGridLayout()
        row = 0
        l.addWidget(QLabel('AutoQuad Tools'), row, 0, 1, 3, Qt.AlignLeft)
        row += 1
        self.setLayout(l)

    def tabName(self):
        return 'AutoQuad'

    def registerMavlinkMessageListeners(self):
        return ['AQ_TELEMETRY_F', 'AQ_ESC_TELEMETRY']
