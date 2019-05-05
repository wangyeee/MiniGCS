from PyQt5.QtWidgets import QWidget

class AbstractControlPanel(QWidget):

    def __init__(self, parent = None):
        super().__init__(parent)
        self.setVisible(False)

    def tabName(self):
        return 'Tools'
