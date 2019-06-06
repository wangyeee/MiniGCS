from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QTextOption, QTransform
from PyQt5.QtSvg import QGraphicsSvgItem, QSvgRenderer
from PyQt5.QtWidgets import (QGraphicsItem, QGraphicsScene, QGraphicsTextItem,
                             QGraphicsView, QGridLayout, QLabel, QLineEdit, QPushButton,
                             QVBoxLayout, QWidget)

from utils import unused

class Barometer(QWidget):
    def __init__(self, parent):
        super().__init__(parent)
        self.uas = None
        svgRenderer = QSvgRenderer('res/barometer.svg')

        bkgnd = QGraphicsSvgItem()
        bkgnd.setSharedRenderer(svgRenderer)
        bkgnd.setCacheMode(QGraphicsItem.NoCache)
        bkgnd.setElementId('background')

        scene = QGraphicsScene()
        scene.addItem(bkgnd)
        scene.setSceneRect(bkgnd.boundingRect())

        self.needle = QGraphicsSvgItem()
        self.needle.setSharedRenderer(svgRenderer)
        self.needle.setCacheMode(QGraphicsItem.NoCache)
        self.needle.setElementId('needle')
        self.needle.setParentItem(bkgnd)
        self.needle.setPos(bkgnd.boundingRect().width() / 2 - self.needle.boundingRect().width() / 2,
                           bkgnd.boundingRect().height() / 2 - self.needle.boundingRect().height() / 2)
        self.needle.setTransformOriginPoint(self.needle.boundingRect().width() / 2,
                                            self.needle.boundingRect().height() / 2)
        # textElement = svgRenderer.boundsOnElement('needle-text')
        self.digitalBaro = QGraphicsTextItem()
        self.digitalBaro.setDefaultTextColor(QColor(255,255,255))
        self.digitalBaro.document().setDefaultTextOption(QTextOption(Qt.AlignCenter))
        self.digitalBaro.setFont(QFont('monospace', 13, 60))
        self.digitalBaro.setParentItem(bkgnd)

        txm = QTransform()
        txm.translate(bkgnd.boundingRect().center().x(),
                      bkgnd.boundingRect().height() - 1.5 * self.digitalBaro.document().size().height())
        self.digitalBaro.setTransform(txm, False)

        view = QGraphicsView(scene)
        view.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        view.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        layout = QVBoxLayout()
        layout.addWidget(view)
        self.setLayout(layout)
        self.setBarometer(1000)

    def setBarometer(self, hbar):
        deg = ((hbar - 950) * 3 + 210) % 360
        self.needle.setRotation(deg)
        self.digitalBaro.setPlainText('{:.1f}'.format(hbar))
        self.digitalBaro.adjustSize()
        self.digitalBaro.setX(0 - self.digitalBaro.textWidth() / 2)

    def updateAirPressure(self, sourceUAS, timestamp, absPressure, diffPressure, temperature):
        unused(sourceUAS, timestamp, diffPressure, temperature)
        self.setBarometer(absPressure)

    def setActiveUAS(self, uas):
        uas.updateAirPressureSignal.connect(self.updateAirPressure)
        self.uas = uas

class BarometerConfigWindow(QWidget):

    updatePressureAltitudeReferenceSignal = pyqtSignal(float, float) # presRef, altiRef

    def __init__(self, presRef, altiRef, parent = None):
        super().__init__(parent)
        self.setWindowTitle('Pressure Altitude Reference')
        l = QGridLayout()
        row = 0
        l.addWidget(QLabel('Pressure Reference'), row, 0, 1, 1)
        self.presRefField = QLineEdit(str(presRef))
        l.addWidget(self.presRefField, row, 1, 1, 1)
        row += 1
        l.addWidget(QLabel('Altitude Reference'), row, 0, 1, 1)
        self.altiRefField = QLineEdit(str(altiRef))
        l.addWidget(self.altiRefField, row, 1, 1, 1)
        row += 1
        self.cancelButton = QPushButton('Cancel')
        self.cancelButton.clicked.connect(self.close)
        l.addWidget(self.cancelButton, row, 0, 1, 1)
        self.okButton = QPushButton('OK')
        self.okButton.clicked.connect(self.__updatePressureAltitudeReference)
        l.addWidget(self.okButton, row, 1, 1, 1)
        self.setLayout(l)

    def __updatePressureAltitudeReference(self):
        try:
            presRef = float(self.presRefField.text())
            altiRef = float(self.altiRefField.text())
            self.updatePressureAltitudeReferenceSignal.emit(presRef, altiRef)
        except ValueError:
            pass
        self.close()
