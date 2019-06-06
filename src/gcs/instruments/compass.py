import math

from PyQt5.QtCore import Qt
from PyQt5.QtSvg import QGraphicsSvgItem, QSvgRenderer
from PyQt5.QtWidgets import (QGraphicsItem, QGraphicsScene, QGraphicsView,
                             QVBoxLayout, QWidget)
from utils import unused

class Compass(QWidget):
    def __init__(self, parent):
        super().__init__(parent)
        self.uas = None
        svgRenderer = QSvgRenderer('res/compass.svg')
        self.compass = QGraphicsSvgItem()
        self.compass.setSharedRenderer(svgRenderer)
        self.compass.setCacheMode(QGraphicsItem.NoCache)
        self.compass.setElementId('needle')

        center = self.compass.boundingRect().center()
        self.compass.setTransformOriginPoint(center.x(), center.y())

        bkgnd = QGraphicsSvgItem()
        bkgnd.setSharedRenderer(svgRenderer)
        bkgnd.setCacheMode(QGraphicsItem.NoCache)
        bkgnd.setElementId('background')

        self.compass.setPos(bkgnd.boundingRect().width() / 2 - self.compass.boundingRect().width() / 2,
                            bkgnd.boundingRect().height() / 2 - self.compass.boundingRect().height() / 2)

        self.compass.setTransformOriginPoint(self.compass.boundingRect().width() / 2, self.compass.boundingRect().height() / 2)

        fregnd = QGraphicsSvgItem()
        fregnd.setSharedRenderer(svgRenderer)
        fregnd.setCacheMode(QGraphicsItem.NoCache)
        fregnd.setElementId('foreground')

        scene = QGraphicsScene()
        scene.addItem(bkgnd)
        scene.addItem(self.compass)
        scene.addItem(fregnd)
        scene.setSceneRect(bkgnd.boundingRect())

        view = QGraphicsView(scene)
        view.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        view.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        layout = QVBoxLayout()
        layout.addWidget(view)
        super().setLayout(layout)

    def setHeading(self, hdr):
        if math.isnan(hdr) == False:
            hdr *= 180.0 / math.pi
            self.compass.setRotation(360.0 - hdr)

    def updateAttitude(self, sourceUAS, timestamp, roll, pitch, yaw):
        unused(sourceUAS, timestamp, roll, pitch)
        self.setHeading(yaw)

    def setActiveUAS(self, uas):
        uas.updateAttitudeSignal.connect(self.updateAttitude)
        self.uas = uas
