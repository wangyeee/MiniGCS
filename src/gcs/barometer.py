from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor, QFont, QTextOption, QTransform
from PyQt5.QtSvg import QGraphicsSvgItem, QSvgRenderer
from PyQt5.QtWidgets import (QGraphicsItem, QGraphicsScene, QGraphicsTextItem,
                             QGraphicsView, QLabel, QVBoxLayout, QWidget)

class Barometer(QWidget):
    def __init__(self):
        super().__init__()
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

        textElement = svgRenderer.boundsOnElement('needle-text')

        print(textElement.center())

        self.digitalBaro = QGraphicsTextItem()
        self.digitalBaro.setDefaultTextColor(QColor(255,255,255))
        self.digitalBaro.document().setDefaultTextOption(QTextOption(Qt.AlignCenter))
        self.digitalBaro.setFont(QFont('monospace', 13, 60))
        self.digitalBaro.setParentItem(bkgnd)

        '''
        https://github.com/TauLabs/TauLabs/blob/next/ground/gcs/src/plugins/dial/dialgadgetwidget.cpp
        QMatrix textMatrix = m_renderer->matrixForElement("needle-text");
        qreal startX = textMatrix.mapRect(m_renderer->boundsOnElement("needle-text")).x();
        qreal startY = textMatrix.mapRect(m_renderer->boundsOnElement("needle-text")).y();
        QTransform matrix;
        matrix.translate(startX,startY);
        m_text1 = new QGraphicsTextItem("0.00");
        m_text1->setDefaultTextColor(QColor("White"));
        m_text1->setTransform(matrix,false);
        l_scene->addItem(m_text1);
        '''
        txm = QTransform()
        txm.translate(bkgnd.boundingRect().center().x(),
                      bkgnd.boundingRect().height() - 1.5 * self.digitalBaro.document().size().height())
        self.digitalBaro.setTransform(txm, False)

        view = QGraphicsView(scene)
        view.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        view.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        layout = QVBoxLayout()
        layout.addWidget(view)
        super().setLayout(layout)

    def setBarometer(self, hbar):
        deg = ((hbar - 950) * 3 + 210) % 360
        self.needle.setRotation(deg)
        self.digitalBaro.setPlainText(str(hbar))
        self.digitalBaro.adjustSize()
        self.digitalBaro.setX(0 - self.digitalBaro.textWidth() / 2)
