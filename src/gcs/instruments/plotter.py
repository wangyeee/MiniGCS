from time import time

from PyQt5.QtChart import QChart, QChartView, QSplineSeries, QValueAxis
from PyQt5.QtCore import Qt, QVariant, pyqtSignal
from PyQt5.QtGui import QPainter
from PyQt5.QtWidgets import (QSplitter, QTreeWidget, QTreeWidgetItem,
                             QVBoxLayout, QWidget)

from utils import unused

class PlotterPanel(QChart):

    def __init__(self, parent = None):
        super().__init__(parent)
        self.xRange = (0, 1)
        self.yRange = (0, 1)
        self.axisX = QValueAxis()
        self.axisY = QValueAxis()
        self.axisX.setRange(self.xRange[0], self.xRange[1])
        self.axisY.setRange(self.yRange[0], self.yRange[1])
        # name(msg_type.attr) = QSplineSeries
        self.data = {}
        self.visibleData = {}
        self.addAxis(self.axisX, Qt.AlignBottom)
        self.addAxis(self.axisY, Qt.AlignLeft)
        self.time0 = 0
        self.extraYScale = 0.2

    def appendMAVLinkMessage(self, msg):
        # print('Add msg:', msg)
        tm = self.__getMessageTime(msg)
        for attr in msg.get_fieldnames():
            k = '{}.{}'.format(msg.get_type(), attr)
            self.appendData(k, msg.format_attr(attr), tm)

    def appendData(self, key, data, msgTime):
        if key not in self.data:
            self.data[key] = QSplineSeries()
            self.data[key].setName(key)
        ser = self.data[key]
        ser.append(msgTime, data)
        if key in self.visibleData:
            r0 = False
            if msgTime < self.xRange[0]:
                self.xRange = (msgTime, self.xRange[1])
                r0 = True
            elif msgTime > self.xRange[1]:
                self.xRange = (self.xRange[0], msgTime)
                r0 = True
            if r0:
                self.axisX.setRange(self.xRange[0], self.xRange[1])
                r0 = False
            if data < self.yRange[0]:
                self.yRange = (data, self.yRange[1])
                r0 = True
            elif data > self.yRange[1]:
                self.yRange = (self.yRange[0], data)
                r0 = True
            if r0:
                # add extra space for Y axis
                ext = self.extraYScale * (self.yRange[1] - self.yRange[0])
                self.yRange = (int(self.yRange[0] - ext), int(self.yRange[1] + ext))
                self.axisY.setRange(self.yRange[0], self.yRange[1])

    def toggleDataVisibility(self, key, disp):
        if disp == 0:
            if key in self.visibleData:
                del self.visibleData[key]
                self.removeSeries(self.data[key])
        else:
            if key not in self.visibleData:
                self.visibleData[key] = self.data[key]
                self.addSeries(self.data[key])
                self.data[key].attachAxis(self.axisX)
                self.data[key].attachAxis(self.axisY)

    def __getMessageTime(self, msg):
        unused(msg)
        if self.time0 == 0:
            self.time0 = int(time()) * 50
            return 1
        return int(time()) * 50 - self.time0

class PlotItemMenu(QWidget):

    messageKeyRole = Qt.UserRole + 1
    plotDataSignal = pyqtSignal(object, int)

    def __init__(self, parent = None):
        super().__init__(parent)
        self.setLayout(QVBoxLayout())
        self.tree = QTreeWidget()
        self.tree.setHeaderHidden(True)
        self.tree.setColumnCount(1)
        self.tree.itemChanged.connect(self.__toggleDataPlot)
        self.layout().addWidget(self.tree)
        self.rootItems = []

    def addMAVLinkMessage(self, msg):
        tp = msg.get_type()
        if tp not in self.rootItems:
            self.rootItems.append(tp)
            rt = QTreeWidgetItem(self.tree)
            rt.setText(0, tp)
            for attr in msg.get_fieldnames():
                chld = QTreeWidgetItem()
                chld.setText(0, attr)
                chld.setFlags(chld.flags() | Qt.ItemIsUserCheckable | Qt.ItemIsSelectable)
                chld.setCheckState(0, Qt.Unchecked)
                chld.setData(0, PlotItemMenu.messageKeyRole, QVariant('{}.{}'.format(tp, attr)))
                rt.addChild(chld)
            self.tree.addTopLevelItem(rt)

    def __toggleDataPlot(self, item, col):
        self.plotDataSignal.emit(item.data(col, PlotItemMenu.messageKeyRole), item.checkState(col))

class PlotterWindow(QSplitter):
    def __init__(self, title, parent = None):
        super().__init__(Qt.Horizontal, parent)
        self.setWindowTitle(title)
        self.plotMessages = {
            'RAW_IMU' : None,
            'SCALED_IMU' : None,
            'GPS_RAW_INT' : None,
            'ATTITUDE' : None,
            'SCALED_PRESSURE' : None
        }
        self.chart = PlotterPanel()
        self.chartView = QChartView(self.chart)
        self.chartView.setRenderHint(QPainter.Antialiasing)
        self.plotControl = PlotItemMenu()
        self.plotControl.plotDataSignal.connect(self.chart.toggleDataVisibility)
        self.addWidget(self.plotControl)
        self.addWidget(self.chartView)

    def handleMavlinkMessage(self, msg):
        if self.isVisible():
            if msg.get_type() in self.plotMessages:
                self.plotControl.addMAVLinkMessage(msg)
                self.chart.appendMAVLinkMessage(msg)
