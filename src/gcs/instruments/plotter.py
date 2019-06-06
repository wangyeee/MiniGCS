from PyQt5.QtChart import QChart, QChartView, QSplineSeries, QValueAxis
from PyQt5.QtWidgets import QWidget, QSplitter
from PyQt5.QtGui import QPainter
from PyQt5.QtCore import Qt
from time import time
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
            self.addSeries(self.data[key])
            self.data[key].attachAxis(self.axisX)
            self.data[key].attachAxis(self.axisY)
        ser = self.data[key]
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
        ser.append(msgTime, data)
        # self.scroll(data, 0)

    def __getMessageTime(self, msg):
        unused(msg)
        if self.time0 == 0:
            self.time0 = int(time())
            return 0.1
        return int(time()) - self.time0

class PlotterWindow(QSplitter):
    def __init__(self, title, parent = None):
        super().__init__(Qt.Vertical, parent)
        self.setWindowTitle(title)
        self.plotMessages = {
            'RAW_IMU' : None,
            'SCALED_IMU' : None
            }
        self.chart = PlotterPanel()
        self.chartView = QChartView(self.chart)
        self.chartView.setRenderHint(QPainter.Antialiasing)
        self.addWidget(self.chartView)

    def handleMavlinkMessage(self, msg):
        if self.isVisible():
            if msg.get_type() in self.plotMessages:
                self.chart.appendMAVLinkMessage(msg)
