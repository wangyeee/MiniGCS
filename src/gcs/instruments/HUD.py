from math import cos, isinf, isnan
from math import pi as M_PI
from math import sin, sqrt

from PyQt5.QtCore import (QPoint, QPointF, QRectF, QSize, QSizeF,
                          QStandardPaths, Qt, QTimer)
from PyQt5.QtGui import (QBrush, QColor, QFont, QFontDatabase, QFontMetrics,
                         QImage, QPainter, QPen, QPixmap, QPolygon, QPolygonF,
                         QVector3D)
from PyQt5.QtOpenGL import QGLWidget
from PyQt5.QtWidgets import (QAction, QFileDialog, QLabel, QMenu, QSizePolicy,
                             QWidget, QVBoxLayout)
from utils import unused

class HUDWindow(QWidget):

    def __init__(self, hud = None, parent = None):
        super().__init__(parent)
        self.setWindowTitle('HUD')
        self.setMinimumSize(640, 480)
        if hud == None:
            self.hud = HUD(self)
        else:
            self.hud = hud
        l = QVBoxLayout()
        l.addWidget(self.hud)
        self.setLayout(l)

class HUD(QLabel):

    DEFAULT_VWIDTH = 320.0

    def __init__(self, parent = None):
        super().__init__(parent)
        self.yawInt = 0.0
        self.mode = 'UNKNOWN MODE'
        self.state = 'UNKNOWN STATE'
        self.fuelStatus = '00.0V (00m:00s)'
        self.xCenterOffset = 0.0
        self.yCenterOffset = 0.0
        self.vwidth = HUD.DEFAULT_VWIDTH
        self.vheight = 150.0
        self.vGaugeSpacing = 65.0
        self.vPitchPerDeg = 6.0 # 4 mm y translation per degree
        self.rawBuffer1 = None
        self.rawBuffer2 = None
        self.rawImage = None
        self.rawLastIndex = 0
        self.rawExpectedBytes = 0
        self.bytesPerLine = 1
        self.imageStarted = False
        self.receivedDepth = 8
        self.receivedChannels = 1
        self.receivedWidth = 640
        self.receivedHeight = 480
        self.warningBlinkRate = 5

        self.noCamera = True
        self.hardwareAcceleration = True
        self.strongStrokeWidth = 1.5
        self.normalStrokeWidth = 1.0
        self.fineStrokeWidth = 0.5
        self.waypointName = ''
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.rollLP = 0.0
        self.pitchLP = 0.0
        self.yawLP = 0.0
        self.yawDiff = 0.0
        self.xPos = 0.0
        self.yPos = 0.0
        self.zPos = 0.0
        self.xSpeed = 0.0
        self.ySpeed = 0.0
        self.zSpeed = 0.0
        self.lastSpeedUpdate = 0
        self.totalSpeed = 0.0
        self.totalAcc = 0.0
        self.lat = 0.0
        self.lon = 0.0
        self.alt = 0.0
        self.desiredRoll = 0.0
        self.desiredPitch = 0.0
        self.desiredHeading = 0.0
        self.targetBearing = 0.0
        self.wpDist = 0.0
        self.load = 0.0
        self.videoRecording = ''
        self.nextOfflineImage = ''
        self.HUDInstrumentsEnabled = True
        self.videoEnabled = False
        self.imageLoggingEnabled = False
        self.imageLogCounter = 0
        self.imageLogDirectory = None
        self.xImageFactor = 1.0
        self.yImageFactor = 1.0
        self.imageRequested = False

        self.videoSrc = None
        self.videoStarted = False
        self.updateInterval = 100

        self.defaultColor = None
        self.setPointColor = None
        self.warningColor = None
        self.criticalColor = None
        self.infoColor = None
        self.fuelColor = None
        self.DEFAULT_BACKGROUND_IMAGE = None

        self.enableHUDAction: QAction = None
        self.enableVideoAction: QAction = None
        self.selectOfflineDirectoryAction: QAction = None

        self.attitudes = {}
        self.uas = None
        self.refreshTimer = QTimer(self)
        # Set auto fill to False
        self.setAutoFillBackground(False)

        # Set minimum size
        self.setMinimumSize(80, 60)
        # Set preferred size
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.scalingFactor = self.width()/self.vwidth

        # Set up the initial color theme. This can be updated by a styleChanged
        # signal from MainWindow.
        self.styleChanged()
        self.glImage = self.DEFAULT_BACKGROUND_IMAGE

        # Refresh timer
        self.refreshTimer.setInterval(self.updateInterval)
        self.refreshTimer.timeout.connect(self.repaint)

        # Resize to correct size and fill with image
        QWidget.resize(self, self.width(), self.height())

        fontDatabase = QFontDatabase()
        fontFileName = './res/vera.ttf' # Font file is part of the QRC file and compiled into the app
        fontFamilyName = 'Bitstream Vera Sans'

        fontDatabase.addApplicationFont(fontFileName)
        font = fontDatabase.font(fontFamilyName, 'Roman', max(5, int(10.0 * self.scalingFactor * 1.2 + 0.5)))
        if font == None:
            print('ERROR! FONT NOT LOADED!')
        if font.family() != fontFamilyName:
            print('ERROR! WRONG FONT LOADED: {}'.format(fontFamilyName))
        self.createActions()

    def sizeHint(self):
        return QSize(self.width(), (self.width()*3.0)/4)

    def styleChanged(self, newTheme = 0):
        unused(newTheme)
        # Generate a background image that's dependent on the current color scheme.
        fill = QImage(self.width(), self.height(), QImage.Format_Indexed8)
        fill.fill(0)
        self.DEFAULT_BACKGROUND_IMAGE = QGLWidget.convertToGLFormat(fill)
        self.defaultColor = QColor(0x66, 0xff, 0x00)  # bright green
        self.setPointColor = QColor(0x82, 0x17, 0x82)
        self.warningColor = QColor(0xff, 0xff, 0x00)
        self.criticalColor = QColor(0xff, 0x00, 0x00)
        self.infoColor = self.defaultColor
        self.fuelColor = self.criticalColor

    def showEvent(self, event):
        # React only to internal (pre-display) events
        QWidget.showEvent(self, event)
        self.styleChanged()
        self.refreshTimer.start(self.updateInterval)
        if self.videoSrc != None:
            if self.videoStarted == False:
                print('starting video...')
                self.videoStarted = True
                self.videoSrc.start()
            self.videoSrc.pauseVideo(False)

    def hideEvent(self, event):
        # React only to internal (pre-display) events
        self.refreshTimer.stop()
        QWidget.hideEvent(self, event)
        if self.videoSrc != None:
            self.videoSrc.pauseVideo(True)

    def resizeEvent(self, event):
        QWidget.resizeEvent(self, event)
        self.scalingFactor = self.width() / HUD.DEFAULT_VWIDTH
        self.vwidth = self.width() / self.scalingFactor
        self.vheight = self.height() / self.scalingFactor
        self.styleChanged()

    def contextMenuEvent(self, event):
        menu = QMenu(self)
        # Update actions
        self.enableHUDAction.setChecked(self.HUDInstrumentsEnabled)
        self.enableVideoAction.setChecked(self.videoEnabled)

        menu.addAction(self.enableHUDAction)
        menu.addAction(self.enableVideoAction)
        menu.addAction(self.selectOfflineDirectoryAction)
        menu.exec(event.globalPos())

    def createActions(self):
        self.enableHUDAction = QAction('Enable HUD', self)
        self.enableHUDAction.setStatusTip('Show the HUD instruments in this window')
        self.enableHUDAction.setCheckable(True)
        self.enableHUDAction.setChecked(self.HUDInstrumentsEnabled)
        self.enableHUDAction.triggered.connect(self.enableHUDInstruments)

        self.enableVideoAction = QAction('Enable Video Live feed', self)
        self.enableVideoAction.setStatusTip('Show the video live feed')
        self.enableVideoAction.setCheckable(True)
        self.enableVideoAction.setChecked(self.videoEnabled)
        self.enableVideoAction.triggered.connect(self.enableVideo)

        self.selectOfflineDirectoryAction = QAction('Load image log', self)
        self.selectOfflineDirectoryAction.setStatusTip('Load previously logged images into simulation / replay')
        self.selectOfflineDirectoryAction.triggered.connect(self.selectOfflineDirectory)

    def setActiveUAS(self, uas):
        uas.updateAttitudeSignal.connect(self.updateAttitude)
        uas.updateBatterySignal.connect(self.updateBattery)
        uas.updateGlobalPositionSignal.connect(self.updateGlobalPosition)
        uas.updateAirSpeedSignal.connect(self.updateSpeed)
        uas.updateGroundSpeedSignal.connect(self.updateGroundSpeed)
        uas.updateVelocitySignal.connect(self.updateVelocity)
        uas.updateNavigationControllerOutputSignal.connect(self.updateNavigationControllerOutput)
        self.uas = uas

    def setVideoSource(self, videoSrc):
        videoSrc.newFrameAvailable.connect(self.setImageExternal)
        self.videoSrc = videoSrc

    def updateAttitude(self, uas, timestamp, roll, pitch, yaw):
        unused(uas, timestamp)
        if isnan(roll) == False and isinf(roll) == False \
        and isnan(pitch) == False and isinf(pitch)== False \
        and isnan(yaw) == False and isinf(yaw) == False:
            self.roll = roll
            self.pitch = pitch*3.35 # Constant here is the 'focal length' of the projection onto the plane
            self.yaw = yaw
            self.attitudes[0] = QVector3D(roll, pitch*3.35, yaw)

    def updateComponentAttitude(self, uas, timestamp, component, roll, pitch, yaw):
        unused(uas, timestamp)
        if isnan(roll) == False and isinf(roll) == False \
        and isnan(pitch) == False and isinf(pitch)== False \
        and isnan(yaw) == False and isinf(yaw) == False:
            self.attitudes[component] = QVector3D(roll, pitch*3.35, yaw) # Constant here is the 'focal length' of the projection onto the plane

    def updateBattery(self, uas, timestamp, voltage, current, percent):
        unused(uas, timestamp, current)
        self.fuelStatus = 'BAT [{}% | {:.1f}V]'.format(percent, voltage)
        if percent < 20.0:
            self.fuelColor = self.warningColor
        elif percent < 10.0:
            self.fuelColor = self.criticalColor
        else:
            self.fuelColor = self.infoColor

    def receiveHeartbeat(self, uas):
        unused(uas)

    def updateThrust(self, uas, thrust):
        unused(uas, thrust)
        #updateValue(uas, "thrust", thrust, MG.TIME.getGroundTimeNow())

    def updateLocalPosition(self, uas, timestamp, x, y, z):
        unused(uas, timestamp)
        self.xPos = x
        self.yPos = y
        self.zPos = z

    def updateGlobalPosition(self, uas, timestamp, lat,  lon, altitude):
        unused(uas, timestamp)
        self.lat = lat
        self.lon = lon
        self.alt = altitude

    def updateVelocity(self, uas, timestamp, x, y, z):
        unused(uas)
        self.xSpeed = x
        self.ySpeed = y
        self.zSpeed = z
        newTotalSpeed = sqrt(self.xSpeed*self.xSpeed + self.ySpeed*self.ySpeed + self.zSpeed*self.zSpeed)
        self.totalAcc = (newTotalSpeed - self.totalSpeed) / ((self.lastSpeedUpdate - timestamp)/1000.0)
        self.totalSpeed = newTotalSpeed
        self.lastSpeedUpdate = timestamp

    def updateSpeed(self, uas, timestamp, speed):
        unused(uas)
        self.totalAcc = (speed - self.totalSpeed) / ((self.lastSpeedUpdate - timestamp)/1000.0)
        self.totalSpeed = speed
        self.lastSpeedUpdate = timestamp

    def updateGroundSpeed(self, uas, timestamp, gndspd):
        unused(uas)
        # only update if we haven't gotten a full 3D speed update in a while
        if timestamp - self.lastSpeedUpdate > 1e6:
            self.totalAcc = (gndspd - self.totalSpeed) / ((self.lastSpeedUpdate - timestamp)/1000.0)
            self.totalSpeed = gndspd
            self.lastSpeedUpdate = timestamp

    def updateState(self, uas, state):
        '''
        Updates the current system state, but only if the uas matches the currently monitored uas.
        :param uas: the system the state message originates from
        :param state: short state text, displayed in HUD
        '''
        unused(uas)
        # Only one UAS is connected at a time
        self.state = state

    def updateMode(self, uas, mode):
        '''
        Updates the current system mode, but only if the uas matches the currently monitored uas.
        :param uas: the system the state message originates from
        :param mode: short mode text, displayed in HUD
        '''
        unused(uas)
        # Only one UAS is connected at a time
        self.mode = mode

    def updateLoad(self, uas, load):
        unused(uas)
        self.load = load
        # updateValue(uas, "load", load, MG.TIME.getGroundTimeNow())

    def updateNavigationControllerOutput(self, uas, desiredRoll, desiredPitch, desiredHeading, targetBearing, wpDist):
        unused(uas)
        self.desiredRoll = desiredRoll
        self.desiredPitch = desiredPitch
        self.desiredHeading = desiredHeading
        self.targetBearing = targetBearing
        self.wpDist = wpDist

    def refToScreenX(self, x):
        return self.scalingFactor * x

    def refToScreenY(self, y):
        return self.scalingFactor * y

    def paintText(self, text, color, fontSize, refX, refY, painter):
        '''
        Paint text on top of the image and OpenGL drawings
        :param text: chars to write
        :param color: text color
        :param fontSize: text size in mm
        :param refX: position in reference units (mm of the real instrument). This is relative to the measurement unit position, NOT in pixels.
        :param refY: position in reference units (mm of the real instrument). This is relative to the measurement unit position, NOT in pixels.
        '''
        prevPen = painter.pen()
        pPositionX = self.refToScreenX(refX) - (fontSize*self.scalingFactor*0.072)
        pPositionY = self.refToScreenY(refY) - (fontSize*self.scalingFactor*0.212)

        font = QFont('Bitstream Vera Sans')
        # Enforce minimum font size of 5 pixels
        fSize = max(5, int(fontSize*self.scalingFactor*1.26))
        font.setPixelSize(fSize)

        metrics = QFontMetrics(font)
        border = max(4, metrics.leading())
        rect = metrics.boundingRect(0, 0, self.width() - 2*border, int(self.height()*0.125),
                                    Qt.AlignLeft | Qt.TextWordWrap, text)
        painter.setPen(color)
        painter.setFont(font)
        painter.setRenderHint(QPainter.TextAntialiasing)
        painter.drawText(pPositionX, pPositionY,
                         rect.width(), rect.height(),
                         Qt.AlignCenter | Qt.TextWordWrap, text)
        painter.setPen(prevPen)

    def paintRollPitchStrips(self):
        pass

    def paintEvent(self, event):
        unused(event)
        if self.isVisible():

            # Read out most important values to limit hash table lookups
            # Low-pass roll, pitch and yaw
            self.rollLP = self.roll#rollLP * 0.2f + 0.8f * roll
            self.pitchLP = self.pitch#pitchLP * 0.2f + 0.8f * pitch
            self.yawLP = self.yaw if isinf(self.yaw) == False and isnan(self.yaw) == False else self.yawLP#yawLP * 0.2f + 0.8f * yaw

            # Translate for yaw
            maxYawTrans = 60.0

            newYawDiff = self.yawDiff
            if isinf(newYawDiff):
                newYawDiff = self.yawDiff
            if newYawDiff > M_PI:
                newYawDiff = newYawDiff - M_PI

            if newYawDiff < -M_PI:
                newYawDiff = newYawDiff + M_PI

            newYawDiff = self.yawDiff * 0.8 + newYawDiff * 0.2

            self.yawDiff = newYawDiff

            self.yawInt += newYawDiff

            if self.yawInt > M_PI:
                self.yawInt = M_PI
            if self.yawInt < -M_PI:
                self.yawInt = -M_PI

            yawTrans = self.yawInt * maxYawTrans
            self.yawInt *= 0.6

            if (yawTrans < 5.0) and (yawTrans > -5.0):
                yawTrans = 0

            # Negate to correct direction
            yawTrans = -yawTrans
            yawTrans = 0
            #qDebug() << "yaw translation" << yawTrans << "integral" << yawInt << "difference" << yawDiff << "yaw" << yaw

            # And if either video or the data stream is enabled, draw the next frame.
            if self.videoEnabled:
                self.xImageFactor = self.width() / float(self.glImage.width())
                self.yImageFactor = self.height() / float(self.glImage.height())

            painter = QPainter()
            painter.begin(self)
            painter.setRenderHint(QPainter.Antialiasing, True)
            painter.setRenderHint(QPainter.HighQualityAntialiasing, True)
            pmap = QPixmap.fromImage(self.glImage).scaledToWidth(self.width())
            painter.drawPixmap(0, (self.height() - pmap.height()) / 2, pmap)

            # END OF OPENGL PAINTING

            if self.HUDInstrumentsEnabled:
                #glEnable(GL_MULTISAMPLE)
                # QT PAINTING
                #makeCurrent()

                painter.translate((self.vwidth/2.0+self.xCenterOffset)*self.scalingFactor, (self.vheight/2.0+self.yCenterOffset)*self.scalingFactor)
                # COORDINATE FRAME IS NOW (0,0) at CENTER OF WIDGET
                # Draw all fixed indicators
                # BATTERY
                self.paintText(self.fuelStatus, self.fuelColor, 6.0, (-self.vwidth/2.0) + 10, -self.vheight/2.0 + 6, painter)
                # Waypoint
                self.paintText(self.waypointName, self.defaultColor, 6.0, (-self.vwidth/3.0) + 10, +self.vheight/3.0 + 15, painter)

                linePen = QPen(Qt.SolidLine)
                linePen.setWidth(self.refLineWidthToPen(1.0))
                linePen.setColor(self.defaultColor)
                painter.setBrush(Qt.NoBrush)
                painter.setPen(linePen)

                # YAW INDICATOR
                #
                #      .
                #    .   .
                #   .......
                #
                _yawIndicatorWidth = 12.0
                _yawIndicatorY = self.vheight/2.0 - 15.0
                yawIndicator = QPolygon(4)
                yawIndicator.setPoint(0, QPoint(self.refToScreenX(0.0), self.refToScreenY(_yawIndicatorY)))
                yawIndicator.setPoint(1, QPoint(self.refToScreenX(_yawIndicatorWidth/2.0), self.refToScreenY(_yawIndicatorY+_yawIndicatorWidth)))
                yawIndicator.setPoint(2, QPoint(self.refToScreenX(-_yawIndicatorWidth/2.0), self.refToScreenY(_yawIndicatorY+_yawIndicatorWidth)))
                yawIndicator.setPoint(3, QPoint(self.refToScreenX(0.0), self.refToScreenY(_yawIndicatorY)))
                painter.drawPolyline(yawIndicator)
                painter.setPen(linePen)
                # CENTER

                # HEADING INDICATOR
                #
                #    __      __
                #       \/\/
                #
                _hIndicatorWidth = 20.0
                _hIndicatorY = -25.0
                _hIndicatorYLow = _hIndicatorY + _hIndicatorWidth / 6.0
                _hIndicatorSegmentWidth = _hIndicatorWidth / 7.0
                hIndicator = QPolygon(7)
                hIndicator.setPoint(0, QPoint(self.refToScreenX(0.0-_hIndicatorWidth/2.0), self.refToScreenY(_hIndicatorY)))
                hIndicator.setPoint(1, QPoint(self.refToScreenX(0.0-_hIndicatorWidth/2.0+_hIndicatorSegmentWidth*1.75), self.refToScreenY(_hIndicatorY)))
                hIndicator.setPoint(2, QPoint(self.refToScreenX(0.0-_hIndicatorSegmentWidth*1.0), self.refToScreenY(_hIndicatorYLow)))
                hIndicator.setPoint(3, QPoint(self.refToScreenX(0.0), self.refToScreenY(_hIndicatorY)))
                hIndicator.setPoint(4, QPoint(self.refToScreenX(0.0+_hIndicatorSegmentWidth*1.0), self.refToScreenY(_hIndicatorYLow)))
                hIndicator.setPoint(5, QPoint(self.refToScreenX(0.0+_hIndicatorWidth/2.0-_hIndicatorSegmentWidth*1.75), self.refToScreenY(_hIndicatorY)))
                hIndicator.setPoint(6, QPoint(self.refToScreenX(0.0+_hIndicatorWidth/2.0), self.refToScreenY(_hIndicatorY)))
                painter.drawPolyline(hIndicator)

                # SETPOINT
                _centerWidth = 8.0
                painter.drawEllipse(
                    QPointF(self.refToScreenX(min(10.0, self.desiredRoll * 10.0)),
                            self.refToScreenY(min(10.0, self.desiredPitch * 10.0))),
                    self.refToScreenX(_centerWidth/2.0), self.refToScreenX(_centerWidth/2.0))

                _centerCrossWidth = 20.0
                # left
                painter.drawLine(QPointF(self.refToScreenX(-_centerWidth / 2.0), self.refToScreenY(0.0)),
                                 QPointF(self.refToScreenX(-_centerCrossWidth / 2.0), self.refToScreenY(0.0)))
                # right
                painter.drawLine(QPointF(self.refToScreenX(_centerWidth / 2.0), self.refToScreenY(0.0)),
                                 QPointF(self.refToScreenX(_centerCrossWidth / 2.0), self.refToScreenY(0.0)))
                # top
                painter.drawLine(QPointF(self.refToScreenX(0.0), self.refToScreenY(-_centerWidth / 2.0)),
                                 QPointF(self.refToScreenX(0.0), self.refToScreenY(-_centerCrossWidth / 2.0)))

                # COMPASS
                _compassY = -self.vheight/2.0 + 6.0
                compassRect = QRectF(QPointF(self.refToScreenX(-12.0), self.refToScreenY(_compassY)),
                                     QSizeF(self.refToScreenX(24.0), self.refToScreenY(12.0)))
                painter.setBrush(Qt.NoBrush)
                painter.setPen(linePen)
                painter.drawRoundedRect(compassRect, 3, 3)

                # YAW is in compass-human readable format, so 0 .. 360 deg.
                _yawDeg = (self.yawLP / M_PI) * 180.0
                if _yawDeg < 0:
                    _yawDeg += 360
                if _yawDeg > 360:
                    _yawDeg -= 360
                # final safeguard for really stupid systems
                _yawAngle = '{:3d}'.format(int(_yawDeg) % 360)
                self.paintText(_yawAngle, self.defaultColor, 8.5, -9.8, _compassY + 1.7, painter)

                painter.setBrush(Qt.NoBrush)
                painter.setPen(linePen)

                # CHANGE RATE STRIPS
                self.drawChangeRateStrip(-95.0, -60.0, 40.0, -10.0, 10.0, self.zSpeed, painter)

                # CHANGE RATE STRIPS
                self.drawChangeRateStrip(95.0, -60.0, 40.0, -10.0, 10.0, self.totalAcc, painter,True)

                # GAUGES

                # Left altitude gauge
                _gaugeAltitude = self.alt if self.alt != 0 else -self.zPos

                painter.setBrush(Qt.NoBrush)
                painter.setPen(linePen)

                self.drawChangeIndicatorGauge(-self.vGaugeSpacing, 35.0, 15.0, 10.0, _gaugeAltitude, self.defaultColor, painter, False)
                self.paintText('alt m', self.defaultColor, 5.5, -73.0, 50, painter)

                # Right speed gauge
                self.drawChangeIndicatorGauge(self.vGaugeSpacing, 35.0, 15.0, 10.0, self.totalSpeed, self.defaultColor, painter, False)
                self.paintText('v m/s', self.defaultColor, 5.5, 55.0, 50, painter)

                # Waypoint name
                if self.waypointName != '':
                    self.paintText(self.waypointName, self.defaultColor, 2.0, (-self.vwidth/3.0) + 10, +self.vheight/3.0 + 15, painter)

                # MOVING PARTS
                painter.translate(self.refToScreenX(yawTrans), 0)
                attColor = painter.pen().color()

                # Draw multi-component attitude
                for key in self.attitudes:
                    att = self.attitudes[key]
                    attColor = attColor.darker(200)
                    painter.setPen(attColor)
                    # Rotate view and draw all roll-dependent indicators
                    painter.rotate((att.x()/M_PI)* -180.0)
                    painter.translate(0, (-att.y()/M_PI)* -180.0 * self.refToScreenY(1.8))
                    #qDebug() << "ROLL" << roll << "PITCH" << pitch << "YAW DIFF" << valuesDot.value("roll", 0.0)
                    # PITCH
                    self.paintPitchLines(att.y(), painter)
                    painter.translate(0, -(-att.y()/M_PI)* -180.0 * self.refToScreenY(1.8))
                    painter.rotate(-(att.x()/M_PI)* -180.0)
            painter.end()

    def paintPitchLines(self, pitch, painter):
        _yDeg = self.vPitchPerDeg
        _lineDistance = 5.0 #/< One pitch line every 10 degrees
        _posIncrement = _yDeg * _lineDistance
        _posY = _posIncrement
        _posLimit = sqrt(pow(self.vwidth, 2.0) + pow(self.vheight, 2.0))*3.0

        _offsetAbs = pitch * _yDeg

        _offset = pitch
        if _offset < 0:
            _offset = -_offset
        _offsetCount = 0
        while _offset > _lineDistance:
            _offset -= _lineDistance
            _offsetCount += 1

        _iPos = int(0.5 + _lineDistance) #/< The first line
        _iNeg = int(-0.5 - _lineDistance) #/< The first line

        _offset *= _yDeg
        painter.setPen(self.defaultColor)

        _posY = -_offsetAbs + _posIncrement #+ 100# + _lineDistance

        while _posY < _posLimit:
            self.paintPitchLinePos('{:3d}'.format(_iPos), 0.0, -_posY, painter)
            _posY += _posIncrement
            _iPos += int(_lineDistance)

        # HORIZON
        #
        #    ------------    ------------
        #
        _pitchWidth = 30.0
        _pitchGap = _pitchWidth / 2.5
        _horizonColor = self.defaultColor
        _diagonal = sqrt(pow(self.vwidth, 2.0) + pow(self.vheight, 2.0))
        _lineWidth = self.refLineWidthToPen(0.5)

        # Left horizon
        self.drawLine(0.0-_diagonal, _offsetAbs, 0.0-_pitchGap/2.0, _offsetAbs, _lineWidth, _horizonColor, painter)
        # Right horizon
        self.drawLine(0.0+_pitchGap/2.0, _offsetAbs, 0.0+_diagonal, _offsetAbs, _lineWidth, _horizonColor, painter)

        _posY = _offsetAbs  + _posIncrement
        while _posY < _posLimit:
            self.paintPitchLineNeg('{:3d}'.format(_iNeg), 0.0, _posY, painter)
            _posY += _posIncrement
            _iNeg -= int(_lineDistance)

    def paintPitchLinePos(self, text, refPosX, refPosY, painter):
        _pitchWidth = 30.0
        _pitchGap = _pitchWidth / 2.5
        _pitchHeight = _pitchWidth / 12.0
        _textSize = _pitchHeight * 1.6
        _lineWidth = 1.5

        # Positive pitch indicator:
        #
        #      _______      _______
        #     |10                  |
        #

        # Left vertical line
        self.drawLine(refPosX-_pitchWidth/2.0, refPosY, refPosX-_pitchWidth/2.0, refPosY+_pitchHeight, _lineWidth, self.defaultColor, painter)
        # Left horizontal line
        self.drawLine(refPosX-_pitchWidth/2.0, refPosY, refPosX-_pitchGap/2.0, refPosY, _lineWidth, self.defaultColor, painter)
        # Text left
        self.paintText(text, self.defaultColor, _textSize, refPosX-_pitchWidth/2.0 + 0.75, refPosY + _pitchHeight - 1.3, painter)

        # Right vertical line
        self.drawLine(refPosX+_pitchWidth/2.0, refPosY, refPosX+_pitchWidth/2.0, refPosY+_pitchHeight, _lineWidth, self.defaultColor, painter)
        # Right horizontal line
        self.drawLine(refPosX+_pitchWidth/2.0, refPosY, refPosX+_pitchGap/2.0, refPosY, _lineWidth, self.defaultColor, painter)

    def paintPitchLineNeg(self, text, refPosX, refPosY, painter):
        _pitchWidth = 30.0
        _pitchGap = _pitchWidth / 2.5
        _pitchHeight = _pitchWidth / 12.0
        _textSize = _pitchHeight * 1.6
        _segmentWidth = ((_pitchWidth - _pitchGap)/2.0) / 7.0 #/< Four lines and three gaps -> 7 segments

        _lineWidth = 1.5

        # Negative pitch indicator:
        #
        #      -10
        #     _ _ _ _|     |_ _ _ _
        #
        #

        # Left vertical line
        self.drawLine(refPosX-_pitchGap/2.0, refPosY, refPosX-_pitchGap/2.0, refPosY-_pitchHeight, _lineWidth, self.defaultColor, painter)
        # Left horizontal line with four segments
        i = 0
        while i < 7:
            self.drawLine(refPosX-_pitchWidth/2.0+(i*_segmentWidth), refPosY, refPosX-_pitchWidth/2.0+(i*_segmentWidth)+_segmentWidth, refPosY, _lineWidth, self.defaultColor, painter)
            i+=2
        # Text left
        self.paintText(text, self.defaultColor, _textSize, refPosX-_pitchWidth/2.0 + 0.75, refPosY + _pitchHeight - 1.3, painter)

        # Right vertical line
        self.drawLine(refPosX+_pitchGap/2.0, refPosY, refPosX+_pitchGap/2.0, refPosY-_pitchHeight, _lineWidth, self.defaultColor, painter)
        # Right horizontal line with four segments
        i = 0
        while i < 7:
            self.drawLine(refPosX+_pitchWidth/2.0-(i*_segmentWidth), refPosY, refPosX+_pitchWidth/2.0-(i*_segmentWidth)-_segmentWidth, refPosY, _lineWidth, self.defaultColor, painter)
            i += 2

    def rotatePointClockWise(self, p: QPointF, angle):
        '''
        Standard 2x2 rotation matrix, counter-clockwise
          |  cos(phi)   sin(phi) |
          | -sin(phi)   cos(phi) |
        '''
        p.setX(cos(angle) * p.x() + sin(angle)* p.y())
        p.setY((-1.0 * sin(angle) * p.x()) + cos(angle) * p.y())

    def refLineWidthToPen(self, line):
        return line * 2.50

    def rotatePolygonClockWiseRad(self, p: QPolygonF, angle, origin):
        '''
        Rotate a polygon around a point
        :param p: polygon to rotate
        :param origin: the rotation center
        :param angle: rotation angle, in radians
        :return p: Polygon p rotated by angle around the origin point
        '''
        for i in range(p.size()):
            curr = p.at(i)
            x = curr.x()
            y = curr.y()
            curr.setX(((cos(angle) * (x-origin.x())) + (-sin(angle) * (y-origin.y()))) + origin.x())
            curr.setY(((sin(angle) * (x-origin.x())) + (cos(angle) * (y-origin.y()))) + origin.y())
            p.replace(i, curr)

    def drawPolygon(self, refPolygon, painter):
        # Scale coordinates
        draw = QPolygonF(refPolygon.size())
        for i in range(refPolygon.size()):
            curr = QPointF()
            curr.setX(self.refToScreenX(refPolygon.at(i).x()))
            curr.setY(self.refToScreenY(refPolygon.at(i).y()))
            draw.replace(i, curr)
        painter.drawPolygon(draw)

    def drawChangeRateStrip(self, xRef, yRef, height, minRate, maxRate, value, painter, reverse = False):
        _scaledValue = value

        # Saturate value
        if value > maxRate:
            _scaledValue = maxRate
        if value < minRate:
            _scaledValue = minRate

        #           x (Origin: xRef, yRef)
        #           -
        #           |
        #           |
        #           |
        #           =
        #           |
        #   -0.005 >|
        #           |
        #           -

        _width = height / 8.0
        _lineWidth = 1.5

        # Indicator lines
        # Top horizontal line
        if reverse:
            self.drawLine(xRef, yRef, xRef-_width, yRef, _lineWidth, self.defaultColor, painter)
            # Vertical main line
            self.drawLine(xRef-_width/2.0, yRef, xRef-_width/2.0, yRef+height, _lineWidth, self.defaultColor, painter)
            # Zero mark
            self.drawLine(xRef, yRef+height/2.0, xRef-_width, yRef+height/2.0, _lineWidth, self.defaultColor, painter)
            # Horizontal bottom line
            self.drawLine(xRef, yRef+height, xRef-_width, yRef+height, _lineWidth, self.defaultColor, painter)

            # Text
            label = '{:06.2f} >'.format(value)

            font = QFont('Bitstream Vera Sans')
            # Enforce minimum font size of 5 pixels
            #int fSize = qMax(5, (int)(6.0f*scalingFactor*1.26f))
            font.setPixelSize(6.0 * 1.26)

            metrics = QFontMetrics(font)
            self.paintText(label, self.defaultColor, 6.0, (xRef-_width) - metrics.width(label), yRef+height-((_scaledValue - minRate)/(maxRate-minRate))*height - 1.6, painter)
        else:
            self.drawLine(xRef, yRef, xRef+_width, yRef, _lineWidth, self.defaultColor, painter)
            # Vertical main line
            self.drawLine(xRef+_width/2.0, yRef, xRef+_width/2.0, yRef+height, _lineWidth, self.defaultColor, painter)
            # Zero mark
            self.drawLine(xRef, yRef+height/2.0, xRef+_width, yRef+height/2.0, _lineWidth, self.defaultColor, painter)
            # Horizontal bottom line
            self.drawLine(xRef, yRef+height, xRef+_width, yRef+height, _lineWidth, self.defaultColor, painter)

            # Text
            label = '<{:06.2f}'.format(value)
            self.paintText(label, self.defaultColor, 6.0, xRef+_width/2.0, yRef+height-((_scaledValue - minRate)/(maxRate-minRate))*height - 1.6, painter)

    def drawSystemIndicator(self, xRef, yRef, maxNum, maxWidth, maxHeight, painter):
        pass

    def drawChangeIndicatorGauge(self, xRef, yRef, radius, expectedMaxChange, value, color, painter, solid):
        # Draw the circle
        circlePen = QPen(Qt.SolidLine)
        if solid == False:
            circlePen.setStyle(Qt.DotLine)
        circlePen.setColor(self.defaultColor)
        circlePen.setWidth(self.refLineWidthToPen(2.0))
        painter.setBrush(Qt.NoBrush)
        painter.setPen(circlePen)
        self.drawCircle(xRef, yRef, radius, 200.0, 170.0, 1.5, color, painter)

        label = '{:05.1f}'.format(value)

        _textSize = radius / 2.5

        # Draw the value
        self.paintText(label, color, _textSize, xRef-_textSize*1.7, yRef-_textSize*0.4, painter)

        # Draw the needle
        # Scale the rotation so that the gauge does one revolution
        # per max. change
        _rangeScale = (2.0 * M_PI) / expectedMaxChange
        _maxWidth = radius / 10.0
        _minWidth = _maxWidth * 0.3

        p = QPolygonF(6)
        p.replace(0, QPointF(xRef-_maxWidth/2.0, yRef-radius * 0.5))
        p.replace(1, QPointF(xRef-_minWidth/2.0, yRef-radius * 0.9))
        p.replace(2, QPointF(xRef+_minWidth/2.0, yRef-radius * 0.9))
        p.replace(3, QPointF(xRef+_maxWidth/2.0, yRef-radius * 0.5))
        p.replace(4, QPointF(xRef,              yRef-radius * 0.46))
        p.replace(5, QPointF(xRef-_maxWidth/2.0, yRef-radius * 0.5))

        self.rotatePolygonClockWiseRad(p, value*_rangeScale, QPointF(xRef, yRef))

        indexBrush = QBrush()
        indexBrush.setColor(self.defaultColor)
        indexBrush.setStyle(Qt.SolidPattern)
        painter.setPen(Qt.SolidLine)
        painter.setPen(self.defaultColor)
        painter.setBrush(indexBrush)
        self.drawPolygon(p, painter)

    def drawLine(self, refX1, refY1, refX2, refY2, width, color, painter):
        pen = QPen(Qt.SolidLine)
        pen.setWidth(self.refLineWidthToPen(width))
        pen.setColor(color)
        painter.setPen(pen)
        painter.drawLine(QPoint(self.refToScreenX(refX1), self.refToScreenY(refY1)),
                         QPoint(self.refToScreenX(refX2), self.refToScreenY(refY2)))

    def drawEllipse(self, refX, refY, radiusX, radiusY, startDeg, endDeg, lineWidth, color, painter):
        unused(startDeg, endDeg)
        pen = QPen(painter.pen().style())
        pen.setWidth(self.refLineWidthToPen(lineWidth))
        pen.setColor(color)
        painter.setPen(pen)
        painter.drawEllipse(QPointF(self.refToScreenX(refX), self.refToScreenY(refY)), self.refToScreenX(radiusX), self.refToScreenY(radiusY))

    def drawCircle(self, refX, refY, radius, startDeg, endDeg, lineWidth, color, painter):
        self.drawEllipse(refX, refY, radius, radius, startDeg, endDeg, lineWidth, color, painter)

    def selectWaypoint(self, uasId, wpid):
        unused(uasId)
        self.waypointName = 'WP{}'.format(wpid)

    def setImageExternal(self, img):
        if self.videoEnabled:
            self.glImage = img
        else:
            self.glImage = self.DEFAULT_BACKGROUND_IMAGE

    def enableHUDInstruments(self, enabled):
        self.HUDInstrumentsEnabled = enabled

    def enableVideo(self, enabled):
        self.videoEnabled = enabled

    def selectOfflineDirectory(self):
        fileName = QFileDialog.getExistingDirectory(self, 'Select image directory',
                                                    QStandardPaths.writableLocation(QStandardPaths.DesktopLocation))
        if fileName != '':
            self.videoRecording = fileName
