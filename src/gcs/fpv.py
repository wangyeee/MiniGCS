from PyQt5.QtCore import pyqtSignal, QThread
from PyQt5.QtGui import QImage
from cv2 import VideoCapture, cvtColor, COLOR_BGR2RGB, CAP_PROP_FPS
from time import sleep, time

class VideoSource(QThread):

    newFrameAvailable = pyqtSignal(object)

    def __init__(self, parent = None):
        super().__init__(parent)
        self.running = True
        self.pause = False

    def pauseVideo(self, pause):
        self.pause = pause

class FileVideoSource(VideoSource):
    '''
    Load a video file as video source, for test or flight replay purposes.
    '''

    def __init__(self, fileName, parent = None):
        super().__init__(parent)
        self.cap = VideoCapture(fileName)
        self.frameRate = self.cap.get(CAP_PROP_FPS)
        self.__delay = 1.0 / self.frameRate

    def run(self):
        self.running = True
        while self.cap.isOpened():
            if self.pause:
                continue
            _s0 = time()
            ret, frame = self.cap.read()
            if ret == True:
                rgbImage = cvtColor(frame, COLOR_BGR2RGB)
                h, w, ch = rgbImage.shape
                bytesPerLine = ch * w
                convertToQtFormat = QImage(rgbImage.data, w, h, bytesPerLine, QImage.Format_RGB888)
                self.newFrameAvailable.emit(convertToQtFormat)
            _s0 = time() - _s0
            sleep(0 if _s0 >= self.__delay else self.__delay - _s0)
        self.running = False
        self.__cleanup()

    def __cleanup(self):
        self.cap.release()
