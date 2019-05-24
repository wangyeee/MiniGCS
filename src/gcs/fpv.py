from PyQt5.QtCore import Qt, pyqtSignal, QThread
from PyQt5.QtGui import QImage
from cv2 import VideoCapture, cvtColor, COLOR_BGR2RGB
from time import sleep

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

    def run(self):
        self.running = True
        while self.cap.isOpened():
            if self.pause:
                continue
            ret, frame = self.cap.read()
            if ret == True:
                rgbImage = cvtColor(frame, COLOR_BGR2RGB)
                h, w, ch = rgbImage.shape
                bytesPerLine = ch * w
                convertToQtFormat = QImage(rgbImage.data, w, h, bytesPerLine, QImage.Format_RGB888)
                # TODO scale video size based on HUD size
                p = convertToQtFormat.scaled(640, 480, Qt.KeepAspectRatio)
                self.newFrameAvailable.emit(p)
            sleep(0.04) # TODO check video frame rate
        self.running = False
        self.__cleanup()

    def __cleanup(self):
        self.cap.release()
