#!/usr/bin/env python

from ui.ui_resources.face_ui_test import *
from PyQt5.QtWidgets import QApplication, QDialog, QInputDialog, QLineEdit
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtCore import QThread, pyqtSignal, pyqtSlot
import cv2
import sys
import os
import numpy as np


class FaceUI(QDialog):
    def __init__(self):
        super(FaceUI, self).__init__()
        self.init_ui()

    def init_ui(self):
        # setup ui
        self.ui = Ui_Dialog()
        self.ui.setupUi(self)
        self.ui.pushButtonRegister.clicked.connect(self.face_registration)
        video_thread = VideoThread(self)
        # video_thread.img_signal.connect(self.display)
        # video_thread.img_signal.connect(self.cvshow)
        video_thread.img_signal.connect(self.update_img)
        video_thread.start()

    @pyqtSlot(QImage)
    def update_img(self, image):
        self.qimage = image
        self.frame_height = image.height()
        self.frame_width = image.width()
        self.live_camera(image)

    # @pyqtSlot(QImage)
    # def display(self, image):
    #     img = QPixmap.fromImage(image)
    #     self.ui.labelCamera.setPixmap(img)

    def live_camera(self, image):
        img = QPixmap.fromImage(image)
        self.ui.labelCamera.resize(self.frame_width, self.frame_height)
        self.ui.labelCamera.setPixmap(img)

    def face_registration(self):
        (face_name, ok) = QInputDialog.getText(self, 'Register Name', 'Type in name', QLineEdit.Normal)


    @staticmethod
    def qimage_to_cv(qimage):
        b = qimage.bits()
        b.setsize(qimage.byteCount())
        height = qimage.height()
        width = qimage.width()
        img_arr = np.array(b).reshape(height, width, 3)
        return cv2.cvtColor(img_arr, cv2.COLOR_RGB2BGR)


class VideoThread(QThread):
    img_signal = pyqtSignal(QImage)

    def run(self):
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 480)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 320)
        while True:
            ret, frame = cap.read()
            if ret:
                rgbImage = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w, ch = rgbImage.shape
                bytesPerLine = w * ch
                img = QImage(rgbImage.data, w, h, bytesPerLine, QImage.Format_RGB888)
                self.img_signal.emit(img)


class FaceDetector:
    def __init__(self):
        cascade_file = os.path.join('/opt/ros/kinetic/share/OpenCV-3.3.1-dev/haarcascades',
                                    'haarcascade_frontalface_default.xml')
        self.faceCascade = cv2.CascadeClassifier(cascade_file)

    def face_detection(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        faces = self.faceCascade.detectMultiScale(gray)
        return faces

    @staticmethod
    def draw_faces(frame, faces):
        for face in faces:
            x, y, w, h = face
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        return frame


if __name__ == '__main__':
    app = QApplication(sys.argv)
    f = FaceUI()
    f.show()
    sys.exit(app.exec_())
