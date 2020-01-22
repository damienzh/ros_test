# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'face_ui_test.ui'
#
# Created by: PyQt5 UI code generator 5.11.3
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_Dialog(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName("Dialog")
        Dialog.resize(813, 359)
        self.listView = QtWidgets.QListView(Dialog)
        self.listView.setGeometry(QtCore.QRect(530, 150, 231, 192))
        self.listView.setObjectName("listView")
        self.pushButtonRegister = QtWidgets.QPushButton(Dialog)
        self.pushButtonRegister.setGeometry(QtCore.QRect(560, 20, 99, 27))
        self.pushButtonRegister.setObjectName("pushButtonRegister")
        self.labelCamera = QtWidgets.QLabel(Dialog)
        self.labelCamera.setGeometry(QtCore.QRect(20, 20, 480, 320))
        self.labelCamera.setFrameShape(QtWidgets.QFrame.Box)
        self.labelCamera.setObjectName("labelCamera")

        self.retranslateUi(Dialog)
        QtCore.QMetaObject.connectSlotsByName(Dialog)

    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "Dialog"))
        self.pushButtonRegister.setText(_translate("Dialog", "Register"))
        self.labelCamera.setText(_translate("Dialog", "TextLabel"))

