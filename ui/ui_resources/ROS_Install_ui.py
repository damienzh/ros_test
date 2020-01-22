# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'ROS_Install_ui.ui'
#
# Created by: PyQt5 UI code generator 5.11.3
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_Dialog(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName("Dialog")
        Dialog.resize(897, 309)
        self.label = QtWidgets.QLabel(Dialog)
        self.label.setGeometry(QtCore.QRect(110, 10, 181, 20))
        font = QtGui.QFont()
        font.setPointSize(16)
        self.label.setFont(font)
        self.label.setObjectName("label")
        self.pushButtonInstall = QtWidgets.QPushButton(Dialog)
        self.pushButtonInstall.setGeometry(QtCore.QRect(150, 240, 99, 27))
        self.pushButtonInstall.setAutoDefault(True)
        self.pushButtonInstall.setObjectName("pushButtonInstall")
        self.groupBox = QtWidgets.QGroupBox(Dialog)
        self.groupBox.setGeometry(QtCore.QRect(10, 40, 381, 91))
        self.groupBox.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.groupBox.setObjectName("groupBox")
        self.radioButtonBasicInstall = QtWidgets.QRadioButton(self.groupBox)
        self.radioButtonBasicInstall.setGeometry(QtCore.QRect(10, 30, 117, 22))
        self.radioButtonBasicInstall.setObjectName("radioButtonBasicInstall")
        self.radioButtonFullInstall = QtWidgets.QRadioButton(self.groupBox)
        self.radioButtonFullInstall.setGeometry(QtCore.QRect(10, 60, 117, 22))
        self.radioButtonFullInstall.setChecked(False)
        self.radioButtonFullInstall.setAutoExclusive(True)
        self.radioButtonFullInstall.setObjectName("radioButtonFullInstall")
        self.labelRespondInstallOption = QtWidgets.QLabel(self.groupBox)
        self.labelRespondInstallOption.setGeometry(QtCore.QRect(170, 40, 171, 17))
        self.labelRespondInstallOption.setAlignment(QtCore.Qt.AlignCenter)
        self.labelRespondInstallOption.setObjectName("labelRespondInstallOption")
        self.dockWidget = QtWidgets.QDockWidget(Dialog)
        self.dockWidget.setGeometry(QtCore.QRect(510, 10, 371, 291))
        self.dockWidget.setObjectName("dockWidget")
        self.dockWidgetContents = QtWidgets.QWidget()
        self.dockWidgetContents.setObjectName("dockWidgetContents")
        self.listWidget = QtWidgets.QListWidget(self.dockWidgetContents)
        self.listWidget.setGeometry(QtCore.QRect(10, 10, 351, 251))
        self.listWidget.setObjectName("listWidget")
        self.dockWidget.setWidget(self.dockWidgetContents)
        self.groupBox.raise_()
        self.label.raise_()
        self.pushButtonInstall.raise_()
        self.dockWidget.raise_()

        self.retranslateUi(Dialog)
        QtCore.QMetaObject.connectSlotsByName(Dialog)

    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "ROS Install"))
        self.label.setText(_translate("Dialog", "Install ROS Kinetic"))
        self.pushButtonInstall.setText(_translate("Dialog", "Install"))
        self.groupBox.setTitle(_translate("Dialog", "Install Option"))
        self.radioButtonBasicInstall.setText(_translate("Dialog", "Basic Install"))
        self.radioButtonFullInstall.setText(_translate("Dialog", "Desktop Full"))
        self.labelRespondInstallOption.setText(_translate("Dialog", "Display Install Option"))
        self.dockWidget.setWindowTitle(_translate("Dialog", "Log"))

