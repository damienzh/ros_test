#! /usr/bin/python

import sys
import platform
from subprocess import Popen, PIPE
from ui.ui_resources.ROS_Install_ui import *
from PyQt5.QtWidgets import QDialog, QApplication, QInputDialog, QLineEdit, QListWidget


class ROSInstallPanel(QDialog):

    def __init__(self):
        super(ROSInstallPanel, self).__init__()
        self.install_option = ''
        self.ui = Ui_Dialog()
        self.init_ui()
        self.ros_disto()

    def init_ui(self):
        self.ui.setupUi(self)
        self.ui.radioButtonBasicInstall.toggled.connect(self.displayOption)
        self.ui.radioButtonFullInstall.toggled.connect(self.displayOption)
        self.ui.pushButtonInstall.clicked.connect(self.installROS)

    def installROS(self):
        if self.install_option != '':
            install_cmd = ['sudo', '-S', 'apt', 'install', 'ros-' + self.disto + '-' + self.install_option]
            install_proc = Popen(install_cmd, stdin=PIPE, stdout=PIPE, stderr=PIPE,
                                 universal_newlines=True)
            pwd = QInputDialog.getText(self, 'SUDO Password', 'Type Password', QLineEdit.Password)
            install_proc.stdin.write(pwd+'\n')
            while True:
                self.ui.pushButtonInstall.setEnabled(False)
                output = install_proc.stdout.readline()
                if output == '' and install_proc.poll() is not None:
                    self.ui.pushButtonInstall.setEnabled(True)
                    break
                if output:
                    print output.strip()
                    self.ui.listWidget.addItem(output.strip())

    def displayOption(self):
        if self.ui.radioButtonFullInstall.isChecked():
            self.install_option = 'desktop-full'
            self.ui.radioButtonBasicInstall.setChecked(False)
            self.ui.labelRespondInstallOption.setText('Install desktop full')
        if self.ui.radioButtonBasicInstall.isChecked():
            self.install_option = 'ros-base'
            self.ui.radioButtonFullInstall.setChecked(False)
            self.ui.labelRespondInstallOption.setText('Install ROS core')

    def run_script(self):
        pass

    def ros_disto(self):
        self.sysdisto = platform.dist()[1]
        if self.sysdisto == '16.04':
            self.disto = 'kinetic'
        if self.sysdisto == '18.04':
            self.disto = 'melodic'


if __name__ == '__main__':
    app = QApplication(sys.argv)
    i = ROSInstallPanel()
    i.show()
    sys.exit(app.exec_())
