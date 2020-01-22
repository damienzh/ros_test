#! /usr/bin/python

from PyQt5.QtWidgets import QApplication, QDialog, QInputDialog, QLineEdit, QListWidgetItem
from PyQt5.QtCore import QProcess
from ui.ui_resources.test import *
import sys
import subprocess


class TestUI(QDialog):

    def __init__(self):
        super(TestUI, self).__init__()
        self.option = ''
        self.init_ui()
        self.msg = 0

    def init_ui(self):
        self.ui = Ui_Dialog()
        self.ui.setupUi(self)
        self.ui.radioButtonTest1.toggled.connect(self.display_option)
        self.ui.radioButtonTest2.toggled.connect(self.display_option)
        self.ui.pushButton.clicked.connect(self.dispmsg)
        self.ui.pushButtonKeep.clicked.connect(self.display_keep)
        self.init_toolbox1()

    def init_toolbox1(self):
        self.ui.pushButtonStrInput.clicked.connect(self.prompt_str)
        self.ui.pushButtonIntInput.clicked.connect(self.prompt_int)
        self.ui.pushButtonAptUpdate.clicked.connect(self.apt_update)

    def dispmsg(self):
        name = self.ui.lineEditName.text()
        if not self.option == '':
            self.ui.labelRespond.setText('Hello ' + name)

    def display_keep(self):
        self.msg += 1
        print self.msg
        self.ui.labelRespondKeep.setText(str(self.msg))

    def display_option(self):
        if self.ui.radioButtonTest1.isChecked():
            self.ui.radioButtonTest2.setChecked(False)
            self.option = 'Button1 checked'
        if self.ui.radioButtonTest2.isChecked():
            self.ui.radioButtonTest1.setChecked(False)
            self.option = 'Button2 checked'
        self.ui.labelTest.setText(self.option)

    def prompt_str(self):
        (text, ok) = QInputDialog.getText(self, 'Text Input', 'Type string', QLineEdit.Password)
        if ok:
            self.ui.lineEditRespondStr.setText(text)

    def prompt_int(self):
        (num, ok) = QInputDialog.getInt(self, 'Int Input', 'Type Int', QLineEdit.Normal)
        if ok:
            self.ui.lineEditRespondInt.setText(str(num))

    def apt_update(self):
        (pwd, ok) = QInputDialog.getText(self, 'Password Input', 'Password', QLineEdit.Password)
        if ok:
            print 'apt process'
            proc = subprocess.Popen(['sudo', '-S', 'apt', 'update'],
                                    stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                    universal_newlines=True)
            proc.stdin.write(pwd+'\n')
            while True:
                self.ui.pushButtonAptUpdate.setEnabled(False)
                output = proc.stdout.readline()
                if output == '' and proc.poll() is not None:
                    self.ui.pushButtonAptUpdate.setEnabled(True)
                    break
                if output:
                    print output.strip()
                    self.ui.plainTextEditApt.appendPlainText(output.strip())


if __name__ == '__main__':
    app = QApplication(sys.argv)
    f = TestUI()
    f.show()
    sys.exit(app.exec_())
