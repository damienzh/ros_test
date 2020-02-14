# -*- coding: utf-8 -*-

from PyQt5.QtWidgets import QDialog, QApplication, QInputDialog
from PyQt5.QtGui import QPixmap
from ui.ui_resources.vending_demo_ui import *
import sys


class VendingUI(QDialog):
    COKE_PIC = '../ui_resources/img/coke.jpg'
    SPRITE_PIC = '../ui_resources/img/sprite.jpg'
    FANTA_PIC = '../ui_resources/img/fanta.jpg'
    DRINK_IN = '../ui_resources/img/indicator_green.jpg'
    DRINK_OUT = '../ui_resources/img/indicator_red.jpg'

    def __init__(self):
        super(VendingUI, self).__init__()
        # initialize parameters
        self.drink_status = '000000'
        self.drink_temp = 20.0
        # initialize ui
        self.init_ui()

    def init_ui(self):
        self.ui = Ui_Dialog()
        self.ui.setupUi(self)

        # setup drink pictures
        self.ui.labelCoke1.setPixmap(QPixmap(self.COKE_PIC))
        self.ui.labelCoke2.setPixmap(QPixmap(self.COKE_PIC))
        self.ui.labelFanta1.setPixmap(QPixmap(self.FANTA_PIC))
        self.ui.labelFanta2.setPixmap(QPixmap(self.FANTA_PIC))
        self.ui.labelSprite1.setPixmap(QPixmap(self.SPRITE_PIC))
        self.ui.labelSprite2.setPixmap(QPixmap(self.SPRITE_PIC))

        # link actions
        self.ui.pushButtonRefreshStatus.clicked.connect(self.update_drink_status)
        self.update_drink_status()

    def update_drink_status(self):
        def update_label_pixmap(label, status):
            if status == '0':
                label.setPixmap(QPixmap(self.DRINK_OUT))
            else:
                label.setPixmap(QPixmap(self.DRINK_IN))
        update_label_pixmap(self.ui.labelCoke1_status, self.drink_status[0])
        update_label_pixmap(self.ui.labelCoke2_status, self.drink_status[1])
        update_label_pixmap(self.ui.labelSprite1_status, self.drink_status[2])
        update_label_pixmap(self.ui.labelSprite2_status, self.drink_status[3])
        update_label_pixmap(self.ui.labelFanta1_status, self.drink_status[4])
        update_label_pixmap(self.ui.labelFanta2_status, self.drink_status[5])

        self.ui.lcdNumberTemp.display(str(self.drink_temp)+"'C")


if __name__ == '__main__':
    app = QApplication(sys.argv)
    vending_ui = VendingUI()
    vending_ui.show()
    sys.exit(app.exec_())
