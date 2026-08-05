'''

下拉列表控件（QComboBox）

1. 如果将列表项添加到QComboBox控件中

2. 如何获取选中的列表项

'''

import sys
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5 import QtCore, QtGui, QtWidgets
import serial.tools.list_ports
port_list = list(serial.tools.list_ports.comports())
port_list_name = []

COM = 'COM10'

class QComboBoxDemo(QWidget):
    def __init__(self):
        super(QComboBoxDemo,self).__init__()
        self.initUI()

    def initUI(self):
        self.setWindowTitle('Radar Sensing Display 串口选择')
        # self.setWindowIcon(QIcon('icon.ico'))
        self.setStyleSheet("background-color: black")
        self.resize(500, 200)



        self.pushButtonOK = QtWidgets.QPushButton(self)
        self.pushButtonOK.setGeometry(QtCore.QRect(150, 20, 150, 40))
        self.pushButtonOK.setObjectName("pushButtonOK")
        self.pushButtonOK.clicked.connect(self.show_child)

        self.pushButtonOK.setText('OK')
        self.pushButtonOK.setStyleSheet("background-color: rgb(0,139,139); font-weight: bold;"
                                        "color: white; font: 150 20pt \"微软雅黑\";")
        self.pushButtonOK.raise_()

        layout = QVBoxLayout()

        self.label = QLabel('请选择对应串口')
        self.label.setStyleSheet("color: white; font: 150 20pt \"微软雅黑\";")

        self.cb = QComboBox()
        self.cb.setStyleSheet("color: white; background-color: rgb(47,79,79); font: 150 20pt \"微软雅黑\";")
        for each_port in port_list:
            self.cb.addItem(each_port[0])

        self.cb.currentIndexChanged.connect(self.selectionChange)

        layout.addWidget(self.label)
        layout.addWidget(self.cb)
        layout.addWidget(self.pushButtonOK)
        
        self.setLayout(layout)

        self.com_output()
        print('Serial Init')

    def show_child(self):
        pass

    def selectionChange(self):
        # global COM
        # self.label.setText(self.cb.currentText())
        # self.label.adjustSize()
        # #
        # # for count in range(self.cb.count()):
        # #     print('item' + str(count) + '=' + self.cb.itemText(count))
        # #     print('current index', i, 'selection changed', self.cb.currentText())
        # COM = self.cb.currentText()
        # print('COM ==', COM)
        # self.COMNum= self.cb.currentText()
        # return COMM
        pass

        # COM = self.COMNum

    def com_output(self):
        pass


# if __name__ == '__main__':
#     app = QApplication(sys.argv)
#     main = QComboBoxDemo()
#     main.show()
#     sys.exit(app.exec_())
