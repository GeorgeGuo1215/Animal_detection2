# -*- coding: utf-8 -*-
import numpy

# from PyQt5.QtWidgets import QMainWindow, QApplication, QGraphicsScene, QGraphicsPixmapItem
from PyQt5.QtGui import QImage, QPixmap, QIcon
import time
import numpy as np
import pyqtgraph as pg
from pyqtgraph import PlotWidget, PlotCurveItem
from PyQt5 import QtCore, QtGui, QtWidgets

from MainV2 import Ui_MainP


class Main_Page_V2(QtWidgets.QMainWindow, Ui_MainP):
    def __init__(self, parent=None):
        super(Main_Page_V2, self).__init__(parent)
        self.setupUi(self)

        # 标题栏设置
        # self.setWindowFlag(QtCore.Qt.FramelessWindowHint)
        self.setWindowTitle('Radar Sensing Display DSQ_Version_1')
        # self.setWindowIcon(QIcon('img/icon.ico'))

        # 计时器绑定
        self.timer_figure_refresh = pg.QtCore.QTimer()
        self.timer_figure_refresh.timeout.connect(self._fetch_figure)
        self.timer_value_refresh = pg.QtCore.QTimer()
        self.timer_value_refresh.timeout.connect(self._fetch_value)

        self.timer_figure_refresh.start(50)
        self.timer_value_refresh.start(2000)

        # 画图控件
        #    __建立控件
        self.plot1 = PlotWidget(self)
        self.plot1.setGeometry(QtCore.QRect(140, 200, 1000, 200))
        self.plot3 = PlotWidget(self)
        self.plot3.setGeometry(QtCore.QRect(140, 420, 1000, 200))
        self.plot2 = PlotWidget(self)
        self.plot2.setGeometry(QtCore.QRect(200, 660, 900, 150))
        self.plot7 = PlotWidget(self)
        self.plot7.setGeometry(QtCore.QRect(200, 830, 900, 150))
        # self.plot8 = PlotWidget(self)
        # self.plot8.setGeometry(QtCore.QRect(120, 730, 1000, 150))

        #    __画初始图
        data0 = 0 * np.ones(500).astype(int)
        self.curve1 = self.plot1.plot(data0, pen=pg.mkPen(width=3.5, color='w'))
        self.curve2 = self.plot2.plot(data0, pen=pg.mkPen(width=3.5, color='g'))
        # self.curve22 = self.plot2.plot(data0, pen=pg.mkPen(width=2, color='w'))
        self.curve3 = self.plot3.plot(data0, pen=pg.mkPen(width=3.5, color='y'))
        self.curve7 = self.plot7.plot(data0, pen=pg.mkPen(width=3.5, color='m'))
        # self.curve81 = self.plot8.plot(data0, pen=pg.mkPen(width=2, color='w'))
        # self.curve82 = self.plot8.plot(data0, pen=pg.mkPen(width=2, color='r'))
        # self.curve4 = self.plot2.plot(data0, pen=pg.mkPen(width=3.5, color='w'))

        #    __设置背景
        #    __图1
        self.plot1.setBackground('k')
        # self.plot1.getPlotItem().hideAxis('bottom')
        # self.plot1.getPlotItem().hideAxis('left')

        #    __图2 心率数值记录图
        self.plot2.setBackground('k')
        self.plot3.setBackground('k')
        self.plot7.setBackground('k')
        # self.plot8.setBackground('k')
        # self.plot3.getPlotItem().hideAxis('bottom')
        # self.plot3.getPlotItem().hideAxis('left')

        self.plot2.getPlotItem().showAxis('right')
        self.plot2.setYRange(0, 35, padding=0)
        self.plot7.getPlotItem().showAxis('right')
        self.plot7.setYRange(50, 130, padding=0)
        # left_axis = self.plot7.getPlotItem().getAxis("left")
        # labelStyle = {'color': '#000', 'font-size': '22pt'}
        # left_axis.setLabel('y', uints='UINT', *labelStyle)

        #    __IQ图
        # self.plot3 = PlotWidget(self)
        # self.plot3.setGeometry(QtCore.QRect(800, 630, 350, 300))
        # self.plot3.setBackground('k')
        # self.curve3 = self.plot3.plot(data0, pen=pg.mkPen(width=3, color='r'))
        # self.plot3.setYRange(-1, 1, padding=0)
        # self.plot3.setXRange(-1, 1, padding=0)
        # self.plot3.getPlotItem().showAxis('top')
        # self.plot3.getPlotItem().showAxis('right')

        # #   __频谱图
        # self.plot4 = PlotWidget(self)
        # self.plot4.setGeometry(QtCore.QRect(1200, 180, 500, 200))
        # self.plot4.setBackground('k')
        # self.curve4 = self.plot4.plot(data0, pen=pg.mkPen(width=3, color='g'))
        # self.plot44 = PlotWidget(self)
        # self.plot44.setGeometry(QtCore.QRect(1200, 380, 500, 200))
        # self.plot44.setBackground('k')
        # self.curve44 = self.plot44.plot(data0, pen=pg.mkPen(width=3, color='g'))

        # #   __PPG最大值点图
        # self.plot5 = PlotWidget(self)
        # self.plot5.setGeometry(QtCore.QRect(125, 410, 980-5, 20))
        # self.plot5.setBackground('k')
        # self.curve5 = pg.ScatterPlotItem(size=10, pen=pg.mkPen(None), symbol='t', brush=pg.mkBrush(50, 255, 255, 240))
        # self.plot5.addItem(self.curve5)
        # self.curve5.setData([1], [1])
        # self.plot5.getPlotItem().hideAxis('bottom')
        # self.plot5.getPlotItem().hideAxis('left')
        # self.plot5.setXRange(1, 500, padding=0)
        # self.plot5.setYRange(0, 2, padding=0)

        # 设置时间
        self.label_time_day.setText(str(time.strftime('%Y/%m/%d', time.localtime(time.time()))))
        self.label_time_min.setText(str(time.strftime('%H:%M', time.localtime(time.time()))))

    def _fetch_figure(self):
        pass

    def _fetch_value(self):
        pass
