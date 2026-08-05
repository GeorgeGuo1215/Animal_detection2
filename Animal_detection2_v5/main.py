# -*- coding: utf-8 -*-
import time
from typing import no_type_check
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QRunnable, QThreadPool, pyqtSlot
import numpy as np
import threading
import serial
import sys
import pyqtgraph as pg
import collections
import time

import os
from PyQt5.QtWidgets import QMainWindow, QApplication, QGraphicsScene, QGraphicsPixmapItem, QFileDialog, QMessageBox
from scipy import signal, fftpack
from scipy.fftpack import fft
import copy

from UI_Init import Main_Page_V2
from Load_port import QComboBoxDemo
from Filter_parameter import HPF_par, LPF_par, HPF_short_1_par, HPF_short_2_par, HPF_short_3_par, HPF_short_4_par, \
    HPF_short_6_par, HPF_short_5_par
# import pywt

# pyinstaller -F main.py --paths="C:\Users\DELL\AppData\Local\Programs\Python\Python39\lib\site-packages\cv2" --icon=icon.ico
# CV path: C:\Users\DELL\AppData\Local\Programs\Python\Python39\lib\site-packages\cv2
#
#  参数设置
fs = 100  # 采样率
n = 1000  # 数据段长度
N = 2000  # 长数据
demo_1 = np.zeros(n)  # figure1 curve
demo_2 = np.zeros(n)  # figure2 curve
dsp_short = np.zeros(n)
dsp_long = np.zeros(N)
data_short = np.zeros(n * 5).reshape(5, n)  # short data
data_long = np.zeros(N * 5).reshape(5, N)  # long data

hb = np.zeros(60)
re = np.zeros(60)

now_heart = 0

num_hb = 0
num_re = 0

heart_history_short = np.ones(200) * 60
heart_history_long = np.ones(200) * 60

heart_delta = 5

write_counter = 0

# # 滤波器参数设置
# bh1, ah1 = signal.butter(4, 0.005, 'highpass')
# bp1, ap1 = signal.butter(4, [0.015, 0.05], 'bandpass')

# 串口初始化
COM = 'COM31'


class QPort(QComboBoxDemo):

    def com_output(self):
        global COM
        COM = self.cb.currentText()

    def selectionChange(self):
        global COM
        print('Text = ', self.cb.currentText())

        COM = self.cb.currentText()

        print('COM  ==', COM)
        print('OK!!')

    def show_child(self):
        self.child_window = Ui_MainP_Frontend()
        self.child_window.show()


class Ui_MainP_Frontend(Main_Page_V2):
    def __init__(self, parent=None) -> None:
        global COM
        print('MainUI start to work ------- ', COM)

        Main_Page_V2.__init__(self, parent)

        # 串口初始化，添加错误处理
        self.ser_c = None
        self.serial_connected = False
        try:
            self.ser_c = serial.Serial(COM, 115200, timeout=0.1)  # 添加超时
            self.serial_connected = True
            print(f"串口 {COM} 连接成功")
        except Exception as e:
            print(f"串口 {COM} 连接失败: {e}")
            print("程序将在无串口模式下运行，只支持数据导入功能")
            self.serial_connected = False

        # 计时器绑定
        self.timer_ser_refresh = pg.QtCore.QTimer()
        self.timer_ser_refresh.timeout.connect(self._fresh_ser)
        self.timer_ser_refresh.start(1)

        # 找圆心计时器绑定
        self.timer_cirfit_refresh = pg.QtCore.QTimer()
        self.timer_cirfit_refresh.timeout.connect(self._fresh_cirfit)
        self.timer_cirfit_refresh.start(10)

        # log的timer与ser read写在一起，保证采样率
        # self.timer_log_refresh = pg.QtCore.QTimer()
        # self.timer_log_refresh.timeout.connect(self._log)
        # self.timer_log_refresh.start(20)

        # 按钮绑定
        self.button_exit.clicked.connect(self.close)
        self.button_record.clicked.connect(self._set_button_att)
        self.button_import.clicked.connect(self._import_data)
        
        # 如果串口未连接，禁用记录按钮
        if not self.serial_connected:
            self.button_record.setEnabled(False)
            self.button_record.setText("无串口连接")
            self.button_record.setStyleSheet("QPushButton{\n"
                "font: 70 40pt \"微软雅黑\";\n"
                "background-color:rgb(100, 100, 100);\n"
                "color:white;\n"
                "}")
            # 更新状态显示
            self.label_state.setText("无串口模式")

        # self.horizontalSlider.valueChanged.connect(self._valuechange)

        # flag初始化
        self.flag_record = 0
        self.cnt_record = 0
        self.cnt_init = 0
        self.flag_hold = 0
        self.flag_state = 1
        self.flag_import_mode = 0  # 0: 实时模式, 1: 导入模式
        self.imported_data = None  # 存储导入的数据

        # 圆拟合参数初始化
        self.crf = [0, 0]
        self.crf_sqrt = 1

        # 参数初始化
        self.value_amp = 0
        self.value_amp_ppg = 0
        self.slider_value = 100

        self.PEOPLEAVAILABLE = 0
        self.people_set = 0

        print('Init finished')

    # 画图更新
    def _fetch_figure(self):
        global data_short, data_long, demo_1, demo_2, dsp_short, n, fs, num_hb, now_heart, num_re, heart_history_short, heart_history_long

        # 在导入模式下，不需要实时更新图形，数据已经在导入时处理过了
        if self.flag_import_mode == 1:
            return

        # 设置曲线
        data0 = (np.array(data_long[0, :]))

        # data_breath1 = np_move_avg((data_long[1, :] - np.mean(data_long[1, :])), 100, mode='valid')[
        #               2000:4000]
        # data_breath2 = np_move_avg((data_long[2, :] - np.mean(data_long[1, :])), 100, mode='valid')[
        #                2000:4000]
        # data_breath = np_move_avg(data_breath, 80, mode='valid')
        # data_breath = data_breath - np.min(data_breath)

        demo_1 = np_move_avg(data0, 10, mode='valid')[N-1000:N]
        # demo_1 = np_move_avg(data0, 100, mode='valid')[3000:4000]
        demo_1 = demo_1 - np.min(demo_1)

        demo_2 = np.convolve(data0, HPF_short_4_par, mode='same')
        demo_2 = np.convolve(demo_2, HPF_short_5_par, mode='same')

        # demo_2 = np.convolve(data0, HPF_short_1_par, mode='same')

        # 设置幅度数字
        # self.uua = 0
        # aaa = data_long[1, 3500:4000]
        # aaa = aaa - np.min(aaa)
        # bbb = data_long[2, 3500:4000]
        # bbb = bbb - np.min(bbb)
        # # aaa = data_long[1, 3800:4000]
        # # bbb = data_long[2, 3800:4000]
        # for i in range(500):
        #     self.uua = self.uua + aaa[i] + bbb[i]
        # self.value_amp = np.around(self.uua, 4)
        # # self.uua = np.around(np.max([aaa, bbb]), 4)
        # # self.value_amp = np.around(self.uua, 4)
        # # self.label_amp_disp.setText(str(self.value_amp))
        # uu22 = np.max(demo_1)
        # # self.label_state.setText(str(np.around(uu22, 3)))

        if self.cnt_init == 72400:

            b3, a3 = signal.butter(4, 0.006, 'highpass')
            data_raw = np.append(data_long[0, :], np.zeros(2000))
            # data_raw_full = np.convolve(data_raw, HPF_par, mode='same')
            # data_raw_full = signal.filtfilt(b3, a3, data_raw)
            # data_raw_full = np.append(signal.filtfilt(b3, a3, data_long[0, :]), np.zeros(2000))

            move_step = 5

            data_disp_short = np.append(np_move_avg(np.diff(data_short[0, :]), move_step, mode='valid'), np.zeros(move_step+1))
            data_disp_short_fft = fft(data_disp_short)
            disp_short_norm = np.abs(data_disp_short_fft)
            disp_short_norm_half = disp_short_norm[range(25)]
            axis_freq_d_short = np.linspace(0, 150, 25)

            bre_slice_short = disp_short_norm_half[2:4]
            heart_slice_short = disp_short_norm_half[9:20]
            num_re_short = int((np.argmax(bre_slice_short) + 2) * 6)
            num_hb_short = int((np.argmax(heart_slice_short) + 9) * 6)

            # self.curve4.setData(axis_freq_d_short, disp_short_norm_half)  # 频谱_10s

            data_disp_long = np.append(np_move_avg(np.diff(data_long[0, :]), move_step, mode='valid'), np.zeros(move_step+1))
            data_disp_long_fft = fft(data_disp_long)
            disp_norm = np.abs(data_disp_long_fft)
            disp_norm_half = disp_norm[range(50)]
            axis_freq_d = np.linspace(0, 150, 50)

            bre_slice_long = disp_short_norm_half[4:8]
            heart_slice_long = disp_short_norm_half[18:40]
            num_re = int((np.argmax(bre_slice_long) + 4) * 3)
            num_hb_long = int((np.argmax(heart_slice_long) + 18) * 3)

            # self.curve44.setData(axis_freq_d, disp_norm_half)  # 频谱_20s

            heart_history_long[:-1] = heart_history_long[1:]
            heart_history_long[199] = num_hb_long

            heart_history_short[:-1] = heart_history_short[1:]
            heart_history_short[199] = num_hb_short

            breath1 = np_move_avg((data_short[0, :] - np.min(data_short[0, :])), 10, mode='same')
            breath1 = np_move_avg(breath1, 5, mode='same')
            # breath2 = np_move_avg((data_short[2, :]), 10, mode='valid')
            self.curve1.setData(np.array(breath1))  # 图1 呼吸图
            self.curve3.setData(np.array(np_move_avg((data_short[0, :]), 5, mode='valid')))# 图2 呼吸图
            # self.curve3.setData(np.array(demo_2)[600:1600])  # 心率图

            data_detrend = data_long[0, N-600:N-1] - np.min(data_long[0, N-600:N-1])
            data_max = np.max(data_detrend)
            self.value_set.setText(str(np.round(data_max, 3)))

            # 设置幅度数字
            self.uua = 0
            aaa = data_long[1, 1500:2000]
            aaa = aaa - np.min(aaa)
            bbb = data_long[2, 1500:2000]
            bbb = bbb - np.min(bbb)
            for i in range(500):
                self.uua = self.uua + aaa[i] * aaa[i] + bbb[i] * bbb[i]
            self.value_amp = np.around(self.uua, 4)
            print(self.value_amp)

            # 显示频谱数字
            u1 = data_long[1, :] - np.mean(data_long[1, :])
            u2 = data_long[2, :] - np.mean(data_long[2, :])

            new = u1 + 1j * u2
            data_raw_full = np.append(new, np.zeros(N))
            data_fft = fft(data_raw_full)
            data_norm = np.abs(data_fft)
            data_norm_half = data_norm[range(100)]
            freq_slice = data_norm_half[5:90]
            freq_sum = np.sum(freq_slice)
            # num_re = int((np.argmax(bre_slice) + 5) * 1.5)
            # num_hb = int((np.argmax(heart_slice) + 34) * 1.5)
            # axis_freq = np.linspace(0, 150, 100)
            # self.curve4.setData(axis_freq, data_norm_half)  # 图2 心跳图
            # print(freq_sum)

            # if data_max < 1.56:
            #____________________________________________________________________________________________________
            if self.value_amp < 0.1:
                self.people_set = 0

                # if self.flag_state == 1:
                self.label_state.setText('OK')

            else:
                self.label_state.setText('OK！')
                self.people_set = 1

            # self.curve2.setData(np.array(data_short[1, :]))  # 图1 呼吸图
            # self.curve7.setData(np.array(data_short[2, :]))  # 图1 呼吸图


        else:
            self.label_state.setText('...')

        # pass

    # 找圆心
    def _fresh_cirfit(self):
        # 在导入模式下，不需要实时计算圆心
        if self.flag_import_mode == 1:
            return
            
        [self.crf, self.crf_sqrt] = _cir_fit(data_long[1, N-1000:-1], data_long[2, N-1000:-1])
        # print('crf', self.crf)
        # print('sqrt', self.crf_sqrt)

    # 数字更新
    def _fetch_value(self):
        global data_short, data_long, n, N, dsp_long, hb, demo_2, num_hb, num_re, heart_history_short, heart_history_long

        # # 获取实圆拟合参数
        # [self.crf, self.crf_sqrt] = _cir_fit(data_long[1, :], data_long[2, :])
        # print('crf', self.crf)
        # print('sqrt', self.crf_sqrt)

        # 在导入模式下，只更新时间显示，不更新其他数值
        if self.flag_import_mode == 1:
            # 显示时间刷新
            self.label_time_day.setText(str(time.strftime('%Y/%m/%d', time.localtime(time.time()))))
            self.label_time_min.setText(str(time.strftime('%H:%M', time.localtime(time.time()))))
            return

        if self.cnt_init == 72400:
            # 心跳稳定程序
            now_display_heart = int(np.mean(heart_history_short))
            self.value_h.setText(str(now_display_heart))
            # self.value_h_2.setText(str(int(np.mean(heart_history_long))))

            # 画心跳频率图
            hb[:-1] = hb[1:]
            hb[-1] = now_display_heart
            re[:-1] = re[1:]

            if self.people_set == 1:
                self.value_b.setText(str(num_re))
                re[-1] = num_re
            else:
                self.value_b.setText("无")
                re[-1] = 0

            self.curve7.setData(np.array(hb))  # 图2 心跳历史图
            self.curve2.setData(np.array(re))  # 图2 呼吸历史图

            # mean_history = int(np.mean(heart_history))
            # if np.abs(num_hb - mean_history) < heart_delta:
            #     disp_now = num_hb
            # else:
            #     disp_now = mean_history + _sign(num_hb - mean_history)
            #
            # # history更新
            # heart_history[:-1] = heart_history[1:]
            # heart_history[-1] = disp_now

            # self.value_b.setText(str(num_re))
            # self.value_h.setText(str(num_hb))

            # if self.PEOPLEAVAILABLE == 1:
            #     self.value_b.setText(str(num_re))
            #     self.value_h.setText(str(disp_now))
            # else:
            #     self.value_b.setText("-")
            #     self.value_h.setText("-")
            #


        # 显示时间刷新
        self.label_time_day.setText(str(time.strftime('%Y/%m/%d', time.localtime(time.time()))))
        self.label_time_min.setText(str(time.strftime('%H:%M', time.localtime(time.time()))))

        # pass

    # @staticmethod
    # 串口读数刷新
    def _fresh_ser(self):
        global data_short, data_long, n, dsp_short, N, dsp_long, num_re, num_hb, write_counter

        # 如果在导入模式下，不执行串口读取
        if self.flag_import_mode == 1:
            return

        # 如果串口未连接，不执行串口读取
        if not self.serial_connected or self.ser_c is None:
            return

        try:
            line = self.ser_c.readline().decode('utf-8').strip('\r\n')
            temp = line.split(' ')
            # temp[0] = line[2:7]
            # temp[1] = line[7:12]
        except Exception as e:
            print(f"串口读取错误: {e}")
            return
        
        if temp[0] != '':

            # 初始化积累数据
            if self.cnt_init < 200:
                self.cnt_init = self.cnt_init + 1
            else:
                self.cnt_init = 72400

            for i in range(2):
                # temp[i] = float(temp[i])
                # temp[i] = float((int(temp[i])) * 1 / 1024) - 1
                temp[i] = float(((int(temp[i]) / 32767) + 1) * 3.3 / 2)
            # temp[2] = temp[2] / 1024 - 0.9
            # temp[3] = temp[3] / 1024 - 0.9

        data_short[:, :-1] = data_short[:, 1:]
        data_short[1, n - 1] = temp[0]
        data_short[2, n - 1] = temp[1]
        data_long[:, :-1] = data_long[:, 1:]
        data_long[1, N - 1] = temp[0]
        data_long[2, N - 1] = temp[1]
        # data_long[1,:] = np_move_avg((data_long[1, :]), 5, mode='same')
        # data_long[2, :] = np_move_avg((data_long[2, :]), 5, mode='same')
        # data_short[1,:] = np_move_avg((data_short[1, :]), 5, mode='same')
        # data_short[2, :] = np_move_avg((data_short[2, :]), 5, mode='same')

        # 当self.flag_record = 1 时记录数据，频率同串口输出率；
        if self.flag_record != 0:
            # if write_counter == 200:
            self.f.write(time.strftime('%Y-%m-%d-%H-%M-%S', time.localtime(time.time())))
            self.f.write('  ')
            self.f.write(str(data_short[1, -1]))
            self.f.write('  ')
            self.f.write(str(data_short[2, -1]))
            self.f.write('\n')
            self.button_record.setText("结束记录数据")
            self.button_record.setStyleSheet("background-color: rgb(255, 30, 30); color: black; font: 150 40pt "
                                             "\"微软雅黑\"; ")
            # write_counter = 0
            # else:
            #     write_counter = write_counter + 1

        else:
            self.button_record.setText("开始记录数据")
            self.button_record.setStyleSheet("background-color: rgb(50, 255, 50); color: black; font: 150 40pt "
                                             "\"微软雅黑\";")
            # self.f.close()

        # 同步解调数据
        if self.cnt_init == 72400:

            i3 = (data_short[1, -3] - self.crf[0]) / self.crf_sqrt
            i2 = (data_short[1, -2] - self.crf[0]) / self.crf_sqrt
            i1 = (data_short[1, -1] - self.crf[0]) / self.crf_sqrt
            q3 = (data_short[2, -3] - self.crf[1]) / self.crf_sqrt
            q2 = (data_short[2, -2] - self.crf[1]) / self.crf_sqrt
            q1 = (data_short[2, -1] - self.crf[1]) / self.crf_sqrt

        # Arcs 方法
            data_long[0, N-1] = data_long[0, N-2] + np.arcsin(
                (i2 * q1 - i1 * q2) / np.sqrt((i2 ** 2 + q2 ** 2) * (i1 ** 2 + q1 ** 2)))

            data_short[0, n - 1] = data_short[0, n - 2] + np.arcsin(
                (i2 * q1 - i1 * q2) / np.sqrt((i2 ** 2 + q2 ** 2) * (i1 ** 2 + q1 ** 2)))


        # ACAA 方法
        # dsp_short[-1] = dsp_short[-2] + np.arcsin(
        #     ((i1-i2)*(q2-q3)-(q1-q2)*(i2-i3)) /
        #     np.sqrt(((i1-i2) ** 2 + (q1-q2) ** 2) * ((i2-i3) ** 2 + (q2-q3) ** 2)))

        # dsp_long[:-1] = dsp_long[1:]
        # dsp_long[-1] = dsp_short[-1]

        # pass

    # 记录数据的button定义
    def _set_button_att(self):
        self.flag_record = (1 + self.flag_record) % 2
        print('Button !!! flag_record = ', self.flag_record)
        if self.flag_record == 1:
            self.cnt_record = self.cnt_record + 1
            str3 = './data/' + str(time.strftime('%Y-%m-%d-%H-%M', time.localtime(time.time()))) + ' - ' + str(
                self.cnt_record) + '.txt'
            print('PATH == ', str3)
            print('Start recording ...')
            self.f = open(str3, 'w+', encoding='utf-8')
        else:
            self.f.close()

    # 滑动杆的数字变化
    def _valuechange(self):
        # 将数值存入平均值中
        self.slider_value = self.horizontalSlider.value()
        self.value_setting.setText(str(self.horizontalSlider.value()))

    # 数据导入功能
    def _import_data(self):
        """导入数据文件并切换到导入模式，或退出导入模式"""
        try:
            # 如果当前是导入模式，则退出导入模式
            if self.flag_import_mode == 1:
                self._exit_import_mode()
                return
            
            # 打开文件选择对话框
            file_path, _ = QFileDialog.getOpenFileName(
                self, 
                "选择数据文件", 
                "./data/",  # 默认打开data目录
                "Text files (*.txt);;All files (*.*)"
            )
            
            if file_path:
                # 读取并解析数据文件
                self._parse_data_file(file_path)
                
                # 切换到导入模式
                self.flag_import_mode = 1
                
                # 停止串口数据读取
                self.timer_ser_refresh.stop()
                
                # 更新按钮状态
                self.button_import.setText("退出导入模式")
                self.button_import.setStyleSheet("QPushButton{\n"
                    "font: 70 40pt \"微软雅黑\";\n"
                    "background-color:rgb(150, 30, 30);\n"
                    "color:white;\n"
                    "}")
                
                # 禁用记录按钮
                self.button_record.setEnabled(False)
                
                # 更新状态显示
                self.label_state.setText("导入模式")
                
                # 立即更新显示
                self._update_imported_display()
                
                print(f"成功导入数据文件: {file_path}")
                
        except Exception as e:
            QMessageBox.warning(self, "错误", f"导入数据失败: {str(e)}")
            print(f"导入数据错误: {e}")

    def _parse_data_file(self, file_path):
        """解析数据文件"""
        global data_short, data_long, n, N
        
        try:
            # 读取文件
            with open(file_path, 'r', encoding='utf-8') as f:
                lines = f.readlines()
            
            # 解析数据
            timestamps = []
            values1 = []
            values2 = []
            
            for line in lines:
                line = line.strip()
                if line:
                    parts = line.split()
                    if len(parts) >= 3:
                        timestamps.append(parts[0])
                        values1.append(float(parts[1]))
                        values2.append(float(parts[2]))
            
            # 存储解析的数据
            self.imported_data = {
                'timestamps': timestamps,
                'values1': np.array(values1),
                'values2': np.array(values2),
                'length': len(values1)
            }
            
            print(f"解析完成，共 {len(values1)} 个数据点")
            
        except Exception as e:
            raise Exception(f"解析文件失败: {str(e)}")

    def _update_imported_display(self):
        """更新导入数据的显示"""
        global data_short, data_long, n, N
        
        if self.imported_data is None:
            return
            
        try:
            values1 = self.imported_data['values1']
            values2 = self.imported_data['values2']
            data_length = self.imported_data['length']
            
            # 清空现有数据
            data_short = np.zeros(n * 5).reshape(5, n)
            data_long = np.zeros(N * 5).reshape(5, N)
            
            # 填充导入的数据到data_long和data_short
            if data_length > 0:
                # 对于data_long (2000个点)
                if data_length >= N:
                    # 如果数据足够长，取最后N个点
                    data_long[1, :] = values1[-N:]
                    data_long[2, :] = values2[-N:]
                else:
                    # 如果数据不够长，填充到末尾
                    data_long[1, -data_length:] = values1
                    data_long[2, -data_length:] = values2
                
                # 对于data_short (1000个点)
                short_length = min(data_length, n)
                if data_length >= n:
                    data_short[1, :] = values1[-n:]
                    data_short[2, :] = values2[-n:]
                else:
                    data_short[1, -short_length:] = values1
                    data_short[2, -short_length:] = values2
                
                # 计算解调数据 (模拟实时处理逻辑)
                self._process_imported_data()
                
                # 设置cnt_init以启用显示逻辑
                self.cnt_init = 72400
                
                # 手动调用显示更新
                self._fetch_figure()
                self._fetch_value()
                
        except Exception as e:
            print(f"更新导入数据显示错误: {e}")

    def _process_imported_data(self):
        """处理导入的数据，计算解调等"""
        global data_short, data_long, n, N
        
        try:
            # 简化的圆拟合 - 使用数据的均值作为圆心
            i_data = data_long[1, :]
            q_data = data_long[2, :]
            
            # 计算圆心和半径
            self.crf = [np.mean(i_data), np.mean(q_data)]
            self.crf_sqrt = np.sqrt(np.var(i_data) + np.var(q_data))
            
            # 计算解调数据 (简化版本)
            for i in range(2, N):
                if i >= 2:
                    i3 = (data_long[1, i-2] - self.crf[0]) / self.crf_sqrt
                    i2 = (data_long[1, i-1] - self.crf[0]) / self.crf_sqrt
                    i1 = (data_long[1, i] - self.crf[0]) / self.crf_sqrt
                    q3 = (data_long[2, i-2] - self.crf[1]) / self.crf_sqrt
                    q2 = (data_long[2, i-1] - self.crf[1]) / self.crf_sqrt
                    q1 = (data_long[2, i] - self.crf[1]) / self.crf_sqrt
                    
                    # 防止除零错误
                    denominator = np.sqrt((i2 ** 2 + q2 ** 2) * (i1 ** 2 + q1 ** 2))
                    if denominator > 1e-10:
                        data_long[0, i] = data_long[0, i-1] + np.arcsin(
                            np.clip((i2 * q1 - i1 * q2) / denominator, -1, 1))
                    else:
                        data_long[0, i] = data_long[0, i-1]
            
            # 同样处理data_short
            for i in range(2, n):
                if i >= 2:
                    i3 = (data_short[1, i-2] - self.crf[0]) / self.crf_sqrt
                    i2 = (data_short[1, i-1] - self.crf[0]) / self.crf_sqrt
                    i1 = (data_short[1, i] - self.crf[0]) / self.crf_sqrt
                    q3 = (data_short[2, i-2] - self.crf[1]) / self.crf_sqrt
                    q2 = (data_short[2, i-1] - self.crf[1]) / self.crf_sqrt
                    q1 = (data_short[2, i] - self.crf[1]) / self.crf_sqrt
                    
                    denominator = np.sqrt((i2 ** 2 + q2 ** 2) * (i1 ** 2 + q1 ** 2))
                    if denominator > 1e-10:
                        data_short[0, i] = data_short[0, i-1] + np.arcsin(
                            np.clip((i2 * q1 - i1 * q2) / denominator, -1, 1))
                    else:
                        data_short[0, i] = data_short[0, i-1]
                        
        except Exception as e:
            print(f"处理导入数据错误: {e}")

    def _exit_import_mode(self):
        """退出导入模式，返回实时模式"""
        try:
            # 切换回实时模式
            self.flag_import_mode = 0
            self.imported_data = None
            
            # 恢复按钮状态
            self.button_import.setText("导入数据")
            self.button_import.setStyleSheet("QPushButton{\n"
                "font: 70 40pt \"微软雅黑\";\n"
                "background-color:rgb(30, 30, 150);\n"
                "color:white;\n"
                "}")
            
            # 根据串口连接状态恢复记录按钮和状态显示
            if self.serial_connected:
                # 重新启动串口数据读取
                self.timer_ser_refresh.start(1)
                # 启用记录按钮
                self.button_record.setEnabled(True)
                self.button_record.setText("开始记录数据")
                self.button_record.setStyleSheet("QPushButton{\n"
                    "font: 70 40pt \"微软雅黑\";\n"
                    "background-color:rgb(30, 150, 30);\n"
                    "color:white;\n"
                    "}")
                # 重置计数器
                self.cnt_init = 0
                # 更新状态显示
                self.label_state.setText("实时模式")
                print("已退出导入模式，返回实时模式")
            else:
                # 无串口模式
                self.button_record.setEnabled(False)
                self.button_record.setText("无串口连接")
                self.button_record.setStyleSheet("QPushButton{\n"
                    "font: 70 40pt \"微软雅黑\";\n"
                    "background-color:rgb(100, 100, 100);\n"
                    "color:white;\n"
                    "}")
                # 更新状态显示
                self.label_state.setText("无串口模式")
                print("已退出导入模式，返回无串口模式")
            
        except Exception as e:
            print(f"退出导入模式错误: {e}")


# # 解调算法
# def _dis_demodulation(i, q, d_):
#     pass

def _sign(a):
    if a > 0:
        return 1
    elif a == 0:
        return 0
    else:
        return -1


# 滑动平均
def np_move_avg(a, nn, mode="same"):
    return np.convolve(a, np.ones((nn,)) / nn, mode=mode)


# 圆拟合
def _cir_fit(i, q):
    len_iq = len(i)

    i_mean = np.mean(i)
    q_mean = np.mean(q)

    Mxx = 0
    Myy = 0
    Mxy = 0
    Mxz = 0
    Myz = 0
    Mzz = 0

    for ii in range(len_iq):
        Xi = i[ii] - i_mean
        Yi = q[ii] - q_mean
        Zi = Xi * Xi + Yi * Yi
        Mxy = Mxy + Xi * Yi
        Mxx = Mxx + Xi * Xi
        Myy = Myy + Yi * Yi
        Mxz = Mxz + Xi * Zi
        Myz = Myz + Yi * Zi
        Mzz = Mzz + Zi * Zi

    Mxx = Mxx / len_iq
    Myy = Myy / len_iq
    Mxy = Mxy / len_iq
    Mxz = Mxz / len_iq
    Myz = Myz / len_iq
    Mzz = Mzz / len_iq

    Mz = Mxx + Myy
    Cov_xy = Mxx * Myy - Mxy * Mxy
    Mxz2 = Mxz * Mxz
    Myz2 = Myz * Myz

    A2 = 4 * Cov_xy - 3 * Mz * Mz - Mzz
    A1 = Mzz * Mz + 4 * Cov_xy * Mz - Mxz2 - Myz2 - Mz * Mz * Mz
    A0 = Mxz2 * Myy + Myz2 * Mxx - Mzz * Cov_xy - 2 * Mxz * Myz * Mxy + Mz * Mz * Cov_xy
    A22 = A2 + A2

    epsilon = 1e-12
    y_new = 1e+20
    IterMax = 40
    x_new = 0

    for iter_num in range(IterMax):
        y_old = y_new
        y_new = A0 + x_new * (A1 + x_new * (A2 + 4 * x_new * x_new))
        if np.abs(y_new) > np.abs(y_old):
            # print('wrong direction: |y_new| > |y_old|')
            x_new = 0
            break
        Dy = A1 + x_new * (A22 + 16 * x_new * x_new)
        x_old = x_new
        x_new = x_old - y_new / Dy

        if np.abs((x_new - x_old) / x_new) < epsilon:
            break
        if iter_num >= IterMax:
            print('no results!')
            x_new = 0
        if x_new < 0:
            print(1, 'negative root:  x=%f\n', x_new)
            x_new = 0

    det = x_new * x_new - x_new * Mz + Cov_xy
    center = [Mxz * (Myy - x_new) - Myz * Mxy, Myz * (Mxx - x_new) - Mxz * Mxy] / det / 2

    return [center + [i_mean, q_mean], np.sqrt(np.dot(center, np.transpose(center)) + Mz + 2 * x_new)]
    # pass


# 主函数
def main():
    app = QApplication(sys.argv)

    # 页面定义
    ui_port = QPort()
    # ui_main = Ui_MainP_Frontend()

    # 开启主页面
    # ui_main.show()
    ui_port.show()

    # 页面切换
    ui_port.pushButtonOK.clicked.connect(lambda: {ui_port.close()})

    app.exec_()


if __name__ == '__main__':
    main()
