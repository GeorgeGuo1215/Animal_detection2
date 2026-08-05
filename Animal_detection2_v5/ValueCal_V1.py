# -*- coding: utf-8 -*-
# @First edit   : 2022-07-01 17:28
# @Author       : Olivermahout
# @FileName     : ValueCal_V1.py
# @Software     : PyCharm
# -- Copyright @ Yuchen Li, SJTU --


import numpy as np
import serial
from scipy.fftpack import fft
from Filter_parameter import HPF_par, LPF_par, HPF_short_1_par, HPF_short_2_par, HPF_short_3_par, HPF_short_4_par, \
    HPF_short_6_par, HPF_short_5_par


COM = "COM5"
N = 4000  # 长数据
data_long = np.zeros(N * 5).reshape(5, N)  # long data


def np_move_avg(a, nn, mode="same"):
    return np.convolve(a, np.ones((nn,)) / nn, mode=mode)


#
# _value_cal(data):
# input:
#       data: @(5,N)np.array
# return:
#       wave_re:    respiratory waveform @ (1,1000)np.array
#       wave_hb:    heartbeat waveform @ (1,1000)np.array
#       value_re:   respiratory rate (beats per minute) @ int
#       value_hb:   heartbeat rate (beats per minute) @ int
#
#
def _value_cal(data):
    wave_re = np_move_avg(data[0, :], 1, mode='valid')[3000:4000]
    wave_re = wave_re - np.min(wave_re)

    wave_hb = np.convolve(data[0, :], HPF_short_4_par, mode='same')
    wave_hb = np.convolve(wave_hb, HPF_short_5_par, mode='same')

    u1 = data[1, :] - np.mean(data[1, :])
    u2 = data[2, :] - np.mean(data[2, :])

    new = u1 + 1j * u2
    data_raw_full = np.append(new, np.zeros(4000))
    data_fft = fft(data_raw_full)
    data_norm = np.abs(data_fft)
    data_norm_half = data_norm[range(200)]
    bre_slice = data_norm_half[10:35]
    heart_slice = data_norm_half[70:180]
    value_re = int((np.argmax(bre_slice) + 9) * 0.75)
    value_hb = int((np.argmax(heart_slice) + 69) * 0.75)

    return wave_re, wave_hb, value_re, value_hb


# 主函数
def main():

    ser_c = serial.Serial(COM, 115200)
    line = ser_c.readline().decode('utf-8').strip('\r\n')
    temp = line.split(' ')
    data_long[:, :-1] = data_long[:, 1:]
    data_long[:, N-1] = temp

    demo1, demo2, num_re, num_hb = _value_cal(data_long)


if __name__ == '__main__':
    main()









