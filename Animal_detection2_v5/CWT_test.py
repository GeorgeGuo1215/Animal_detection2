# -*- coding: utf-8 -*-
# @First edit   : 2022-03-02 11:54
# @Author       : Oliver
# @FileName     : CWT_test.py
# @Software     : PyCharm
# -- Copyright @ Yuchen Li, SJTU --

import pywt
import numpy

data0 = numpy.zeros(100).reshape(1, 100)
[] = pywt.cwt(data0, 32, wavelet='db4')


