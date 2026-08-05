import numpy as np
import pandas as pd
from scipy.signal import butter, filtfilt, find_peaks
import re

class PetActivityAlgorithm:
    def __init__(self, fs=50):
        """
        初始化算法模块
        :param fs: 采样率 (Hz), 默认为 50Hz
        """
        self.fs = fs
        # 初始化计步用的带通滤波器 (0.5Hz - 5.0Hz)
        # 针对宠物步态，滤除直流重力分量和高频噪声
        self.b, self.a = butter(2, [0.5, 5], btype='bandpass', fs=fs)
        
    def parse_raw_data(self, raw_str):
        """
        解析原始 BLE 日志数据
        """
        data = []
        # 正则匹配格式: [Time] ADC:val1 val2|Acc:x y z|Gyr:x y z|T:temp
        pattern = re.compile(
            r'ADC:(\d+)\s+(\d+)\|Acc:([-.\d]+)\s+([-.\d]+)\s+([-.\d]+)\|Gyr:([-.\d]+)\s+([-.\d]+)\s+([-.\d]+)\|T:([-.\d]+)'
        )
        
        for line in raw_str.strip().split('\n'):
            match = pattern.search(line)
            if match:
                groups = match.groups()
                row = {
                    'adc_ppg_ir': int(groups[0]),   # 假设 ADC1 为红外/绿光
                    'adc_ppg_red': int(groups[1]),  # 假设 ADC2 为红光/环境光
                    'acc_x': float(groups[2]),
                    'acc_y': float(groups[3]),
                    'acc_z': float(groups[4]),
                    'gyr_x': float(groups[5]),
                    'gyr_y': float(groups[6]),
                    'gyr_z': float(groups[7]),
                    'temp': float(groups[8])
                }
                data.append(row)
        return pd.DataFrame(data)

    def calculate_activity_metrics(self, acc_data_g):
        """
        量化活动量 (Activity Volume)
        :param acc_data_g: N x 3 的加速度 numpy 数组 (单位: g)
        :return: dict 包含 ENMO (能量消耗代理) 和 MAD (强度指标)
        """
        # 1. 计算合加速度 (Signal Vector Magnitude, SVM)
        svm = np.sqrt(np.sum(acc_data_g**2, axis=1))
        
        # 2. ENMO (Euclidean Norm Minus One)
        # 公式: max(0, SVM - 1g)。去除重力，只保留运动产生的加速度。
        # 这是目前科研界最常用的活动量化指标 (类似 Apple Watch 的 Move Ring 基础)
        enmo = np.maximum(0, svm - 1.0)
        activity_counts = np.sum(enmo) # 这一段数据内的总活动量
        
        # 3. MAD (Mean Amplitude Deviation)
        # 描述加速度偏离平均值的程度，对传感器重力校准误差不敏感，非常鲁棒
        mad = np.mean(np.abs(svm - np.mean(svm)))
        
        return {
            "enmo_total": activity_counts,
            "mad_intensity": mad,
            "is_active": mad > 0.02 # 简单的活跃/静止判断阈值
        }

    def count_steps(self, acc_data_g):
        """
        计算步数 (Step Counter)
        :param acc_data_g: N x 3 的加速度 numpy 数组
        :return: 步数 (int)
        """
        # 1. 计算合加速度
        svm = np.sqrt(np.sum(acc_data_g**2, axis=1))
        
        # 2. 数据长度检查 (滤波需要一定长度)
        if len(svm) < 15:
            return 0
            
        # 3. 带通滤波 (提取步伐产生的震荡信号)
        # 使用 filtfilt 进行零相位滤波，避免波形偏移
        filt_svm = filtfilt(self.b, self.a, svm)
        
        # 4. 峰值检测 (Peak Detection)
        # height: 最小峰值高度 (0.05g 对应非常轻微的动作，跑动通常 > 0.5g)
        # distance: 最小步间距 (50Hz / 4 = 12.5个点，即限制每秒最多 4 步，防止误判)
        peaks, _ = find_peaks(filt_svm, height=0.05, distance=self.fs//4)
        
        return len(peaks)

    def process_heart_respiratory_rate(self, adc_buffer, acc_buffer):
        """
        [接口] 计算心率与呼吸率
        :param adc_buffer: 原始 PPG 光电信号
        :param acc_buffer: 对应的加速度信号 (用于运动伪影消除)
        :return: (HeartRate, RespiratoryRate)
        """
        # TODO: 这里实现 PPG 信号处理算法
        # 1. 信号预处理 (带通滤波 0.5-4Hz 提取心率, 0.1-0.5Hz 提取呼吸)
        # 2. 自适应滤波 (LMS/RLS) 使用 Acc 去除运动噪声
        # 3. FFT 或 峰值检测计算频率
        print(f"Processing HR/RR on {len(adc_buffer)} samples...")
        return 0, 0 # Placeholder

# ==========================================
# 模拟运行 (使用你提供的数据)
# ==========================================

# 1. 原始数据字符串
raw_log = """
[1:37:20 AM] ADC:8118 12161|Acc:-0.153 0.027 0.943|Gyr:-6.2 0.5 -0.4|T:23.6
[1:37:20 AM] ADC:8186 12076|Acc:-0.153 0.028 0.942|Gyr:-6.5 0.2 -0.6|T:23.6
[1:37:20 AM] ADC:8092 12158|Acc:-0.154 0.027 0.941|Gyr:-6.6 0.5 -0.3|T:23.6
[1:37:20 AM] ADC:8223 12133|Acc:-0.152 0.029 0.942|Gyr:-6.4 0.5 -0.6|T:23.6
[1:37:20 AM] ADC:8180 12080|Acc:-0.153 0.028 0.941|Gyr:-6.4 0.5 -0.5|T:23.6
[1:37:20 AM] ADC:8160 12088|Acc:-0.152 0.028 0.942|Gyr:-6.5 0.6 -0.6|T:23.6
[1:37:20 AM] ADC:8121 12170|Acc:-0.154 0.027 0.942|Gyr:-6.3 0.3 -0.4|T:23.6
[1:37:20 AM] ADC:8196 12139|Acc:-0.151 0.029 0.941|Gyr:-6.3 0.3 -0.2|T:23.6
[1:37:20 AM] ADC:8200 12136|Acc:-0.152 0.027 0.942|Gyr:-6.5 0.5 -0.3|T:23.6
[1:37:20 AM] ADC:8228 12101|Acc:-0.152 0.028 0.941|Gyr:-6.4 0.2 -0.5|T:23.6
[1:37:20 AM] ADC:8197 12086|Acc:-0.152 0.028 0.943|Gyr:-6.4 0.3 -0.5|T:23.6
[1:37:20 AM] ADC:8176 12106|Acc:-0.152 0.027 0.941|Gyr:-6.5 0.4 -0.6|T:23.6
[1:37:20 AM] ADC:8176 12108|Acc:-0.152 0.027 0.940|Gyr:-6.4 0.3 -0.5|T:23.6
[1:37:20 AM] ADC:8171 12105|Acc:-0.154 0.027 0.942|Gyr:-6.4 0.3 -0.4|T:23.6
[1:37:20 AM] ADC:8164 12110|Acc:-0.152 0.027 0.941|Gyr:-6.4 0.4 -0.5|T:23.6
[1:37:20 AM] ADC:8162 12114|Acc:-0.152 0.027 0.940|Gyr:-6.3 0.3 -0.5|T:23.6
[1:37:20 AM] ADC:8170 12102|Acc:-0.153 0.027 0.941|Gyr:-6.3 0.5 -0.7|T:23.6
[1:37:20 AM] ADC:8157 12102|Acc:-0.153 0.029 0.940|Gyr:-6.5 0.4 -0.5|T:23.6
[1:37:20 AM] ADC:8168 12106|Acc:-0.151 0.028 0.942|Gyr:-6.2 0.3 -0.4|T:23.6
[1:37:20 AM] ADC:8178 12099|Acc:-0.153 0.028 0.940|Gyr:-6.4 0.4 -0.4|T:23.6
[1:37:20 AM] ADC:8182 12101|Acc:-0.152 0.029 0.943|Gyr:-6.4 0.6 -0.4|T:23.6
[1:37:20 AM] ADC:8183 12104|Acc:-0.152 0.027 0.940|Gyr:-6.3 0.4 -0.6|T:23.6
[1:37:20 AM] ADC:8193 12096|Acc:-0.152 0.027 0.942|Gyr:-6.4 0.5 -0.4|T:23.6
[1:37:20 AM] ADC:8205 12100|Acc:-0.151 0.027 0.943|Gyr:-6.4 0.4 -0.5|T:23.6
"""

# 初始化算法
processor = PetActivityAlgorithm(fs=50)

# 解析数据
df = processor.parse_raw_data(raw_log)
acc_data = df[['acc_x', 'acc_y', 'acc_z']].values

# 计算指标
metrics = processor.calculate_activity_metrics(acc_data)
steps = processor.count_steps(acc_data)
hr, rr = processor.process_heart_respiratory_rate(df['adc_ppg_ir'].values, acc_data)

print("-" * 30)
print(f"解析样本数: {len(df)}")
print(f"活动量 (ENMO): {metrics['enmo_total']:.4f} (该值累积越高代表运动量越大)")
print(f"活动强度 (MAD): {metrics['mad_intensity']:.4f} (该值越大代表运动越剧烈)")
print(f"检测步数: {steps} 步")
print(f"活跃状态: {'Active' if metrics['is_active'] else 'Resting'}")