# -*- coding: utf-8 -*-
"""
毫米波雷达数据处理程序
独立处理data目录中的生理信号数据文件

功能：
1. 读取并解析数据文件
2. 圆拟合校正I/Q信号
3. ARCSIN相位解调
4. 数字滤波处理
5. FFT频谱分析
6. 提取心率和呼吸频率
7. 批量处理和结果可视化

作者：基于原始main.py提取
日期：2024
"""

import numpy as np
import os
import glob
import matplotlib.pyplot as plt
import matplotlib
from scipy.fftpack import fft
from scipy import signal
import pandas as pd
from datetime import datetime
import argparse
import platform

# 配置中文字体支持
def setup_chinese_font():
    """配置matplotlib中文字体支持"""
    system = platform.system()
    
    if system == "Darwin":  # macOS
        # macOS系统字体
        fonts = ['Arial Unicode MS', 'PingFang SC', 'Hiragino Sans GB', 'STHeiti', 'SimHei']
    elif system == "Windows":  # Windows
        # Windows系统字体
        fonts = ['SimHei', 'Microsoft YaHei', 'KaiTi', 'FangSong', 'STSong']
    else:  # Linux
        # Linux系统字体
        fonts = ['WenQuanYi Micro Hei', 'Noto Sans CJK SC', 'SimHei', 'DejaVu Sans']
    
    # 尝试设置字体
    for font in fonts:
        try:
            matplotlib.rcParams['font.sans-serif'] = [font]
            matplotlib.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题
            
            # 测试字体是否可用
            fig, ax = plt.subplots(figsize=(1, 1))
            ax.text(0.5, 0.5, '测试中文', fontsize=10, ha='center')
            plt.close(fig)
            
            print(f"✓ 中文字体设置成功: {font}")
            return True
        except Exception as e:
            continue
    
    # 如果都不可用，尝试使用系统默认字体
    try:
        matplotlib.rcParams['font.sans-serif'] = ['sans-serif']
        matplotlib.rcParams['axes.unicode_minus'] = False
        print("⚠️  使用系统默认字体，中文显示可能不正常")
        print("   建议运行 python test_chinese_font.py 检查字体配置")
        return False
    except:
        print("❌ 字体配置失败，图表可能无法正常显示")
        return False

# 初始化中文字体
try:
    setup_chinese_font()
except Exception as e:
    print(f"字体初始化失败: {e}")
    print("将使用默认字体设置")

# 滤波器参数（从Filter_parameter.py提取关键部分）
# 高通滤波器参数（简化版本）
HPF_short_4_par = np.array([0,-5.50164604360785e-05,-0.000170712307325988,-0.000321372765076530,-0.000523025797360845,-0.000744196112614598,-0.000983826634810562,-0.00127235081696804,-0.00160419939233453,-0.00193993867530017,-0.00228082848478637,-0.00264302585359132,-0.00301758574544834,-0.00343699236516377,-0.00390408151456911,-0.00439597535068256,-0.00492064436945537,-0.00544043127996703,-0.00594971362380537,-0.00647249773797565,-0.00699784286438748,-0.00754300147293485,-0.00810999030682790,-0.00868059148941056,-0.00925842014653436,-0.00988031890708563,-0.0105471202298266,-0.0112435445759278,-0.0119790033062446,-0.0127275069499252,-0.0134874967666727,-0.0142798607935558,-0.0150989968242221,-0.0159037541140797,-0.0166924941429137,-0.0174807050035277,-0.0182572362317448,-0.0190510330770520,-0.0198629881240401,-0.0206687276426293,-0.0214738649139352,-0.0223007879891433,-0.0231482012320617,-0.0240087850093419,-0.0248875902870985,-0.0257644553582338,-0.0266380712009247,-0.0275223504264536])

HPF_short_5_par = np.array([-0.00339276769147847,-0.000289899591150812,-0.000301275418827859,-0.000312098312620977,-0.000322549062240940,-0.000332381770852629,-0.000341731680987501,-0.000350400526126641,-0.000358525665298154,-0.000365887160504275,-0.000372637078486777,-0.000378527224152246,-0.000383744164044131,-0.000388006414911702,-0.000391501840818984,-0.000393924679427624,-0.000395478711351671,-0.000395850492619180,-0.000395247988442467,-0.000393338704009786,-0.000390363416943796,-0.000385999580496542,-0.000380553095135587,-0.000373688291692988,-0.000365770714463747,-0.000356439027816037,-0.000346128807188419,-0.000334416670053275,-0.000321755744087934,-0.000307592555574287,-0.000292389547082744,-0.000275452034383983,-0.000257331448332866,-0.000237239705447390,-0.000216136464554341,-0.000193292824624094,-0.000170553182210679,-0.000146675220298725,-0.000123762543021994,-9.27732390070040e-05,-6.62743648944459e-05,-3.81338877599648e-05,-7.62116480334422e-06,2.26657868080909e-05,5.46318749383660e-05])

class RadarDataProcessor:
    """毫米波雷达数据处理器"""
    
    def __init__(self, sampling_rate=100):
        """
        初始化处理器
        
        参数:
            sampling_rate: 采样率 (Hz)
        """
        self.fs = sampling_rate
        self.n_short = 1000  # 短数据长度
        self.N_long = 2000   # 长数据长度
        
    def read_data_file(self, file_path):
        """
        读取数据文件
        
        参数:
            file_path: 数据文件路径
            
        返回:
            timestamps: 时间戳列表
            i_data: I通道数据
            q_data: Q通道数据
        """
        timestamps = []
        i_data = []
        q_data = []
        
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                lines = f.readlines()
            
            for line in lines:
                line = line.strip()
                if line:
                    parts = line.split()
                    if len(parts) >= 3:
                        timestamps.append(parts[0])
                        i_data.append(float(parts[1]))
                        q_data.append(float(parts[2]))
            
            return timestamps, np.array(i_data), np.array(q_data)
            
        except Exception as e:
            print(f"读取文件 {file_path} 失败: {e}")
            return None, None, None
    
    def circle_fitting(self, i_data, q_data):
        """
        圆拟合算法 - 校正I/Q信号的直流偏移和幅度不平衡
        
        参数:
            i_data: I通道数据
            q_data: Q通道数据
            
        返回:
            center: 圆心坐标 [i_center, q_center]
            radius: 圆半径
        """
        len_iq = len(i_data)
        
        i_mean = np.mean(i_data)
        q_mean = np.mean(q_data)
        
        # 计算矩阵元素
        Mxx = Myy = Mxy = Mxz = Myz = Mzz = 0
        
        for ii in range(len_iq):
            Xi = i_data[ii] - i_mean
            Yi = q_data[ii] - q_mean
            Zi = Xi * Xi + Yi * Yi
            Mxy += Xi * Yi
            Mxx += Xi * Xi
            Myy += Yi * Yi
            Mxz += Xi * Zi
            Myz += Yi * Zi
            Mzz += Zi * Zi
        
        # 归一化
        Mxx /= len_iq
        Myy /= len_iq
        Mxy /= len_iq
        Mxz /= len_iq
        Myz /= len_iq
        Mzz /= len_iq
        
        Mz = Mxx + Myy
        Cov_xy = Mxx * Myy - Mxy * Mxy
        Mxz2 = Mxz * Mxz
        Myz2 = Myz * Myz
        
        # 求解三次方程
        A2 = 4 * Cov_xy - 3 * Mz * Mz - Mzz
        A1 = Mzz * Mz + 4 * Cov_xy * Mz - Mxz2 - Myz2 - Mz * Mz * Mz
        A0 = Mxz2 * Myy + Myz2 * Mxx - Mzz * Cov_xy - 2 * Mxz * Myz * Mxy + Mz * Mz * Cov_xy
        A22 = A2 + A2
        
        # 牛顿迭代法求解
        epsilon = 1e-12
        y_new = 1e+20
        IterMax = 40
        x_new = 0
        
        for iter_num in range(IterMax):
            y_old = y_new
            y_new = A0 + x_new * (A1 + x_new * (A2 + 4 * x_new * x_new))
            if np.abs(y_new) > np.abs(y_old):
                x_new = 0
                break
            Dy = A1 + x_new * (A22 + 16 * x_new * x_new)
            x_old = x_new
            x_new = x_old - y_new / Dy
            
            if np.abs((x_new - x_old) / x_new) < epsilon:
                break
            if iter_num >= IterMax:
                print('圆拟合迭代未收敛!')
                x_new = 0
            if x_new < 0:
                print(f'圆拟合得到负根: x={x_new}')
                x_new = 0
        
        # 计算圆心和半径
        det = x_new * x_new - x_new * Mz + Cov_xy
        if det != 0:
            center = np.array([Mxz * (Myy - x_new) - Myz * Mxy, 
                              Myz * (Mxx - x_new) - Mxz * Mxy]) / det / 2
            center = center + np.array([i_mean, q_mean])
            radius = np.sqrt(np.dot(center - np.array([i_mean, q_mean]), 
                                   center - np.array([i_mean, q_mean])) + Mz + 2 * x_new)
        else:
            # 简化处理：使用均值作为圆心
            center = np.array([i_mean, q_mean])
            radius = np.sqrt(np.var(i_data) + np.var(q_data))
        
        return center, radius
    
    def arcsin_demodulation(self, i_data, q_data, center, radius):
        """
        ARCSIN相位解调算法
        
        参数:
            i_data: I通道数据
            q_data: Q通道数据
            center: 圆心坐标
            radius: 圆半径
            
        返回:
            phase_data: 解调后的相位数据
        """
        data_length = len(i_data)
        phase_data = np.zeros(data_length)
        
        # 归一化I/Q数据
        i_norm = (i_data - center[0]) / radius
        q_norm = (q_data - center[1]) / radius
        
        # ARCSIN解调
        for i in range(2, data_length):
            i3 = i_norm[i-2]
            i2 = i_norm[i-1]
            i1 = i_norm[i]
            q3 = q_norm[i-2]
            q2 = q_norm[i-1]
            q1 = q_norm[i]
            
            # 防止除零错误
            denominator = np.sqrt((i2**2 + q2**2) * (i1**2 + q1**2))
            if denominator > 1e-10:
                phase_increment = np.arcsin(
                    np.clip((i2 * q1 - i1 * q2) / denominator, -1, 1))
                phase_data[i] = phase_data[i-1] + phase_increment
            else:
                phase_data[i] = phase_data[i-1]
        
        return phase_data
    
    def moving_average(self, data, window_size, mode="same"):
        """滑动平均滤波"""
        return np.convolve(data, np.ones(window_size) / window_size, mode=mode)
    
    def apply_filters(self, phase_data):
        """
        应用数字滤波器
        
        参数:
            phase_data: 相位数据
            
        返回:
            respiratory_wave: 呼吸波形
            heartbeat_wave: 心跳波形
        """
        # 呼吸波形提取（低频成分）
        respiratory_wave = self.moving_average(phase_data, 10, mode='same')
        respiratory_wave = self.moving_average(respiratory_wave, 5, mode='same')
        respiratory_wave = respiratory_wave - np.min(respiratory_wave)
        
        # 心跳波形提取（高频滤波）
        heartbeat_wave = np.convolve(phase_data, HPF_short_4_par, mode='same')
        heartbeat_wave = np.convolve(heartbeat_wave, HPF_short_5_par, mode='same')
        
        return respiratory_wave, heartbeat_wave
    
    def extract_vital_signs(self, i_data, q_data, phase_data):
        """
        提取生理参数（心率和呼吸频率）
        
        参数:
            i_data: I通道数据
            q_data: Q通道数据
            phase_data: 相位数据
            
        返回:
            heart_rate: 心率 (bpm)
            respiratory_rate: 呼吸频率 (bpm)
        """
        # 去除直流分量
        u1 = i_data - np.mean(i_data)
        u2 = q_data - np.mean(q_data)
        
        # 构造复数信号
        complex_signal = u1 + 1j * u2
        
        # 零填充提高频率分辨率
        padded_signal = np.append(complex_signal, np.zeros(len(complex_signal)))
        
        # FFT分析
        fft_result = fft(padded_signal)
        magnitude = np.abs(fft_result)
        
        # 取前半部分频谱
        half_spectrum = magnitude[:len(magnitude)//4]  # 进一步减少范围
        
        # 频率轴
        freq_axis = np.linspace(0, self.fs/4, len(half_spectrum))
        
        # 呼吸频率范围 (0.1-0.5 Hz, 对应6-30 bpm)
        breath_freq_range = (freq_axis >= 0.1) & (freq_axis <= 0.5)
        breath_spectrum = half_spectrum[breath_freq_range]
        breath_freqs = freq_axis[breath_freq_range]
        
        # 心率频率范围 (0.8-3.0 Hz, 对应48-180 bpm)
        heart_freq_range = (freq_axis >= 0.8) & (freq_axis <= 3.0)
        heart_spectrum = half_spectrum[heart_freq_range]
        heart_freqs = freq_axis[heart_freq_range]
        
        # 找到峰值频率
        if len(breath_spectrum) > 0 and np.max(breath_spectrum) > 0:
            breath_peak_idx = np.argmax(breath_spectrum)
            breath_freq_hz = breath_freqs[breath_peak_idx]
            respiratory_rate = int(breath_freq_hz * 60)  # 转换为bpm
        else:
            respiratory_rate = 0
        
        if len(heart_spectrum) > 0 and np.max(heart_spectrum) > 0:
            heart_peak_idx = np.argmax(heart_spectrum)
            heart_freq_hz = heart_freqs[heart_peak_idx]
            heart_rate = int(heart_freq_hz * 60)  # 转换为bpm
        else:
            heart_rate = 0
        
        return heart_rate, respiratory_rate
    
    def process_single_file(self, file_path, plot_results=False):
        """
        处理单个数据文件
        
        参数:
            file_path: 文件路径
            plot_results: 是否绘制结果图
            
        返回:
            result: 处理结果字典
        """
        print(f"正在处理文件: {os.path.basename(file_path)}")
        
        # 读取数据
        timestamps, i_data, q_data = self.read_data_file(file_path)
        if i_data is None:
            return None
        
        print(f"  数据点数: {len(i_data)}")
        
        # 圆拟合校正
        center, radius = self.circle_fitting(i_data, q_data)
        print(f"  圆心: ({center[0]:.4f}, {center[1]:.4f}), 半径: {radius:.4f}")
        
        # 相位解调
        phase_data = self.arcsin_demodulation(i_data, q_data, center, radius)
        
        # 滤波处理
        respiratory_wave, heartbeat_wave = self.apply_filters(phase_data)
        
        # 提取生理参数
        heart_rate, respiratory_rate = self.extract_vital_signs(i_data, q_data, phase_data)
        
        print(f"  心率: {heart_rate} bpm")
        print(f"  呼吸频率: {respiratory_rate} bpm")
        
        # 构造结果
        result = {
            'file_path': file_path,
            'file_name': os.path.basename(file_path),
            'data_points': len(i_data),
            'heart_rate': heart_rate,
            'respiratory_rate': respiratory_rate,
            'circle_center': center,
            'circle_radius': radius,
            'timestamps': timestamps,
            'i_data': i_data,
            'q_data': q_data,
            'phase_data': phase_data,
            'respiratory_wave': respiratory_wave,
            'heartbeat_wave': heartbeat_wave
        }
        
        # 绘制结果
        if plot_results:
            self.plot_results(result)
        
        return result
    
    def plot_results(self, result):
        """
        绘制处理结果
        
        参数:
            result: 处理结果字典
        """
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        fig.suptitle(f'数据处理结果 - {result["file_name"]}', fontsize=16)
        
        # I/Q信号图
        axes[0, 0].plot(result['i_data'][:1000], label='I通道', alpha=0.7)
        axes[0, 0].plot(result['q_data'][:1000], label='Q通道', alpha=0.7)
        axes[0, 0].set_title('原始I/Q信号')
        axes[0, 0].set_xlabel('采样点')
        axes[0, 0].set_ylabel('幅度')
        axes[0, 0].legend()
        axes[0, 0].grid(True)
        
        # I/Q星座图
        axes[0, 1].scatter(result['i_data'][::10], result['q_data'][::10], 
                          alpha=0.5, s=1)
        axes[0, 1].plot(result['circle_center'][0], result['circle_center'][1], 
                       'ro', markersize=8, label='圆心')
        circle = plt.Circle(result['circle_center'], result['circle_radius'], 
                           fill=False, color='red', linestyle='--', label='拟合圆')
        axes[0, 1].add_patch(circle)
        axes[0, 1].set_title('I/Q星座图与圆拟合')
        axes[0, 1].set_xlabel('I通道')
        axes[0, 1].set_ylabel('Q通道')
        axes[0, 1].legend()
        axes[0, 1].grid(True)
        axes[0, 1].axis('equal')
        
        # 呼吸波形
        time_axis = np.arange(len(result['respiratory_wave'])) / self.fs
        axes[1, 0].plot(time_axis[:1000], result['respiratory_wave'][:1000])
        axes[1, 0].set_title(f'呼吸波形 (频率: {result["respiratory_rate"]} bpm)')
        axes[1, 0].set_xlabel('时间 (s)')
        axes[1, 0].set_ylabel('幅度')
        axes[1, 0].grid(True)
        
        # 心跳波形
        axes[1, 1].plot(time_axis[:1000], result['heartbeat_wave'][:1000])
        axes[1, 1].set_title(f'心跳波形 (心率: {result["heart_rate"]} bpm)')
        axes[1, 1].set_xlabel('时间 (s)')
        axes[1, 1].set_ylabel('幅度')
        axes[1, 1].grid(True)
        
        plt.tight_layout()
        plt.show()
    
    def batch_process(self, data_directory, output_csv=None, plot_summary=True):
        """
        批量处理数据文件
        
        参数:
            data_directory: 数据目录路径
            output_csv: 输出CSV文件路径
            plot_summary: 是否绘制汇总图
            
        返回:
            results: 处理结果列表
        """
        # 查找所有txt文件
        pattern = os.path.join(data_directory, "*.txt")
        files = glob.glob(pattern)
        
        if not files:
            print(f"在目录 {data_directory} 中未找到txt文件")
            return []
        
        print(f"找到 {len(files)} 个数据文件")
        
        results = []
        for file_path in sorted(files):
            result = self.process_single_file(file_path)
            if result:
                results.append(result)
        
        # 保存结果到CSV
        if output_csv and results:
            self.save_results_to_csv(results, output_csv)
        
        # 绘制汇总图
        if plot_summary and results:
            self.plot_summary(results)
        
        return results
    
    def save_results_to_csv(self, results, output_path):
        """
        保存结果到CSV文件
        
        参数:
            results: 处理结果列表
            output_path: 输出文件路径
        """
        data = []
        for result in results:
            data.append({
                '文件名': result['file_name'],
                '数据点数': result['data_points'],
                '心率(bpm)': result['heart_rate'],
                '呼吸频率(bpm)': result['respiratory_rate'],
                '圆心I': result['circle_center'][0],
                '圆心Q': result['circle_center'][1],
                '圆半径': result['circle_radius']
            })
        
        df = pd.DataFrame(data)
        df.to_csv(output_path, index=False, encoding='utf-8-sig')
        print(f"结果已保存到: {output_path}")
    
    def plot_summary(self, results):
        """
        绘制批量处理汇总图
        
        参数:
            results: 处理结果列表
        """
        if not results:
            return
        
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        fig.suptitle('批量处理结果汇总', fontsize=16)
        
        # 提取数据
        file_names = [r['file_name'] for r in results]
        heart_rates = [r['heart_rate'] for r in results]
        respiratory_rates = [r['respiratory_rate'] for r in results]
        data_points = [r['data_points'] for r in results]
        
        # 心率分布
        axes[0, 0].bar(range(len(heart_rates)), heart_rates)
        axes[0, 0].set_title('心率分布')
        axes[0, 0].set_xlabel('文件序号')
        axes[0, 0].set_ylabel('心率 (bpm)')
        axes[0, 0].grid(True)
        
        # 呼吸频率分布
        axes[0, 1].bar(range(len(respiratory_rates)), respiratory_rates)
        axes[0, 1].set_title('呼吸频率分布')
        axes[0, 1].set_xlabel('文件序号')
        axes[0, 1].set_ylabel('呼吸频率 (bpm)')
        axes[0, 1].grid(True)
        
        # 心率vs呼吸频率散点图
        axes[1, 0].scatter(heart_rates, respiratory_rates)
        axes[1, 0].set_title('心率 vs 呼吸频率')
        axes[1, 0].set_xlabel('心率 (bpm)')
        axes[1, 0].set_ylabel('呼吸频率 (bpm)')
        axes[1, 0].grid(True)
        
        # 数据点数分布
        axes[1, 1].bar(range(len(data_points)), data_points)
        axes[1, 1].set_title('数据点数分布')
        axes[1, 1].set_xlabel('文件序号')
        axes[1, 1].set_ylabel('数据点数')
        axes[1, 1].grid(True)
        
        plt.tight_layout()
        plt.show()
        
        # 打印统计信息
        print("\n=== 处理结果统计 ===")
        print(f"处理文件数: {len(results)}")
        print(f"平均心率: {np.mean(heart_rates):.1f} ± {np.std(heart_rates):.1f} bpm")
        print(f"平均呼吸频率: {np.mean(respiratory_rates):.1f} ± {np.std(respiratory_rates):.1f} bpm")
        print(f"心率范围: {min(heart_rates)} - {max(heart_rates)} bpm")
        print(f"呼吸频率范围: {min(respiratory_rates)} - {max(respiratory_rates)} bpm")


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='毫米波雷达数据处理程序')
    parser.add_argument('--data_dir', '-d', default='./data', 
                       help='数据目录路径 (默认: ./data)')
    parser.add_argument('--output', '-o', default='processing_results.csv',
                       help='输出CSV文件路径 (默认: processing_results.csv)')
    parser.add_argument('--single_file', '-f', default=None,
                       help='处理单个文件')
    parser.add_argument('--plot', '-p', action='store_true',
                       help='显示处理结果图')
    parser.add_argument('--no_summary', action='store_true',
                       help='不显示汇总图')
    
    args = parser.parse_args()
    
    # 创建处理器
    processor = RadarDataProcessor()
    
    if args.single_file:
        # 处理单个文件
        result = processor.process_single_file(args.single_file, plot_results=args.plot)
        if result:
            print("\n=== 处理完成 ===")
            print(f"文件: {result['file_name']}")
            print(f"心率: {result['heart_rate']} bpm")
            print(f"呼吸频率: {result['respiratory_rate']} bpm")
    else:
        # 批量处理
        results = processor.batch_process(
            args.data_dir, 
            output_csv=args.output,
            plot_summary=not args.no_summary
        )
        
        if results:
            print(f"\n=== 批量处理完成 ===")
            print(f"成功处理 {len(results)} 个文件")
            print(f"结果已保存到: {args.output}")


if __name__ == "__main__":
    main()
