import numpy as np
import pandas as pd
import re

class PetCalorieEstimator:
    def __init__(self, weight_kg=10.0, fs=50):
        """
        初始化算法
        :param weight_kg: 宠物体重 (kg)，默认 10kg
        :param fs: 采样率 (Hz)，默认 50Hz
        """
        self.weight_kg = weight_kg
        self.fs = fs
        # 1. 计算基础代谢率 (RER - Resting Energy Requirement)
        # 公式: 70 * weight_kg ^ 0.75 (kcal/day)
        self.rer_daily = 70 * (self.weight_kg ** 0.75)
        # 转换为每秒的基础消耗 (BMR per second)
        self.bmr_per_sec = self.rer_daily / 86400.0
        
    def parse_ble_data(self, raw_str):
        """解析 BLE 原始字符串数据"""
        data = []
        pattern = re.compile(
            r'\[(.*?)\] ADC:(\d+) (\d+)\|Acc:([-.\d]+) ([-.\d]+) ([-.\d]+)\|Gyr:([-.\d]+) ([-.\d]+) ([-.\d]+)\|T:([-.\d]+)'
        )
        for line in raw_str.strip().split('\n'):
            match = pattern.search(line)
            if match:
                groups = match.groups()
                data.append({
                    'acc_x': float(groups[3]),
                    'acc_y': float(groups[4]),
                    'acc_z': float(groups[5]),
                    # Gyr 和 ADC 数据预留给心率算法，此处暂不使用
                })
        return pd.DataFrame(data)

    def calculate_calories(self, raw_data_str):
        """
        输入原始数据字符串，输出这段时间的卡路里消耗
        """
        df = self.parse_ble_data(raw_data_str)
        if df.empty:
            return 0.0
            
        # 1. 提取加速度数据
        acc = df[['acc_x', 'acc_y', 'acc_z']].values
        
        # 2. 计算合成加速度 (SVM)
        svm = np.sqrt(np.sum(acc**2, axis=1))
        
        # 3. 计算 ENMO (去重力运动强度)
        # max(0, SVM - 1.0g)
        enmo = np.maximum(0, svm - 1.0)
        
        # 4. 动态计算 METs (代谢当量)
        # 根据强度分级映射 METs
        # 默认 MET = 1.0 (休息)
        mets = np.ones(len(enmo))
        
        # 阈值判定 (Thresholds) - 可根据实际测试微调
        mets[enmo > 0.05] = 2.0  # 轻微活动/慢走
        mets[enmo > 0.20] = 4.0  # 中度活动/小跑
        mets[enmo > 0.50] = 6.0  # 剧烈活动/狂奔 (飞盘/敏捷赛)
        
        # 5. 积分计算总卡路里
        # Calorie = BMR_per_sec * MET * duration
        # 每个点代表 1/fs 秒
        dt = 1.0 / self.fs
        kcal_per_sample = self.bmr_per_sec * mets * dt
        total_kcal = np.sum(kcal_per_sample)
        
        return {
            "total_kcal": total_kcal,
            "duration_sec": len(df) / self.fs,
            "avg_mets": np.mean(mets),
            "activity_level": "Resting" if np.mean(enmo) < 0.02 else "Active"
        }

# ================= 使用示例 =================
# 假设这是一只 10kg 的柯基/柴犬
estimator = PetCalorieEstimator(weight_kg=10.0, fs=50)

# 你的原始数据
raw_data = """
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

result = estimator.calculate_calories(raw_data)

print("-" * 30)
print(f"宠物体重: {estimator.weight_kg} kg")
print(f"基础代谢(RER): {estimator.rer_daily:.2f} kcal/day")
print(f"样本时长: {result['duration_sec']:.2f} 秒")
print(f"当前平均 METs: {result['avg_mets']:.2f} (1.0代表静止)")
print(f"该片段消耗热量: {result['total_kcal']:.6f} kcal")
print(f"推算每小时消耗: {result['total_kcal'] * (3600/result['duration_sec']):.2f} kcal/hr")
