import numpy as np
import sys
sys.path.append('.')
import importlib.util
spec = importlib.util.spec_from_file_location("activity", "活动量+步数.py")
activity_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(activity_module)
PetActivityAlgorithm = activity_module.PetActivityAlgorithm

# 初始化算法
processor = PetActivityAlgorithm(fs=50)

print("=" * 60)
print("ENMO值测试 - 不同活动强度")
print("=" * 60)

# 测试1: 静息状态 (SVM ≈ 1.0g)
print("\n【测试1】静息状态 (1秒 = 50样本)")
acc_rest = np.array([[0, 0, 1.0]] * 50)  # 完美静止
metrics = processor.calculate_activity_metrics(acc_rest)
print(f"  SVM平均: 1.000g")
print(f"  ENMO总和: {metrics['enmo_total']:.6f}")
print(f"  ENMO/样本: {metrics['enmo_total']/50:.6f}")
print(f"  MAD: {metrics['mad_intensity']:.6f}")

# 测试2: 轻微晃动 (SVM ≈ 1.05g)
print("\n【测试2】轻微晃动 (1秒 = 50样本)")
acc_light = []
for i in range(50):
    # 模拟轻微的正弦波动
    x = 0.05 * np.sin(2 * np.pi * i / 50)
    y = 0.05 * np.cos(2 * np.pi * i / 50)
    z = 1.0
    acc_light.append([x, y, z])
acc_light = np.array(acc_light)
metrics = processor.calculate_activity_metrics(acc_light)
svm_avg = np.mean(np.sqrt(np.sum(acc_light**2, axis=1)))
print(f"  SVM平均: {svm_avg:.3f}g")
print(f"  ENMO总和: {metrics['enmo_total']:.6f}")
print(f"  ENMO/样本: {metrics['enmo_total']/50:.6f}")
print(f"  MAD: {metrics['mad_intensity']:.6f}")

# 测试3: 慢走 (SVM ≈ 1.1-1.2g)
print("\n【测试3】慢走 (1秒 = 50样本)")
acc_walk = []
for i in range(50):
    # 模拟走路的加速度模式 (1.5Hz步频)
    x = 0.1 * np.sin(2 * np.pi * 1.5 * i / 50)
    y = 0.1 * np.cos(2 * np.pi * 1.5 * i / 50)
    z = 1.0 + 0.15 * np.sin(2 * np.pi * 1.5 * i / 50)
    acc_walk.append([x, y, z])
acc_walk = np.array(acc_walk)
metrics = processor.calculate_activity_metrics(acc_walk)
svm_avg = np.mean(np.sqrt(np.sum(acc_walk**2, axis=1)))
print(f"  SVM平均: {svm_avg:.3f}g")
print(f"  ENMO总和: {metrics['enmo_total']:.6f}")
print(f"  ENMO/样本: {metrics['enmo_total']/50:.6f}")
print(f"  MAD: {metrics['mad_intensity']:.6f}")

# 测试4: 快走/慢跑 (SVM ≈ 1.3-1.5g)
print("\n【测试4】快走/慢跑 (1秒 = 50样本)")
acc_jog = []
for i in range(50):
    # 模拟跑步的加速度模式 (2Hz步频)
    x = 0.2 * np.sin(2 * np.pi * 2.0 * i / 50)
    y = 0.2 * np.cos(2 * np.pi * 2.0 * i / 50)
    z = 1.0 + 0.4 * np.sin(2 * np.pi * 2.0 * i / 50)
    acc_jog.append([x, y, z])
acc_jog = np.array(acc_jog)
metrics = processor.calculate_activity_metrics(acc_jog)
svm_avg = np.mean(np.sqrt(np.sum(acc_jog**2, axis=1)))
print(f"  SVM平均: {svm_avg:.3f}g")
print(f"  ENMO总和: {metrics['enmo_total']:.6f}")
print(f"  ENMO/样本: {metrics['enmo_total']/50:.6f}")
print(f"  MAD: {metrics['mad_intensity']:.6f}")

# 测试5: 剧烈运动 (SVM ≈ 1.5-2.0g)
print("\n【测试5】剧烈运动/跳跃 (1秒 = 50样本)")
acc_vigorous = []
for i in range(50):
    # 模拟剧烈运动
    x = 0.4 * np.sin(2 * np.pi * 2.5 * i / 50)
    y = 0.4 * np.cos(2 * np.pi * 2.5 * i / 50)
    z = 1.0 + 0.8 * np.sin(2 * np.pi * 2.5 * i / 50)
    acc_vigorous.append([x, y, z])
acc_vigorous = np.array(acc_vigorous)
metrics = processor.calculate_activity_metrics(acc_vigorous)
svm_avg = np.mean(np.sqrt(np.sum(acc_vigorous**2, axis=1)))
print(f"  SVM平均: {svm_avg:.3f}g")
print(f"  ENMO总和: {metrics['enmo_total']:.6f}")
print(f"  ENMO/样本: {metrics['enmo_total']/50:.6f}")
print(f"  MAD: {metrics['mad_intensity']:.6f}")

print("\n" + "=" * 60)
print("结论:")
print("=" * 60)
print("1. Python代码返回的是ENMO总和（所有样本的和）")
print("2. 对于1秒50个样本:")
print("   - 静息: ENMO总和 ≈ 0")
print("   - 轻微: ENMO总和 ≈ 0-2")
print("   - 慢走: ENMO总和 ≈ 2-5")
print("   - 快走: ENMO总和 ≈ 5-15")
print("   - 剧烈: ENMO总和 ≈ 15-30")
print("\n3. 如果每日目标是100，合理的达成时间:")
print("   - 慢走: 100 / 3 = 33秒 (太快!)")
print("   - 快走: 100 / 10 = 10秒 (太快!)")
print("\n4. 问题: Python代码的ENMO值也很高!")
print("   建议: 应该除以采样率，得到平均ENMO/秒")
