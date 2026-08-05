"""
分析和可视化捕获的SAM特征
"""

import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import os
import glob

output_dir = '../../output'

# 查找所有.npy文件
npy_files = glob.glob(f'{output_dir}/feature_*.npy')

if not npy_files:
    print("❌ 没有找到特征文件，请先运行 run_with_direct_capture.py")
    exit(1)

print(f"找到 {len(npy_files)} 个特征文件\n")
print("="*80)

# 加载和分析每个特征
features_info = []

for npy_file in npy_files:
    feature = np.load(npy_file)
    filename = os.path.basename(npy_file)
    
    info = {
        'file': filename,
        'path': npy_file,
        'shape': feature.shape,
        'mean': feature.mean(),
        'std': feature.std(),
        'min': feature.min(),
        'max': feature.max(),
        'data': feature
    }
    
    features_info.append(info)
    
    B, C, H, W = feature.shape
    
    print(f"文件: {filename}")
    print(f"  形状: {feature.shape}")
    print(f"  解读: {B}个图像, {C}个通道, {H}x{W}空间分辨率")
    print(f"  统计: mean={info['mean']:.4f}, std={info['std']:.4f}")
    print(f"  范围: [{info['min']:.4f}, {info['max']:.4f}]")
    
    # 判断类型
    if B == 1:
        print(f"  ✓ 这是 BASE 特征 (全局图像)")
    elif B > 1:
        print(f"  ✓ 这是 PATCHES 特征 ({B}个局部裁剪)")
    
    if C == 1024:
        print(f"  ✓ 这是 SAM 原始输出 (1024通道)")
    
    print()

print("="*80)
print("\n创建可视化...")

# 为每个特征创建多通道可视化
fig_num = 1

for info in features_info:
    feature = info['data']
    B, C, H, W = feature.shape
    
    # 选择要可视化的通道（每隔128个通道取一个，最多16个）
    channel_indices = list(range(0, C, max(1, C // 16)))[:16]
    
    n_channels = len(channel_indices)
    n_cols = 4
    n_rows = (n_channels + n_cols - 1) // n_cols
    
    # 为每个batch创建一个图
    for b in range(B):
        fig, axes = plt.subplots(n_rows, n_cols, figsize=(15, 3.5*n_rows))
        fig.suptitle(f'{info["file"]}\nBatch {b}, Shape: {feature.shape}', fontsize=12)
        
        if n_rows * n_cols == 1:
            axes = [axes]
        else:
            axes = axes.flatten()
        
        for idx, ch_idx in enumerate(channel_indices):
            ax = axes[idx]
            
            # 获取特征图
            feat_map = feature[b, ch_idx, :, :]
            
            # 显示
            im = ax.imshow(feat_map, cmap='viridis', aspect='auto')
            ax.set_title(f'Channel {ch_idx}')
            ax.axis('off')
            
            # 添加colorbar
            plt.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
        
        # 隐藏多余的子图
        for idx in range(n_channels, len(axes)):
            axes[idx].axis('off')
        
        plt.tight_layout()
        
        # 保存
        save_path = f'{output_dir}/analysis_batch{b}_{info["file"]}.png'
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"✓ 保存可视化: {save_path}")
        plt.close()

print("\n" + "="*80)
print("创建特征统计对比图...")

# 对比不同特征的统计分布
fig, axes = plt.subplots(2, 2, figsize=(12, 10))

for idx, info in enumerate(features_info):
    color = f'C{idx}'
    label = f"Feature {idx}: {info['shape']}"
    
    # 展平特征用于统计
    feature_flat = info['data'].flatten()
    
    # 直方图
    axes[0, 0].hist(feature_flat, bins=100, alpha=0.6, label=label, color=color)
    
    # 累积分布
    axes[0, 1].hist(feature_flat, bins=100, alpha=0.6, cumulative=True, 
                    density=True, label=label, color=color)
    
    # Box plot数据
    axes[1, 0].boxplot([feature_flat], positions=[idx], widths=0.6,
                       patch_artist=True, 
                       boxprops=dict(facecolor=color, alpha=0.6))
    
    # 每个通道的均值
    channel_means = info['data'].mean(axis=(0, 2, 3))  # 对B,H,W求均值
    axes[1, 1].plot(channel_means, label=label, color=color, alpha=0.7)

axes[0, 0].set_title('Value Distribution')
axes[0, 0].set_xlabel('Feature Value')
axes[0, 0].set_ylabel('Frequency')
axes[0, 0].legend()
axes[0, 0].grid(True, alpha=0.3)

axes[0, 1].set_title('Cumulative Distribution')
axes[0, 1].set_xlabel('Feature Value')
axes[0, 1].set_ylabel('Cumulative Probability')
axes[0, 1].legend()
axes[0, 1].grid(True, alpha=0.3)

axes[1, 0].set_title('Value Range (Box Plot)')
axes[1, 0].set_ylabel('Feature Value')
axes[1, 0].set_xticks(range(len(features_info)))
axes[1, 0].set_xticklabels([f'Feature {i}' for i in range(len(features_info))])
axes[1, 0].grid(True, alpha=0.3, axis='y')

axes[1, 1].set_title('Mean Value per Channel')
axes[1, 1].set_xlabel('Channel Index')
axes[1, 1].set_ylabel('Mean Value')
axes[1, 1].legend()
axes[1, 1].grid(True, alpha=0.3)

plt.tight_layout()
stats_path = f'{output_dir}/feature_statistics_comparison.png'
plt.savefig(stats_path, dpi=150, bbox_inches='tight')
print(f"✓ 保存统计对比图: {stats_path}")
plt.close()

print("\n" + "="*80)
print("特征相似度分析...")

# 如果有多个特征，计算它们之间的相似度
if len(features_info) > 1:
    from scipy.spatial.distance import cosine
    
    print("\n特征向量的余弦相似度:")
    
    for i in range(len(features_info)):
        for j in range(i+1, len(features_info)):
            feat_i = features_info[i]['data'].flatten()
            feat_j = features_info[j]['data'].flatten()
            
            # 如果长度不同，取较短的长度
            min_len = min(len(feat_i), len(feat_j))
            feat_i = feat_i[:min_len]
            feat_j = feat_j[:min_len]
            
            # 计算余弦相似度
            similarity = 1 - cosine(feat_i, feat_j)
            
            print(f"  Feature {i} vs Feature {j}: {similarity:.4f}")

print("\n" + "="*80)
print("生成特征总结报告...")

# 创建文本报告
report = []
report.append("="*80)
report.append("DeepSeek-OCR SAM 特征分析报告")
report.append("="*80)
report.append("")

for idx, info in enumerate(features_info):
    B, C, H, W = info['shape']
    
    report.append(f"特征 {idx}: {info['file']}")
    report.append("-" * 40)
    report.append(f"  形状: {info['shape']}")
    report.append(f"  类型: {'BASE (全局)' if B == 1 else f'PATCHES ({B}个局部)'}")
    report.append(f"  空间分辨率: {H} x {W} = {H*W} 个特征点")
    report.append(f"  特征维度: {C}")
    report.append(f"  总参数量: {B * C * H * W:,}")
    report.append(f"  内存占用: {B * C * H * W * 4 / (1024**2):.2f} MB (float32)")
    report.append("")
    report.append(f"  统计信息:")
    report.append(f"    均值: {info['mean']:.6f}")
    report.append(f"    标准差: {info['std']:.6f}")
    report.append(f"    最小值: {info['min']:.6f}")
    report.append(f"    最大值: {info['max']:.6f}")
    report.append("")
    
    # 通道统计
    channel_means = info['data'].mean(axis=(0, 2, 3))
    report.append(f"  通道统计:")
    report.append(f"    激活最强的通道: {channel_means.argmax()} (均值={channel_means.max():.4f})")
    report.append(f"    激活最弱的通道: {channel_means.argmin()} (均值={channel_means.min():.4f})")
    report.append("")

report.append("="*80)
report.append("如何使用这些特征:")
report.append("="*80)
report.append("")
report.append("1. 加载特征:")
report.append("   import numpy as np")
report.append("   feature = np.load('feature_0_ImageEncoderViT_xxx.npy')")
report.append("")
report.append("2. 特征的物理意义:")
report.append("   - BASE: 全局图像的视觉表示，包含整体布局信息")
report.append("   - PATCHES: 局部区域的高分辨率特征，包含细节信息")
report.append("")
report.append("3. 可能的应用:")
report.append("   - 文档布局分析")
report.append("   - 文本区域检测")
report.append("   - 图像检索")
report.append("   - 特征可视化")
report.append("")

report_text = "\n".join(report)
print(report_text)

# 保存报告
report_path = f'{output_dir}/sam_features_report.txt'
with open(report_path, 'w', encoding='utf-8') as f:
    f.write(report_text)

print(f"\n✓ 报告已保存: {report_path}")

print("\n" + "="*80)
print("分析完成！生成的文件:")
print("="*80)
print(f"  - {output_dir}/feature_*.npy : 原始特征数据")
print(f"  - {output_dir}/analysis_*.png : 多通道可视化")
print(f"  - {output_dir}/feature_statistics_comparison.png : 统计对比")
print(f"  - {output_dir}/sam_features_report.txt : 详细报告")
print("="*80)




















