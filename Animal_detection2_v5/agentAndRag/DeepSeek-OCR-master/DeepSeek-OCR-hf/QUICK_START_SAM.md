# 🎯 SAM特征捕获快速指南

## ✅ 成功捕获SAM输出！

根据你的运行结果，已经成功捕获到SAM模型的输出：

### 捕获到的特征

1. **PATCHES特征** (局部裁剪)
   - 形状: `[6, 1024, 10, 10]`
   - 6个局部patch，每个1024通道，10x10空间分辨率

2. **BASE特征** (全局图像)
   - 形状: `[1, 1024, 16, 16]`
   - 1个全局视图，1024通道，16x16空间分辨率

这两个就是SAM模型的**原始输出**，在经过CLIP和Projector处理后变成：
- BASE: `[1, 256, 1280]` (reshape后)
- PATCHES: `[6, 100, 1280]` (reshape后)

---

## 🚀 使用步骤

### Step 1: 捕获SAM特征

修复BFloat16问题后，重新运行：

```bash
cd F:\BaiduNetdiskDownload\港城大\deepseek-ocr\DeepSeek-OCR\DeepSeek-OCR-master\DeepSeek-OCR-hf
python run_with_direct_capture.py
```

**预期输出**：
```
✓ 捕获到 2 个vision特征
保存特征:
[0] ../../output/feature_0_ImageEncoderViT_6x1024x10x10.npy
[1] ../../output/feature_1_ImageEncoderViT_1x1024x16x16.npy
```

### Step 2: 分析SAM特征

```bash
python analyze_sam_features.py
```

这会生成：
- 📊 多通道特征可视化
- 📈 统计对比图
- 📄 详细分析报告

---

## 📦 生成的文件说明

### 主要输出文件

```
output/
├── feature_0_ImageEncoderViT_6x1024x10x10.npy      # PATCHES SAM输出
├── feature_1_ImageEncoderViT_1x1024x16x16.npy      # BASE SAM输出
├── feature_0_ImageEncoderViT_vis.png                # 快速可视化
├── feature_1_ImageEncoderViT_vis.png                # 快速可视化
├── analysis_batch0_feature_0_*.png                  # 详细多通道分析
├── feature_statistics_comparison.png                # 统计对比
└── sam_features_report.txt                          # 文本报告
```

---

## 💻 使用SAM特征的代码示例

### 基础加载

```python
import numpy as np

# 加载PATCHES特征
patches = np.load('output/feature_0_ImageEncoderViT_6x1024x10x10.npy')
print(f"PATCHES形状: {patches.shape}")  # [6, 1024, 10, 10]

# 加载BASE特征
base = np.load('output/feature_1_ImageEncoderViT_1x1024x16x16.npy')
print(f"BASE形状: {base.shape}")  # [1, 1024, 16, 16]
```

### 可视化单个通道

```python
import matplotlib.pyplot as plt

# 可视化BASE的第一个通道
plt.figure(figsize=(10, 10))
plt.imshow(base[0, 0, :, :], cmap='viridis')
plt.colorbar()
plt.title('BASE Feature - Channel 0')
plt.savefig('base_channel_0.png')
```

### 分析空间激活模式

```python
import numpy as np
import matplotlib.pyplot as plt

# 计算每个空间位置的平均激活
base_spatial = base[0].mean(axis=0)  # 对1024个通道求平均 -> [16, 16]

plt.figure(figsize=(8, 8))
plt.imshow(base_spatial, cmap='hot', interpolation='nearest')
plt.colorbar(label='Mean Activation')
plt.title('BASE Spatial Activation Map')
plt.xlabel('Width')
plt.ylabel('Height')
plt.savefig('base_spatial_activation.png')
```

### 分析通道重要性

```python
# 计算每个通道的总激活强度
channel_importance = base[0].reshape(1024, -1).mean(axis=1)

# 找出最重要的10个通道
top_10_channels = np.argsort(channel_importance)[-10:][::-1]

print("最重要的10个通道:")
for i, ch in enumerate(top_10_channels):
    print(f"  {i+1}. Channel {ch}: {channel_importance[ch]:.4f}")

# 可视化
plt.figure(figsize=(12, 4))
plt.plot(channel_importance)
plt.scatter(top_10_channels, channel_importance[top_10_channels], 
           c='red', s=100, zorder=5)
plt.xlabel('Channel Index')
plt.ylabel('Mean Activation')
plt.title('Channel Importance')
plt.grid(True, alpha=0.3)
plt.savefig('channel_importance.png')
```

### 比较BASE和PATCHES

```python
# 计算BASE和第一个PATCH的特征相似度
from scipy.spatial.distance import cosine

base_vec = base[0].flatten()
patch1_vec = patches[0].flatten()

# 调整长度
min_len = min(len(base_vec), len(patch1_vec))
base_vec = base_vec[:min_len]
patch1_vec = patch1_vec[:min_len]

similarity = 1 - cosine(base_vec, patch1_vec)
print(f"BASE vs PATCH1 相似度: {similarity:.4f}")

# 计算所有PATCH之间的相似度矩阵
n_patches = patches.shape[0]
similarity_matrix = np.zeros((n_patches, n_patches))

for i in range(n_patches):
    for j in range(n_patches):
        vec_i = patches[i].flatten()
        vec_j = patches[j].flatten()
        similarity_matrix[i, j] = 1 - cosine(vec_i, vec_j)

plt.figure(figsize=(8, 6))
plt.imshow(similarity_matrix, cmap='coolwarm', vmin=0, vmax=1)
plt.colorbar(label='Cosine Similarity')
plt.title('Patch-to-Patch Similarity Matrix')
plt.xlabel('Patch Index')
plt.ylabel('Patch Index')
plt.savefig('patch_similarity.png')
```

### 特征聚合

```python
# 将所有PATCH特征聚合
patches_mean = patches.mean(axis=0)  # [1024, 10, 10]
patches_max = patches.max(axis=0)    # [1024, 10, 10]

# 可视化聚合后的空间模式
fig, axes = plt.subplots(1, 2, figsize=(12, 5))

axes[0].imshow(patches_mean.mean(axis=0), cmap='viridis')
axes[0].set_title('PATCHES Mean Pooling')
axes[0].axis('off')

axes[1].imshow(patches_max.mean(axis=0), cmap='viridis')
axes[1].set_title('PATCHES Max Pooling')
axes[1].axis('off')

plt.savefig('patches_pooling.png')
```

---

## 🔍 特征的物理意义

### BASE特征 `[1, 1024, 16, 16]`

- **用途**: 全局文档理解
- **包含信息**: 
  - 整体布局结构
  - 段落分布
  - 图文混排模式
- **分辨率**: 16x16（相对低分辨率，关注全局）

### PATCHES特征 `[6, 1024, 10, 10]`

- **用途**: 局部细节识别
- **包含信息**:
  - 文字细节
  - 边缘和笔画
  - 局部纹理
- **分辨率**: 10x10（每个patch的局部特征）
- **数量**: 6个不同的局部区域

---

## 🎨 高级应用示例

### 1. 文档区域检测

```python
import cv2

# 使用BASE特征的空间激活来检测重要区域
base_activation = base[0].mean(axis=0)  # [16, 16]

# 上采样到原始图像大小
from scipy.ndimage import zoom
upsampled = zoom(base_activation, (800/16, 600/16))

# 应用阈值
threshold = upsampled.mean() + upsampled.std()
mask = upsampled > threshold

# 可视化
plt.figure(figsize=(12, 6))

plt.subplot(1, 2, 1)
plt.imshow(upsampled, cmap='hot')
plt.title('Activation Heatmap')
plt.colorbar()

plt.subplot(1, 2, 2)
plt.imshow(mask, cmap='gray')
plt.title('Important Regions')

plt.savefig('region_detection.png')
```

### 2. 特征降维可视化 (t-SNE)

```python
from sklearn.manifold import TSNE

# 将所有特征展平
all_features = []
labels = []

# BASE
base_flat = base.reshape(1, -1)  # [1, 1024*16*16]
all_features.append(base_flat[0])
labels.append('BASE')

# PATCHES
for i in range(patches.shape[0]):
    patch_flat = patches[i].reshape(-1)  # [1024*10*10]
    all_features.append(patch_flat)
    labels.append(f'PATCH-{i}')

all_features = np.array(all_features)

# t-SNE降维到2D
tsne = TSNE(n_components=2, random_state=42)
features_2d = tsne.fit_transform(all_features)

# 可视化
plt.figure(figsize=(10, 8))
for i, label in enumerate(labels):
    color = 'red' if label == 'BASE' else 'blue'
    marker = 'o' if label == 'BASE' else '^'
    plt.scatter(features_2d[i, 0], features_2d[i, 1], 
               c=color, marker=marker, s=200, alpha=0.7, label=label)

plt.legend()
plt.title('SAM Features in 2D Space (t-SNE)')
plt.xlabel('Dimension 1')
plt.ylabel('Dimension 2')
plt.grid(True, alpha=0.3)
plt.savefig('features_tsne.png')
```

### 3. 创建自定义特征图

```python
def create_feature_map(feature, method='mean'):
    """
    从SAM特征创建单通道特征图
    
    Args:
        feature: [B, C, H, W]
        method: 'mean', 'max', 'std', 'pca'
    
    Returns:
        feature_map: [H, W]
    """
    if method == 'mean':
        return feature.mean(axis=(0, 1))  # 对B和C求平均
    elif method == 'max':
        return feature.max(axis=(0, 1))
    elif method == 'std':
        return feature.std(axis=(0, 1))
    elif method == 'pca':
        from sklearn.decomposition import PCA
        B, C, H, W = feature.shape
        flat = feature.reshape(B*C, H*W).T  # [H*W, B*C]
        pca = PCA(n_components=1)
        result = pca.fit_transform(flat)  # [H*W, 1]
        return result.reshape(H, W)

# 使用
base_mean = create_feature_map(base, 'mean')
base_max = create_feature_map(base, 'max')
base_std = create_feature_map(base, 'std')

fig, axes = plt.subplots(1, 3, figsize=(15, 4))

axes[0].imshow(base_mean, cmap='viridis')
axes[0].set_title('Mean')

axes[1].imshow(base_max, cmap='viridis')
axes[1].set_title('Max')

axes[2].imshow(base_std, cmap='viridis')
axes[2].set_title('Std Dev')

plt.savefig('feature_maps_comparison.png')
```

---

## 🐛 故障排查

### 问题：BFloat16错误

**解决**: 已在修复版脚本中处理，会自动转换为Float32

### 问题：内存不足

**解决**: 只保存你需要的特征

```python
# 在捕获脚本中添加过滤
if output.shape[1] == 1024:  # 只保存1024通道的
    sam_outputs.append(output)
```

### 问题：特征太大

**解决**: 保存降采样版本

```python
# 保存时降采样
tensor_downsampled = tensor[:, ::4, :, :]  # 只保存每4个通道
np.save(file, tensor_downsampled.numpy())
```

---

## 📚 参考资料

1. **SAM论文**: "Segment Anything" - Meta AI Research
2. **DeepSeek-OCR**: 结合SAM+CLIP的多模态OCR
3. **特征可视化**: 使用t-SNE, PCA等降维技术

---

## ✨ 总结

你现在可以：

1. ✅ 捕获SAM模型的原始输出
2. ✅ 理解BASE和PATCHES的区别
3. ✅ 分析和可视化特征
4. ✅ 使用特征进行下游任务

**下一步**: 根据你的具体需求，使用这些特征进行：
- 文档布局分析
- 区域检测
- 图像检索
- 特征学习

祝实验顺利！🎉




















