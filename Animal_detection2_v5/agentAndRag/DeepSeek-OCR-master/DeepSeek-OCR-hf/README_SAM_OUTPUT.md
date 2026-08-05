# DeepSeek-OCR 工作流程详解与SAM输出截获指南

## 📋 目录
1. [整体架构](#整体架构)
2. [为什么能自动生成结果](#为什么能自动生成结果)
3. [SAM模型的作用](#sam模型的作用)
4. [如何截获SAM输出](#如何截获sam输出)
5. [实际操作步骤](#实际操作步骤)

---

## 🏗️ 整体架构

DeepSeek-OCR 是一个多模态文档理解模型，其架构包含三个核心组件：

```
输入图像
    ↓
┌─────────────────────────────────────┐
│  1. SAM 模型 (Segment Anything)     │
│     - 提取低层视觉特征              │
│     - 输出: [B, 1024, H', W']       │
└─────────────────────────────────────┘
    ↓
┌─────────────────────────────────────┐
│  2. CLIP 模型                       │
│     - 提取高层语义特征              │
│     - 与SAM特征结合                 │
└─────────────────────────────────────┘
    ↓
┌─────────────────────────────────────┐
│  3. Projector + Language Model      │
│     - 将视觉特征投影到文本空间      │
│     - 生成OCR文本和位置信息         │
└─────────────────────────────────────┘
    ↓
输出: Markdown文本 + 位置标注
```

---

## 🎯 为什么能自动生成结果图片和Markdown文档？

### 1. **模型的 `infer()` 方法**

当你调用 `model.infer()` 时，这个方法内部执行以下步骤：

```python
res = model.infer(
    tokenizer, 
    prompt=prompt,           # "<image>\n<|grounding|>Convert the document to markdown."
    image_file=image_file,   # 输入图像路径
    output_path=output_path, # 输出目录
    base_size=1024,          # 全局视图的基础大小
    image_size=640,          # 局部裁剪的图像大小
    crop_mode=True,          # 是否使用裁剪模式
    save_results=True,       # ✨ 关键参数：自动保存结果
    test_compress=True
)
```

**关键：`save_results=True`** 参数会触发以下操作：

#### a) 生成带位置标注的输出

模型输出格式包含特殊token：
```markdown
<ref>text content<box>[[x1,y1,x2,y2]]</box></ref>
```

例如：
```markdown
<ref>Sample Document<box>[[100,50,500,100]]</box></ref>
<ref>This is a paragraph.<box>[[100,120,500,200]]</box></ref>
```

#### b) 后处理流程

`infer()` 方法内部会：

1. **提取位置信息**：解析 `<ref>` 和 `<box>` 标签
2. **绘制边界框**：在原图上绘制检测到的文本区域
3. **生成Markdown**：移除位置标注，保留纯文本
4. **保存文件**：
   - `result.mmd`: 清理后的Markdown文档
   - `result_with_boxes.jpg`: 带边界框的可视化图像

### 2. **代码实现位置**

`infer()` 方法的实现在 **HuggingFace Hub** 的模型仓库中，因为：

```python
model = AutoModel.from_pretrained(
    model_name, 
    trust_remote_code=True,  # ✨ 允许执行远程代码
    use_safetensors=True
)
```

`trust_remote_code=True` 会从 HuggingFace 下载并加载模型的自定义代码，包括 `infer()` 方法。

### 3. **参考vLLM版本的实现**

从项目中的 vLLM 版本可以看到类似的后处理逻辑：

```python
# DeepSeek-OCR-vllm/run_dpsk_ocr_image.py
def draw_bounding_boxes(image, refs):
    """绘制边界框"""
    for ref in refs:
        # 解析 <ref> 和 <box> 标签
        label_type, points_list = extract_coordinates_and_label(ref)
        # 在图像上绘制框
        draw.rectangle([x1, y1, x2, y2], outline=color)
    return image

# 保存结果
result_image.save(f'{OUTPUT_PATH}/result_with_boxes.jpg')
with open(f'{OUTPUT_PATH}/result.mmd', 'w') as f:
    f.write(cleaned_markdown)
```

---

## 🔍 SAM模型的作用

### SAM (Segment Anything Model)

**SAM 是一个强大的视觉编码器，专门用于提取图像的底层特征。**

### 在 DeepSeek-OCR 中的角色：

1. **特征提取器**
   ```python
   # 处理全局图像
   global_features_1 = self.sam_model(image_ori)
   # 输出: [batch, 1024, H, W]
   
   # 处理局部裁剪patches
   local_features_1 = self.sam_model(patches)
   # 输出: [batch, 1024, H', W']
   ```

2. **与CLIP结合**
   ```python
   # SAM提供低层特征
   sam_features = self.sam_model(image)
   
   # CLIP处理高层语义，接收SAM特征作为输入
   clip_features = self.vision_model(image, sam_features)
   
   # 拼接两种特征
   combined_features = torch.cat([clip_features[:, 1:], 
                                  sam_features.flatten(2).permute(0, 2, 1)], 
                                  dim=-1)
   
   # 投影到语言模型空间
   final_features = self.projector(combined_features)
   ```

3. **多尺度处理**
   - **Global View**: 处理整个图像的缩放版本
   - **Local View**: 处理高分辨率的局部裁剪块
   - 两者结合提供更丰富的视觉信息

### SAM的输出结构：

```python
# SAM模型架构
class ImageEncoderViT:
    def forward(self, x):
        x = self.patch_embed(x)        # [B, H*W, 768]
        for block in self.blocks:
            x = block(x)                # Transformer blocks
        
        x = self.neck(x)                # [B, 256, H, W]
        x = self.net_2(x)               # [B, 512, H/2, W/2]
        x = self.net_3(x)               # [B, 1024, H/4, W/4]
        return x
```

**输出特征图**：
- 形状: `[batch_size, 1024, H/4, W/4]`
- 每个空间位置的1024维特征向量
- 包含丰富的视觉语义信息

---

## 🎣 如何截获SAM输出

### 方法1: 使用 PyTorch Hook（推荐）

PyTorch的Hook机制允许你在不修改模型代码的情况下拦截中间层的输出。

#### 完整代码示例：

```python
from transformers import AutoModel, AutoTokenizer
import torch
import numpy as np

# 加载模型
model = AutoModel.from_pretrained(
    'deepseek-ai/DeepSeek-OCR', 
    trust_remote_code=True
).eval().cuda()

# 存储SAM输出的容器
sam_outputs = []

def sam_hook(module, input, output):
    """
    Hook函数：每次SAM模型前向传播时调用
    
    参数:
        module: 被hook的模块（SAM模型）
        input: 输入到模块的数据
        output: 模块的输出
    """
    # 保存输出（注意detach避免影响梯度）
    sam_outputs.append(output.detach().cpu())
    
    # 打印信息
    print(f"SAM Output Shape: {output.shape}")
    print(f"SAM Output Range: [{output.min():.3f}, {output.max():.3f}]")
    
    return output

# 注册hook
# 需要找到模型中SAM的确切路径
if hasattr(model, 'vision_model') and hasattr(model.vision_model, 'sam_model'):
    handle = model.vision_model.sam_model.register_forward_hook(sam_hook)
    print("✓ Hook registered successfully")

# 运行推理
res = model.infer(tokenizer, prompt=prompt, image_file=image_file, ...)

# 保存SAM输出
for idx, sam_out in enumerate(sam_outputs):
    np.save(f'sam_output_{idx}.npy', sam_out.numpy())
    print(f"Saved: sam_output_{idx}.npy, shape={sam_out.shape}")

# 清理hook
handle.remove()
```

### 方法2: 修改模型源码

如果Hook方法不work，你需要修改HuggingFace下载的模型代码。

#### 步骤：

1. **找到缓存的模型代码**：
   ```bash
   # HuggingFace缓存通常在：
   # Windows: C:\Users\<username>\.cache\huggingface\modules\transformers_modules\
   # Linux: ~/.cache/huggingface/modules/transformers_modules/
   ```

2. **定位SAM调用位置**：
   在模型文件中搜索 `sam_model` 或类似的调用

3. **添加保存代码**：
   ```python
   # 在 _pixel_values_to_embedding 或类似方法中
   def forward(self, ...):
       # ...
       sam_output = self.sam_model(image)
       
       # 添加保存逻辑
       torch.save(sam_output, 'sam_output.pt')
       np.save('sam_output.npy', sam_output.cpu().numpy())
       
       # ...继续原有逻辑
   ```

### 方法3: 创建包装类

```python
class SAMWrapper(nn.Module):
    def __init__(self, sam_model, save_dir='./sam_outputs'):
        super().__init__()
        self.sam_model = sam_model
        self.save_dir = save_dir
        self.counter = 0
        os.makedirs(save_dir, exist_ok=True)
    
    def forward(self, x):
        output = self.sam_model(x)
        
        # 保存输出
        save_path = f"{self.save_dir}/sam_{self.counter}.npy"
        np.save(save_path, output.detach().cpu().numpy())
        self.counter += 1
        
        return output

# 替换原始SAM模型
model.vision_model.sam_model = SAMWrapper(model.vision_model.sam_model)
```

---

## 🚀 实际操作步骤

### Step 1: 使用提供的脚本

我已经为你创建了 `run_dpsk_ocr_with_sam_output.py`，直接运行：

```bash
cd F:\BaiduNetdiskDownload\港城大\deepseek-ocr\DeepSeek-OCR\DeepSeek-OCR-master\DeepSeek-OCR-hf
conda activate deepseek-ocr
python run_dpsk_ocr_with_sam_output.py
```

### Step 2: 检查输出

脚本会生成：
- `sam_output_*.npy`: SAM模型的原始输出
- `sam_feature_vis_*.png`: SAM特征的可视化（第一个通道）
- `result.mmd`: OCR结果的Markdown文档
- `result_with_boxes.jpg`: 带边界框的图像

### Step 3: 分析SAM输出

```python
import numpy as np
import matplotlib.pyplot as plt

# 加载SAM输出
sam_out = np.load('sam_output_0.npy')
print(f"Shape: {sam_out.shape}")  # [batch, 1024, H, W]

# 可视化不同通道
fig, axes = plt.subplots(2, 4, figsize=(15, 8))
for i in range(8):
    ax = axes[i//4, i%4]
    feature_map = sam_out[0, i*128, :, :]  # 每隔128个通道取一个
    ax.imshow(feature_map, cmap='viridis')
    ax.set_title(f'Channel {i*128}')
    ax.axis('off')
plt.tight_layout()
plt.savefig('sam_features_visualization.png')
```

### Step 4: 如果Hook不工作

可能需要调试模型结构：

```python
# 打印模型结构
def print_model_structure(model, prefix=''):
    for name, child in model.named_children():
        print(f"{prefix}{name}: {type(child).__name__}")
        print_model_structure(child, prefix + '  ')

print_model_structure(model)
```

找到SAM模块的确切路径后，修改hook注册代码。

---

## 📊 预期输出示例

### 终端输出：
```
✓ Successfully registered hook to sam_model
============================================================
开始推理...
============================================================
[SAM Output] Shape: torch.Size([1, 1024, 64, 64])
[SAM Output] Min: -2.3456, Max: 3.7890, Mean: 0.1234
[SAM Output] Shape: torch.Size([4, 1024, 64, 64])
[SAM Output] Min: -1.9876, Max: 4.2345, Mean: 0.2345
============================================================
推理完成！
============================================================
捕获到 2 个SAM输出
保存 SAM 输出 0: shape=(1, 1024, 64, 64) -> ../../output/sam_output_0.npy
保存 SAM 特征可视化 0 -> ../../output/sam_feature_vis_0.png
```

### 文件输出：
```
output/
├── result.mmd                    # OCR结果Markdown
├── result_with_boxes.jpg         # 带边界框的图像
├── sam_output_0.npy              # SAM输出1
├── sam_output_1.npy              # SAM输出2
├── sam_feature_vis_0.png         # 可视化1
└── sam_feature_vis_1.png         # 可视化2
```

---

## 🔧 故障排查

### 问题1: "Could not find sam_model"

**原因**: 模型结构与预期不同

**解决**:
```python
# 方法1: 打印所有属性
print("Model attributes:", dir(model))
for attr in dir(model):
    if 'sam' in attr.lower() or 'vision' in attr.lower():
        print(f"  Found: {attr}")

# 方法2: 递归搜索
def find_module(model, target_class_name):
    for name, module in model.named_modules():
        if target_class_name in str(type(module)):
            print(f"Found {target_class_name} at: {name}")
            return name, module
    return None, None

path, sam_module = find_module(model, 'ImageEncoderViT')
```

### 问题2: Hook没有被调用

**原因**: 可能SAM模型在模型外部被调用，或者使用了编译优化

**解决**:
- 尝试在更上层注册hook
- 检查是否使用了 `torch.compile()` 等优化

### 问题3: 内存溢出

**原因**: SAM输出很大（1024个通道）

**解决**:
```python
# 只保存部分通道
def sam_hook(module, input, output):
    # 只保存前64个通道
    reduced_output = output[:, :64, :, :].detach().cpu()
    sam_outputs.append(reduced_output)
```

---

## 📚 参考资料

1. **DeepSeek-OCR 论文**: 详细算法原理
2. **SAM (Segment Anything)**: https://segment-anything.com/
3. **PyTorch Hooks文档**: https://pytorch.org/docs/stable/generated/torch.nn.Module.html#torch.nn.Module.register_forward_hook
4. **项目中的vLLM实现**: `DeepSeek-OCR-vllm/deepseek_ocr.py` (第394, 404行)

---

## ✅ 总结

1. **为什么能生成结果**: `model.infer()` 内部实现了完整的后处理流程
2. **SAM的作用**: 提取低层视觉特征，与CLIP结合形成多尺度表示
3. **如何截获**: 使用PyTorch Hook机制最简单，无需修改代码
4. **实际应用**: 使用提供的脚本 `run_dpsk_ocr_with_sam_output.py`

祝你实验顺利！🎉




















