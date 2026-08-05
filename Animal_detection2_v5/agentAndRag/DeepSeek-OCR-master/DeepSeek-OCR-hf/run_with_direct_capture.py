"""
DeepSeek-OCR - 直接从打印位置截获特征
根据输出中的 "BASE:" 和 "PATCHES:" 定位
"""

from transformers import AutoModel, AutoTokenizer
import torch
import os
import numpy as np
from PIL import Image
import sys
from io import StringIO

os.environ["CUDA_VISIBLE_DEVICES"] = '0'

# ============ 全局存储 ============
features_captured = {
    'base_features': [],
    'patch_features': [],
    'sam_raw_outputs': []
}

# ============ 拦截print输出 ============
class PrintCapture:
    """捕获print输出并解析特征信息"""
    def __init__(self, original_stdout):
        self.original_stdout = original_stdout
        self.buffer = StringIO()
    
    def write(self, text):
        # 写到原始输出
        self.original_stdout.write(text)
        self.original_stdout.flush()
        
        # 同时写到缓冲区用于分析
        self.buffer.write(text)
    
    def flush(self):
        self.original_stdout.flush()

# ============ Monkey Patch torch.nn.Module ============
print("设置功能更强的捕获机制...")

# 全局计数器
call_counter = {'forward': 0}

# 保存所有4D tensor输出
all_4d_tensors = []

# 原始的__call__方法
original_module_call = torch.nn.Module.__call__

def patched_module_call(self, *args, **kwargs):
    """拦截所有Module的调用"""
    # 调用原始方法
    output = original_module_call(self, *args, **kwargs)
    
    # 检查输出
    if isinstance(output, torch.Tensor):
        if output.ndim == 4:  # [B, C, H, W]
            # 可能是vision feature
            C = output.shape[1]
            
            # SAM的特征通道数通常是1024
            # 投影后的特征通道数可能是1280（如你的输出所示）
            if C >= 256:
                class_name = type(self).__name__
                
                # 只记录vision相关的模块
                vision_keywords = ['Encoder', 'Vision', 'SAM', 'Image', 'Embed', 'Projector']
                if any(kw in class_name for kw in vision_keywords):
                    call_counter['forward'] += 1
                    
                    info = {
                        'id': call_counter['forward'],
                        'module_class': class_name,
                        'shape': tuple(output.shape),
                        'tensor': output.detach().cpu(),
                        'stats': {
                            'min': output.min().item(),
                            'max': output.max().item(),
                            'mean': output.mean().item()
                        }
                    }
                    
                    all_4d_tensors.append(info)
                    
                    print(f"\n[特征捕获 #{call_counter['forward']}]")
                    print(f"  模块: {class_name}")
                    print(f"  形状: {output.shape}")
                    print(f"  统计: min={info['stats']['min']:.4f}, "
                          f"max={info['stats']['max']:.4f}, mean={info['stats']['mean']:.4f}")
    
    return output

# 应用patch
torch.nn.Module.__call__ = patched_module_call
print("✓ 已拦截所有torch.nn.Module的forward调用")

# ============ 加载模型 ============
model_name = 'deepseek-ai/DeepSeek-OCR'

print("\n加载模型...")
tokenizer = AutoTokenizer.from_pretrained(model_name, trust_remote_code=True)
model = AutoModel.from_pretrained(model_name, trust_remote_code=True, use_safetensors=True)
model = model.eval().cuda().to(torch.bfloat16)

# ============ 运行推理 ============
prompt = "<image>\n<|grounding|>Convert the document to markdown. "
image_file = '../../test_images/sample.jpg'
output_path = '../../output'

print("\n" + "="*80)
print("开始推理（将捕获所有vision特征）...")
print("="*80)

# 捕获print输出
old_stdout = sys.stdout
capture = PrintCapture(old_stdout)
sys.stdout = capture

try:
    res = model.infer(
        tokenizer, 
        prompt=prompt, 
        image_file=image_file, 
        output_path=output_path, 
        base_size=1024, 
        image_size=640, 
        crop_mode=True, 
        save_results=True, 
        test_compress=True
    )
finally:
    # 恢复stdout
    sys.stdout = old_stdout

print("\n" + "="*80)
print("推理完成！")
print("="*80)

# ============ 分析捕获的输出 ============
print(f"\n✓ 捕获到 {len(all_4d_tensors)} 个vision特征")

if all_4d_tensors:
    print("\n" + "="*80)
    print("特征详情")
    print("="*80)
    
    os.makedirs(output_path, exist_ok=True)
    
    # 按形状分组
    shape_groups = {}
    for info in all_4d_tensors:
        shape = info['shape']
        if shape not in shape_groups:
            shape_groups[shape] = []
        shape_groups[shape].append(info)
    
    print(f"\n发现 {len(shape_groups)} 种不同的特征形状:")
    for shape, items in shape_groups.items():
        print(f"  {shape}: {len(items)} 个")
    
    # 保存所有捕获的特征
    print("\n保存特征:")
    for idx, info in enumerate(all_4d_tensors):
        tensor = info['tensor']
        shape = info['shape']
        module = info['module_class']
        
        # 保存numpy (转换为float32以支持numpy)
        npy_file = f'{output_path}/feature_{idx}_{module}_{"x".join(map(str, shape))}.npy'
        tensor_np = tensor.float().numpy()  # BFloat16 -> Float32
        np.save(npy_file, tensor_np)
        
        print(f"\n[{idx}] {npy_file}")
        print(f"    模块: {module}")
        print(f"    形状: {shape}")
        print(f"    统计: min={info['stats']['min']:.3f}, "
              f"max={info['stats']['max']:.3f}, mean={info['stats']['mean']:.3f}")
        
        # 分析这是什么类型的特征
        B, C, H, W = shape
        
        feature_type = "未知"
        if C == 1024:
            feature_type = "可能是SAM原始输出"
        elif C == 1280:
            feature_type = "可能是投影后的特征（BASE/PATCHES）"
        elif C >= 2000:
            feature_type = "可能是拼接后的特征"
        elif C == 256:
            feature_type = "可能是SAM的neck输出"
        
        print(f"    类型: {feature_type}")
        
        # 可视化
        try:
            # 取第一个batch，前3个通道（或第1个通道重复3次）
            if C >= 3:
                # RGB可视化
                vis_data = tensor[0, :3, :, :].float().numpy()
                # 归一化到0-1
                for c in range(3):
                    channel = vis_data[c]
                    vis_data[c] = (channel - channel.min()) / (channel.max() - channel.min() + 1e-8)
                
                # 转换为HWC格式
                vis_data = np.transpose(vis_data, (1, 2, 0))
                vis_data = (vis_data * 255).astype(np.uint8)
            else:
                # 灰度可视化
                vis_data = tensor[0, 0, :, :].float().numpy()
                vis_data = ((vis_data - vis_data.min()) / 
                          (vis_data.max() - vis_data.min() + 1e-8) * 255).astype(np.uint8)
            
            img = Image.fromarray(vis_data)
            img_file = f'{output_path}/feature_{idx}_{module}_vis.png'
            img.save(img_file)
            print(f"    可视化: {img_file}")
        except Exception as e:
            print(f"    可视化失败: {e}")
    
    # 特别标注BASE和PATCHES
    print("\n" + "="*80)
    print("识别BASE和PATCHES特征")
    print("="*80)
    
    # BASE特征特点: [1, *, H, W] - batch size = 1
    # PATCHES特征特点: [N, *, H, W] - batch size > 1
    
    for idx, info in enumerate(all_4d_tensors):
        B, C, H, W = info['shape']
        
        if B == 1 and C >= 1000:
            print(f"[{idx}] 可能是 BASE 特征:")
            print(f"     形状: {info['shape']}")
            print(f"     (对应输出: BASE: torch.Size([1, {H*W}, {C}]))")
        
        elif B > 1 and C >= 1000:
            print(f"[{idx}] 可能是 PATCHES 特征:")
            print(f"     形状: {info['shape']}")
            print(f"     (对应输出: PATCHES: torch.Size([{B}, {H*W}, {C}]))")

else:
    print("\n⚠ 没有捕获到vision特征")
    print("这很奇怪，因为模型明显在运行")

# 恢复原始的Module.__call__
torch.nn.Module.__call__ = original_module_call

print(f"\n输出结果: {res}")
print(f"结果已保存到: {output_path}")
print("\n提示: 你可以使用以下代码分析特征:")
print("""
import numpy as np
feature = np.load('output/feature_X_ModuleName_shape.npy')
print(f"形状: {feature.shape}")
print(f"均值: {feature.mean()}, 标准差: {feature.std()}")
""")

