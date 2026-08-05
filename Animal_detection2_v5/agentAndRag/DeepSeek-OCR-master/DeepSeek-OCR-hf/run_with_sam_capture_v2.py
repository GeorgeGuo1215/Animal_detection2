"""
DeepSeek-OCR - 使用Monkey Patching技术截获SAM输出
这个方法不需要知道SAM模型的确切路径
"""

from transformers import AutoModel, AutoTokenizer
import torch
import os
import numpy as np
from PIL import Image

os.environ["CUDA_VISIBLE_DEVICES"] = '0'

# ============ 全局存储 ============
sam_outputs = {
    'captures': [],
    'count': 0
}

# ============ Monkey Patch技术 ============
print("正在设置Monkey Patch...")

# 保存原始的forward方法
original_forwards = {}

def create_wrapper(original_forward, module_name):
    """创建一个wrapper函数来包装原始的forward"""
    def wrapped_forward(self, x, *args, **kwargs):
        # 调用原始forward
        output = original_forward(x, *args, **kwargs)
        
        # 检查输出是否像SAM的输出（通常是4D tensor: [B, C, H, W]）
        if isinstance(output, torch.Tensor) and output.ndim == 4:
            # 检查通道数是否在合理范围（SAM输出通常是256-1024通道）
            if 200 <= output.shape[1] <= 2048:
                sam_outputs['count'] += 1
                print(f"\n[捕获 #{sam_outputs['count']}] 从 {module_name}")
                print(f"  形状: {output.shape}")
                print(f"  数据类型: {output.dtype}")
                print(f"  范围: [{output.min().item():.4f}, {output.max().item():.4f}]")
                print(f"  均值: {output.mean().item():.4f}")
                
                # 保存输出
                sam_outputs['captures'].append({
                    'tensor': output.detach().cpu(),
                    'module': module_name,
                    'shape': tuple(output.shape)
                })
        
        return output
    
    return wrapped_forward

def patch_all_modules(model, prefix=''):
    """递归地给所有可能的模块打补丁"""
    for name, module in model.named_children():
        full_name = f"{prefix}.{name}" if prefix else name
        
        # 检查是否是可能的vision encoder
        class_name = type(module).__name__
        
        # 如果模块名或类名包含关键词，打补丁
        keywords = ['ImageEncoder', 'Encoder', 'SAM', 'Vision', 'Embed']
        is_target = any(kw in class_name for kw in keywords)
        
        if is_target and hasattr(module, 'forward'):
            try:
                # 保存原始forward
                original_forwards[full_name] = module.forward
                # 替换为wrapper
                module.forward = create_wrapper(module.forward, full_name).__get__(module, type(module))
                print(f"  ✓ 已打补丁: {full_name} ({class_name})")
            except Exception as e:
                print(f"  ✗ 打补丁失败: {full_name} - {e}")
        
        # 递归处理子模块
        patch_all_modules(module, full_name)

# ============ 加载模型 ============
model_name = 'deepseek-ai/DeepSeek-OCR'

print("\n加载模型...")
tokenizer = AutoTokenizer.from_pretrained(model_name, trust_remote_code=True)
model = AutoModel.from_pretrained(model_name, trust_remote_code=True, use_safetensors=True)
model = model.eval().cuda().to(torch.bfloat16)

print("\n" + "="*80)
print("应用Monkey Patch到所有可能的模块")
print("="*80)
patch_all_modules(model)

# ============ 运行推理 ============
prompt = "<image>\n<|grounding|>Convert the document to markdown. "
image_file = '../../test_images/sample.jpg'
output_path = '../../output'

print("\n" + "="*80)
print("开始推理...")
print("="*80)

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

print("\n" + "="*80)
print("推理完成！")
print("="*80)

# ============ 保存捕获的输出 ============
if sam_outputs['captures']:
    print(f"\n✓ 成功捕获 {len(sam_outputs['captures'])} 个输出")
    print("\n保存文件:")
    
    os.makedirs(output_path, exist_ok=True)
    
    for idx, capture in enumerate(sam_outputs['captures']):
        tensor = capture['tensor']
        module_name = capture['module']
        
        # 清理模块名用作文件名
        safe_name = module_name.replace('.', '_')
        
        # 保存numpy数组
        npy_file = f'{output_path}/capture_{idx}_{safe_name}.npy'
        np.save(npy_file, tensor.numpy())
        print(f"  [{idx}] {npy_file}")
        print(f"      来源: {module_name}")
        print(f"      形状: {capture['shape']}")
        
        # 可视化第一个通道
        if tensor.shape[1] > 0:
            feature_map = tensor[0, 0, :, :].numpy()
            feature_norm = ((feature_map - feature_map.min()) / 
                          (feature_map.max() - feature_map.min() + 1e-8) * 255).astype(np.uint8)
            
            img = Image.fromarray(feature_norm)
            img_file = f'{output_path}/capture_{idx}_{safe_name}_vis.png'
            img.save(img_file)
            print(f"      可视化: {img_file}")
        
        print()
    
    # 额外分析：识别哪个最可能是SAM
    print("\n" + "="*80)
    print("分析捕获的输出")
    print("="*80)
    
    for idx, capture in enumerate(sam_outputs['captures']):
        shape = capture['shape']
        module = capture['module']
        
        # SAM输出的特征：
        # 1. 通道数通常是1024
        # 2. 空间维度较小（相对于输入）
        
        is_likely_sam = False
        reasons = []
        
        if shape[1] == 1024:
            is_likely_sam = True
            reasons.append("通道数=1024 (典型SAM输出)")
        
        if 16 <= shape[2] <= 128 and 16 <= shape[3] <= 128:
            is_likely_sam = True
            reasons.append(f"空间维度{shape[2]}x{shape[3]}合理")
        
        print(f"[{idx}] {module}")
        print(f"     形状: {shape}")
        print(f"     可能是SAM? {'✓ 是' if is_likely_sam else '✗ 否'}")
        if reasons:
            print(f"     原因: {', '.join(reasons)}")
        print()

else:
    print("\n⚠ 警告：没有捕获到任何输出")
    print("可能原因:")
    print("  1. 模型结构完全不同于预期")
    print("  2. 输出不是标准的4D tensor")
    print("  3. 需要运行 debug_model_structure.py 来详细分析")

print(f"\n输出结果: {res}")
print(f"结果已保存到: {output_path}")




















