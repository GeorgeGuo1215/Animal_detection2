"""
DeepSeek-OCR 运行脚本 - 带SAM模型输出截获功能

这个脚本展示了如何截获SAM模型的中间输出
"""

from transformers import AutoModel, AutoTokenizer
import torch
import os
import numpy as np
from PIL import Image


os.environ["CUDA_VISIBLE_DEVICES"] = '0'


model_name = 'deepseek-ai/DeepSeek-OCR'


tokenizer = AutoTokenizer.from_pretrained(model_name, trust_remote_code=True)
model = AutoModel.from_pretrained(model_name, trust_remote_code=True, use_safetensors=True)
model = model.eval().cuda().to(torch.bfloat16)


# ========== 关键部分：Hook函数来截获SAM模型的输出 ==========

# 用于保存SAM模型输出的全局变量
sam_outputs = {
    'global_features': [],
    'local_features': [],
}

def sam_hook(module, input, output):
    """
    Hook函数：用于截获SAM模型的输出
    
    SAM模型输出形状：[batch, 1024, H, W]
    其中H和W取决于输入图像大小和下采样率
    """
    # 将输出保存到全局变量
    # 注意：这里的output是一个tensor
    output_np = output.detach().cpu().numpy()
    
    # 保存完整的特征图
    if len(sam_outputs['global_features']) < 10:  # 限制保存数量，避免内存溢出
        sam_outputs['local_features'].append(output_np)
    
    print(f"\n[SAM Output] Shape: {output.shape}, dtype: {output.dtype}")
    print(f"[SAM Output] Min: {output.min().item():.4f}, Max: {output.max().item():.4f}, Mean: {output.mean().item():.4f}")
    
    return output


# 注册hook到SAM模型
# 注意：需要找到模型中SAM模块的路径
# 在DeepSeek-OCR中，SAM模型通常在 model.vision_model.sam_model 或类似路径
try:
    # 尝试访问SAM模型并注册hook
    if hasattr(model, 'vision_model'):
        if hasattr(model.vision_model, 'sam_model'):
            handle = model.vision_model.sam_model.register_forward_hook(sam_hook)
            print("✓ Successfully registered hook to sam_model")
        else:
            print("⚠ Warning: Could not find sam_model in vision_model")
    elif hasattr(model, 'sam_model'):
        handle = model.sam_model.register_forward_hook(sam_hook)
        print("✓ Successfully registered hook to sam_model")
    else:
        print("⚠ Warning: Could not find sam_model in model")
        print("Model attributes:", dir(model))
except Exception as e:
    print(f"⚠ Error registering hook: {e}")


# prompt = "<image>\nFree OCR. "
prompt = "<image>\n<|grounding|>Convert the document to markdown. "
image_file = '../../test_images/sample.jpg'
output_path = '../../output'


print("\n" + "="*60)
print("开始推理...")
print("="*60)

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

print("\n" + "="*60)
print("推理完成！")
print("="*60)

# 保存SAM输出到文件
if sam_outputs['local_features']:
    print(f"\n捕获到 {len(sam_outputs['local_features'])} 个SAM输出")
    
    # 保存为numpy文件
    for idx, sam_out in enumerate(sam_outputs['local_features']):
        np.save(f'{output_path}/sam_output_{idx}.npy', sam_out)
        print(f"保存 SAM 输出 {idx}: shape={sam_out.shape} -> {output_path}/sam_output_{idx}.npy")
        
        # 可选：可视化SAM特征的第一个通道
        if sam_out.shape[1] > 0:  # 确保有通道维度
            # 取第一个batch，第一个通道
            feature_map = sam_out[0, 0, :, :]
            
            # 归一化到 0-255
            feature_map_norm = ((feature_map - feature_map.min()) / (feature_map.max() - feature_map.min()) * 255).astype(np.uint8)
            
            # 保存为图像
            feature_img = Image.fromarray(feature_map_norm)
            feature_img.save(f'{output_path}/sam_feature_vis_{idx}.png')
            print(f"保存 SAM 特征可视化 {idx} -> {output_path}/sam_feature_vis_{idx}.png")

else:
    print("\n⚠ 警告：没有捕获到SAM输出")
    print("可能原因：")
    print("1. Hook注册位置不正确")
    print("2. 模型结构与预期不同")
    print("3. 需要查看HuggingFace Hub上的实际模型代码")

print("\n输出结果:", res)
print(f"\n结果已保存到: {output_path}")
print(f"- result.mmd: Markdown文档")
print(f"- result_with_boxes.jpg: 带边界框的图像")




















