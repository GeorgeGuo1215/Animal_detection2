"""
调试脚本：递归查找模型中的所有模块，定位SAM模型
"""

from transformers import AutoModel, AutoTokenizer
import torch
import os

os.environ["CUDA_VISIBLE_DEVICES"] = '0'

model_name = 'deepseek-ai/DeepSeek-OCR'

print("加载模型...")
tokenizer = AutoTokenizer.from_pretrained(model_name, trust_remote_code=True)
model = AutoModel.from_pretrained(model_name, trust_remote_code=True, use_safetensors=True)
model = model.eval().cuda().to(torch.bfloat16)

print("\n" + "="*80)
print("1. 递归搜索所有模块（包含ImageEncoder或SAM相关）")
print("="*80)

def find_modules_recursive(module, prefix='', depth=0, max_depth=10):
    """递归查找所有模块"""
    if depth > max_depth:
        return
    
    for name, child in module.named_children():
        full_name = f"{prefix}.{name}" if prefix else name
        class_name = type(child).__name__
        
        # 查找包含关键词的模块
        keywords = ['sam', 'vision', 'encoder', 'image', 'embed', 'patch']
        if any(kw in name.lower() or kw in class_name.lower() for kw in keywords):
            print(f"{'  ' * depth}[{depth}] {full_name}")
            print(f"{'  ' * depth}    └─ Type: {class_name}")
            print(f"{'  ' * depth}    └─ Has forward: {hasattr(child, 'forward')}")
            
        # 递归
        find_modules_recursive(child, full_name, depth + 1, max_depth)

find_modules_recursive(model)

print("\n" + "="*80)
print("2. 查看顶层模型结构")
print("="*80)

for name, child in model.named_children():
    print(f"{name}: {type(child).__name__}")
    if hasattr(child, 'named_children'):
        for sub_name, sub_child in child.named_children():
            print(f"  └─ {sub_name}: {type(sub_child).__name__}")

print("\n" + "="*80)
print("3. 检查 model.model 属性")
print("="*80)

if hasattr(model, 'model'):
    print("✓ model.model 存在")
    print(f"  Type: {type(model.model).__name__}")
    
    if hasattr(model.model, 'named_children'):
        print("  子模块:")
        for name, child in model.model.named_children():
            print(f"    └─ {name}: {type(child).__name__}")
            
            # 检查vision相关的模块
            if 'vision' in name.lower():
                print(f"       [Vision模块详情]")
                for sub_name, sub_child in child.named_children():
                    print(f"         └─ {sub_name}: {type(sub_child).__name__}")

print("\n" + "="*80)
print("4. 搜索所有包含ImageEncoderViT的模块")
print("="*80)

found_sam = False
for name, module in model.named_modules():
    class_name = type(module).__name__
    if 'ImageEncoder' in class_name or 'SAM' in class_name or class_name == 'ImageEncoderViT':
        print(f"✓ 找到! 路径: {name}")
        print(f"  类型: {class_name}")
        found_sam = True
        
        # 尝试注册hook
        try:
            def test_hook(m, inp, out):
                print(f"  [Hook触发] 输出形状: {out.shape if hasattr(out, 'shape') else type(out)}")
            
            handle = module.register_forward_hook(test_hook)
            print(f"  Hook注册成功!")
        except Exception as e:
            print(f"  Hook注册失败: {e}")

if not found_sam:
    print("⚠ 未找到ImageEncoderViT类型的模块")

print("\n" + "="*80)
print("5. 检查模型的 infer 方法")
print("="*80)

if hasattr(model, 'infer'):
    print("✓ model.infer 方法存在")
    import inspect
    
    # 尝试获取源代码
    try:
        source = inspect.getsource(model.infer)
        print("\ninfer 方法前50行:")
        lines = source.split('\n')[:50]
        for i, line in enumerate(lines, 1):
            print(f"{i:3d} | {line}")
    except Exception as e:
        print(f"⚠ 无法获取源代码: {e}")
        
        # 至少显示签名
        try:
            sig = inspect.signature(model.infer)
            print(f"方法签名: infer{sig}")
        except Exception as e2:
            print(f"⚠ 无法获取签名: {e2}")
else:
    print("⚠ model.infer 方法不存在")

print("\n" + "="*80)
print("6. 查找模型定义文件的位置")
print("="*80)

if hasattr(model, '__module__'):
    print(f"模块名: {model.__module__}")
    
    # 尝试找到实际文件
    try:
        import importlib
        module_obj = importlib.import_module(model.__module__)
        if hasattr(module_obj, '__file__'):
            print(f"文件位置: {module_obj.__file__}")
    except Exception as e:
        print(f"无法定位文件: {e}")

# 检查缓存目录
import os
from pathlib import Path

cache_dirs = [
    Path.home() / ".cache" / "huggingface" / "modules" / "transformers_modules",
    Path(os.environ.get('HF_HOME', Path.home() / ".cache" / "huggingface")) / "modules",
]

print("\n可能的模型代码位置:")
for cache_dir in cache_dirs:
    if cache_dir.exists():
        print(f"  {cache_dir}")
        # 查找包含deepseek的目录
        try:
            for item in cache_dir.rglob("*deepseek*"):
                if item.is_dir():
                    print(f"    └─ {item}")
        except:
            pass

print("\n" + "="*80)
print("调试完成!")
print("="*80)




















