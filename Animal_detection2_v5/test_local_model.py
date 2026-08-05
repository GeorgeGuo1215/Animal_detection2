#!/usr/bin/env python3
"""
测试本地HuggingFace模型是否存在，并提供离线模式配置
"""
import os
import sys
from pathlib import Path

def check_huggingface_cache():
    """检查HuggingFace缓存目录"""
    # 常见的缓存路径
    cache_paths = [
        Path.home() / ".cache" / "huggingface" / "hub",
        Path.home() / ".cache" / "torch" / "hub",
        Path.home() / ".cache" / "transformers",
    ]

    model_name = "intfloat/multilingual-e5-small"
    print(f"🔍 正在查找模型: {model_name}")
    print(f"📂 缓存路径: {[str(p) for p in cache_paths]}")

    found_paths = []
    for cache_path in cache_paths:
        if cache_path.exists():
            # 查找包含模型名的目录
            for item in cache_path.rglob("*"):
                if item.is_dir() and model_name.replace("/", "--") in str(item):
                    found_paths.append(item)
                    print(f"✅ 找到本地模型: {item}")

    if not found_paths:
        print("❌ 未找到本地模型缓存")
        print("\n💡 建议解决方案:")
        print("1. 确保网络连接正常，首次运行时会自动下载模型")
        print("2. 或者手动下载模型到本地缓存目录")
        print("3. 使用离线模式启动（如果已下载过）")
        return False

    print(f"🎉 找到 {len(found_paths)} 个本地模型副本")
    return True

def create_offline_startup_script():
    """创建离线模式启动脚本"""
    script_content = '''#!/usr/bin/env python3
"""
离线模式启动Agent API
"""
import os
import subprocess
import sys

# 设置离线模式环境变量
os.environ['HF_HUB_OFFLINE'] = '1'
os.environ['TRANSFORMERS_OFFLINE'] = '1'
os.environ['SENTENCE_TRANSFORMERS_HOME'] = os.path.expanduser('~/.cache/torch/sentence_transformers')

# 禁用网络请求
os.environ['HF_HUB_DISABLE_TELEMETRY'] = '1'

print("🚀 启动Agent API (离线模式)")
print(f"📍 HF_HUB_OFFLINE: {os.environ.get('HF_HUB_OFFLINE')}")
print(f"📍 SENTENCE_TRANSFORMERS_HOME: {os.environ.get('SENTENCE_TRANSFORMERS_HOME')}")

# 启动agent
cmd = [
    sys.executable, "-m", "uvicorn",
    "agentAndRag.agent_api.app.main:app",
    "--host", "127.0.0.1",
    "--port", "9001"
]

print(f"🔧 执行命令: {' '.join(cmd)}")
subprocess.run(cmd)
'''

    script_path = Path(__file__).parent / "start_agent_offline.py"
    with open(script_path, 'w', encoding='utf-8') as f:
        f.write(script_content)

    print(f"📝 已创建离线启动脚本: {script_path}")
    return script_path

def create_environment_file():
    """创建环境变量配置文件"""
    env_content = '''# HuggingFace 离线模式配置
HF_HUB_OFFLINE=1
TRANSFORMERS_OFFLINE=1
HF_HUB_DISABLE_TELEMETRY=1
SENTENCE_TRANSFORMERS_HOME=%USERPROFILE%\\.cache\\torch\\sentence_transformers

# Agent API 配置
AGENT_WARMUP_RAG=1
AGENT_WARMUP_EMBEDDING_MODEL=intfloat/multilingual-e5-small
AGENT_WARMUP_DEVICE=cpu
AGENT_WARMUP_BM25=1
AGENT_WARMUP_RERANKER=0
'''

    env_path = Path(__file__).parent / ".env"
    with open(env_path, 'w', encoding='utf-8') as f:
        f.write(env_content)

    print(f"📝 已创建环境配置文件: {env_path}")
    return env_path

if __name__ == "__main__":
    print("🔧 HuggingFace 模型检查工具")
    print("=" * 50)

    # 检查本地模型
    has_local_model = check_huggingface_cache()

    print("\n" + "=" * 50)

    # 创建配置文件
    print("📝 正在创建离线模式配置文件...")
    offline_script = create_offline_startup_script()
    env_file = create_environment_file()

    print("\n🎯 使用方法:")
    print("1. 如果有本地模型，使用离线脚本启动:")
    print(f"   python {offline_script}")
    print()
    print("2. 或者设置环境变量后启动:")
    print(f"   加载 {env_file} 中的环境变量")
    print("   然后运行: python -m uvicorn agentAndRag.agent_api.app.main:app --host 127.0.0.1 --port 9001")
    print()
    print("3. 如果没有本地模型，确保网络连接正常，首次启动会自动下载")

    if has_local_model:
        print("\n✅ 检测到本地模型，可以使用离线模式启动")
    else:
        print("\n⚠️  未检测到本地模型，首次启动需要网络连接下载模型")