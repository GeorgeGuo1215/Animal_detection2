from __future__ import annotations

import platform


def _bytes_to_gb(x: int) -> float:
    return x / (1024**3)


def main():
    print("=== LoRA 环境自检 ===")
    print(f"platform: {platform.platform()}")

    try:
        import torch
    except Exception as e:  # noqa: BLE001
        print("torch: NOT INSTALLED")
        raise SystemExit(f"无法导入 torch：{e}") from e

    print(f"torch: {torch.__version__}")
    print(f"cuda available: {torch.cuda.is_available()}")
    print(f"torch cuda version: {getattr(torch.version, 'cuda', None)}")

    if torch.cuda.is_available():
        n = torch.cuda.device_count()
        print(f"cuda device_count: {n}")
        for i in range(n):
            name = torch.cuda.get_device_name(i)
            props = torch.cuda.get_device_properties(i)
            total_gb = _bytes_to_gb(props.total_memory)
            print(f"- GPU[{i}]: {name} | total_mem={total_gb:.2f} GB")
    else:
        print("提示：如果你机器有 NVIDIA GPU（如 3080Ti），但这里显示 cuda False，通常是：")
        print("- 当前虚拟环境装的是 CPU 版 torch；或")
        print("- CUDA/驱动未配置好；或")
        print("- 你没有切到正确的 conda/venv。")


if __name__ == "__main__":
    main()


