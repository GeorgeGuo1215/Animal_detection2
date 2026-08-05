# -*- coding: utf-8 -*-
"""
中文字体测试脚本
用于测试matplotlib中文字体显示是否正常
"""

import matplotlib.pyplot as plt
import matplotlib
import platform
import numpy as np

def setup_chinese_font():
    """配置matplotlib中文字体支持"""
    system = platform.system()
    print(f"检测到系统: {system}")
    
    if system == "Darwin":  # macOS
        # macOS系统字体
        fonts = ['Arial Unicode MS', 'PingFang SC', 'Hiragino Sans GB', 'STHeiti', 'SimHei']
    elif system == "Windows":  # Windows
        # Windows系统字体
        fonts = ['SimHei', 'Microsoft YaHei', 'KaiTi', 'FangSong', 'STSong']
    else:  # Linux
        # Linux系统字体
        fonts = ['DejaVu Sans', 'WenQuanYi Micro Hei', 'SimHei', 'Noto Sans CJK SC']
    
    print("尝试设置中文字体...")
    
    # 尝试设置字体
    for font in fonts:
        try:
            matplotlib.rcParams['font.sans-serif'] = [font]
            matplotlib.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题
            
            # 测试字体是否可用
            fig, ax = plt.subplots(figsize=(2, 1))
            ax.text(0.5, 0.5, '测试中文字体', fontsize=12, ha='center', va='center')
            plt.close(fig)
            
            print(f"✓ 成功设置字体: {font}")
            return True
        except Exception as e:
            print(f"✗ 字体 {font} 设置失败: {e}")
            continue
    
    # 如果都不可用，使用默认设置
    print("⚠️  警告: 未找到合适的中文字体，可能无法正常显示中文")
    matplotlib.rcParams['axes.unicode_minus'] = False
    return False

def test_chinese_display():
    """测试中文显示效果"""
    print("\n=== 测试中文字体显示 ===")
    
    # 设置中文字体
    font_success = setup_chinese_font()
    
    # 创建测试图表
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle('毫米波雷达数据处理 - 中文字体测试', fontsize=16)
    
    # 生成测试数据
    x = np.linspace(0, 10, 100)
    y1 = np.sin(x)
    y2 = np.cos(x)
    
    # 测试图1 - 基本中文标签
    axes[0, 0].plot(x, y1, label='正弦波')
    axes[0, 0].set_title('原始I/Q信号')
    axes[0, 0].set_xlabel('采样点')
    axes[0, 0].set_ylabel('幅度')
    axes[0, 0].legend()
    axes[0, 0].grid(True)
    
    # 测试图2 - 散点图
    axes[0, 1].scatter(y1[::5], y2[::5], alpha=0.7, label='数据点')
    axes[0, 1].set_title('I/Q星座图与圆拟合')
    axes[0, 1].set_xlabel('I通道')
    axes[0, 1].set_ylabel('Q通道')
    axes[0, 1].legend()
    axes[0, 1].grid(True)
    
    # 测试图3 - 生理参数
    heart_rates = [72, 75, 68, 80, 77, 73, 76]
    axes[1, 0].plot(heart_rates, 'o-', color='red', label='心率数据')
    axes[1, 0].set_title('心率分布 (测试数据)')
    axes[1, 0].set_xlabel('文件序号')
    axes[1, 0].set_ylabel('心率 (bpm)')
    axes[1, 0].legend()
    axes[1, 0].grid(True)
    
    # 测试图4 - 呼吸频率
    resp_rates = [18, 20, 16, 22, 19, 17, 21]
    axes[1, 1].bar(range(len(resp_rates)), resp_rates, color='blue', alpha=0.7)
    axes[1, 1].set_title('呼吸频率分布 (测试数据)')
    axes[1, 1].set_xlabel('文件序号')
    axes[1, 1].set_ylabel('呼吸频率 (bpm)')
    axes[1, 1].grid(True)
    
    plt.tight_layout()
    
    # 保存测试图片
    try:
        plt.savefig('chinese_font_test.png', dpi=150, bbox_inches='tight')
        print("✓ 测试图片已保存为: chinese_font_test.png")
    except Exception as e:
        print(f"✗ 保存图片失败: {e}")
    
    # 显示图片
    try:
        plt.show()
        print("✓ 图片显示成功")
    except Exception as e:
        print(f"✗ 图片显示失败: {e}")
    
    return font_success

def list_available_fonts():
    """列出系统可用字体"""
    print("\n=== 系统可用字体列表 ===")
    
    try:
        from matplotlib import font_manager
        
        # 获取所有字体
        fonts = font_manager.findSystemFonts()
        font_names = set()
        
        for font_path in fonts:
            try:
                font_prop = font_manager.FontProperties(fname=font_path)
                font_name = font_prop.get_name()
                if any(ord(char) > 127 for char in font_name):  # 包含非ASCII字符，可能是中文字体
                    font_names.add(font_name)
            except:
                continue
        
        print("可能支持中文的字体:")
        for font_name in sorted(font_names):
            print(f"  - {font_name}")
        
        # 常见中文字体检查
        common_chinese_fonts = [
            'SimHei', 'Microsoft YaHei', 'KaiTi', 'FangSong', 'STSong',
            'PingFang SC', 'Hiragino Sans GB', 'STHeiti', 'Arial Unicode MS',
            'WenQuanYi Micro Hei', 'Noto Sans CJK SC'
        ]
        
        print("\n常见中文字体检查:")
        available_fonts = [f.name for f in font_manager.fontManager.ttflist]
        
        for font in common_chinese_fonts:
            if font in available_fonts:
                print(f"  ✓ {font} - 可用")
            else:
                print(f"  ✗ {font} - 不可用")
                
    except Exception as e:
        print(f"获取字体列表失败: {e}")

def main():
    """主函数"""
    print("=== matplotlib中文字体测试程序 ===")
    
    # 显示当前matplotlib配置
    print(f"\n当前matplotlib版本: {matplotlib.__version__}")
    print(f"当前字体设置: {matplotlib.rcParams['font.sans-serif']}")
    
    # 列出可用字体
    list_available_fonts()
    
    # 测试中文显示
    success = test_chinese_display()
    
    print("\n=== 测试结果 ===")
    if success:
        print("✓ 中文字体设置成功！")
        print("现在可以正常使用data_processor.py处理数据并显示中文图表。")
    else:
        print("✗ 中文字体设置失败！")
        print("建议:")
        print("1. 在macOS上安装中文字体或使用系统自带的中文字体")
        print("2. 在Windows上确保安装了SimHei或Microsoft YaHei字体")
        print("3. 在Linux上安装中文字体包: sudo apt-get install fonts-wqy-microhei")
        print("4. 或者手动下载字体文件放到matplotlib字体目录")

if __name__ == "__main__":
    main()













