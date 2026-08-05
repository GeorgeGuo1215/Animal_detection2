# -*- coding: utf-8 -*-
"""
数据处理程序测试脚本
用于验证data_processor.py是否能正常工作
"""

import os
import sys

def test_import():
    """测试模块导入"""
    try:
        from data_processor import RadarDataProcessor
        print("✓ 成功导入 RadarDataProcessor")
        return True
    except ImportError as e:
        print(f"✗ 导入失败: {e}")
        return False

def test_data_files():
    """检查数据文件"""
    data_dir = "./data"
    if not os.path.exists(data_dir):
        print(f"✗ 数据目录不存在: {data_dir}")
        return False
    
    txt_files = [f for f in os.listdir(data_dir) if f.endswith('.txt')]
    if not txt_files:
        print(f"✗ 在 {data_dir} 中未找到txt文件")
        return False
    
    print(f"✓ 找到 {len(txt_files)} 个数据文件")
    return True

def test_single_file_processing():
    """测试单文件处理"""
    try:
        from data_processor import RadarDataProcessor
        
        processor = RadarDataProcessor()
        
        # 找到第一个数据文件
        data_dir = "./data"
        txt_files = [f for f in os.listdir(data_dir) if f.endswith('.txt')]
        if not txt_files:
            print("✗ 没有数据文件可供测试")
            return False
        
        test_file = os.path.join(data_dir, txt_files[0])
        print(f"测试文件: {test_file}")
        
        # 测试数据读取
        timestamps, i_data, q_data = processor.read_data_file(test_file)
        if i_data is None:
            print("✗ 数据读取失败")
            return False
        
        print(f"✓ 成功读取数据，共 {len(i_data)} 个点")
        
        # 测试圆拟合
        center, radius = processor.circle_fitting(i_data, q_data)
        print(f"✓ 圆拟合完成，圆心: ({center[0]:.4f}, {center[1]:.4f}), 半径: {radius:.4f}")
        
        # 测试相位解调
        phase_data = processor.arcsin_demodulation(i_data, q_data, center, radius)
        print(f"✓ 相位解调完成，数据范围: {phase_data.min():.4f} ~ {phase_data.max():.4f}")
        
        # 测试生理参数提取
        heart_rate, respiratory_rate = processor.extract_vital_signs(i_data, q_data, phase_data)
        print(f"✓ 生理参数提取完成，心率: {heart_rate} bpm, 呼吸: {respiratory_rate} bpm")
        
        return True
        
    except Exception as e:
        print(f"✗ 单文件处理测试失败: {e}")
        return False

def test_dependencies():
    """测试依赖包"""
    required_packages = ['numpy', 'matplotlib', 'scipy', 'pandas']
    missing_packages = []
    
    for package in required_packages:
        try:
            __import__(package)
            print(f"✓ {package} 已安装")
        except ImportError:
            print(f"✗ {package} 未安装")
            missing_packages.append(package)
    
    if missing_packages:
        print(f"\n请安装缺失的包:")
        print(f"pip install {' '.join(missing_packages)}")
        return False
    
    return True

def test_chinese_font():
    """测试中文字体支持"""
    try:
        import matplotlib.pyplot as plt
        import matplotlib
        
        # 测试中文字体设置
        fig, ax = plt.subplots(figsize=(6, 4))
        ax.text(0.5, 0.5, '中文字体测试\n心率: 75 bpm\n呼吸频率: 18 bpm', 
                fontsize=14, ha='center', va='center')
        ax.set_title('中文字体显示测试')
        ax.set_xlabel('测试X轴标签')
        ax.set_ylabel('测试Y轴标签')
        
        # 保存测试图片
        plt.savefig('font_test.png', dpi=100, bbox_inches='tight')
        plt.close()
        
        print("✓ 中文字体测试图片已保存为 font_test.png")
        print(f"  当前字体设置: {matplotlib.rcParams['font.sans-serif']}")
        return True
        
    except Exception as e:
        print(f"✗ 中文字体测试失败: {e}")
        return False

def main():
    """主测试函数"""
    print("=== 数据处理程序测试 ===\n")
    
    tests = [
        ("依赖包检查", test_dependencies),
        ("中文字体测试", test_chinese_font),
        ("模块导入测试", test_import),
        ("数据文件检查", test_data_files),
        ("单文件处理测试", test_single_file_processing),
    ]
    
    passed = 0
    total = len(tests)
    
    for test_name, test_func in tests:
        print(f"\n--- {test_name} ---")
        if test_func():
            passed += 1
            print(f"✓ {test_name} 通过")
        else:
            print(f"✗ {test_name} 失败")
    
    print(f"\n=== 测试结果 ===")
    print(f"通过: {passed}/{total}")
    
    if passed == total:
        print("🎉 所有测试通过！程序可以正常使用。")
        print("\n使用方法:")
        print("1. 批量处理: python data_processor.py")
        print("2. 单文件处理: python data_processor.py -f data/filename.txt")
        print("3. 运行示例: python process_example.py")
    else:
        print("❌ 部分测试失败，请检查上述错误信息。")

if __name__ == "__main__":
    main()
