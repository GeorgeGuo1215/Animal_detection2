# -*- coding: utf-8 -*-
"""
数据处理示例脚本
演示如何使用data_processor.py处理毫米波雷达数据

使用方法:
1. 直接运行此脚本进行批量处理
2. 或者导入data_processor模块自定义处理
"""

from data_processor import RadarDataProcessor
import os

def example_single_file():
    """单文件处理示例"""
    print("=== 单文件处理示例 ===")
    
    # 创建处理器
    processor = RadarDataProcessor()
    
    # 处理单个文件（请根据实际情况修改文件路径）
    data_dir = "./data"
    files = [f for f in os.listdir(data_dir) if f.endswith('.txt')]
    
    if files:
        file_path = os.path.join(data_dir, files[1])
        print(f"处理文件: {file_path}")
        
        result = processor.process_single_file(file_path, plot_results=True)
        
        if result:
            print(f"处理结果:")
            print(f"  心率: {result['heart_rate']} bpm")
            print(f"  呼吸频率: {result['respiratory_rate']} bpm")
            print(f"  数据点数: {result['data_points']}")
    else:
        print("未找到数据文件")

def example_batch_processing():
    """批量处理示例"""
    print("\n=== 批量处理示例 ===")
    
    # 创建处理器
    processor = RadarDataProcessor()
    
    # 批量处理data目录中的所有文件
    results = processor.batch_process(
        data_directory="./data",
        output_csv="batch_results.csv",
        plot_summary=True
    )
    
    if results:
        print(f"成功处理 {len(results)} 个文件")
        
        # 显示前几个结果
        print("\n前3个文件的处理结果:")
        for i, result in enumerate(results[:3]):
            print(f"{i+1}. {result['file_name']}")
            print(f"   心率: {result['heart_rate']} bpm")
            print(f"   呼吸频率: {result['respiratory_rate']} bpm")

def example_custom_processing():
    """自定义处理示例"""
    print("\n=== 自定义处理示例 ===")
    
    # 创建处理器，自定义采样率
    processor = RadarDataProcessor(sampling_rate=100)
    
    data_dir = "./data"
    files = [f for f in os.listdir(data_dir) if f.endswith('.txt')]
    
    if files:
        file_path = os.path.join(data_dir, files[0])
        
        # 逐步处理演示
        print("1. 读取数据...")
        timestamps, i_data, q_data = processor.read_data_file(file_path)
        
        if i_data is not None:
            print(f"   数据点数: {len(i_data)}")
            
            print("2. 圆拟合校正...")
            center, radius = processor.circle_fitting(i_data, q_data)
            print(f"   圆心: ({center[0]:.4f}, {center[1]:.4f})")
            print(f"   半径: {radius:.4f}")
            
            print("3. 相位解调...")
            phase_data = processor.arcsin_demodulation(i_data, q_data, center, radius)
            print(f"   相位数据范围: {np.min(phase_data):.4f} ~ {np.max(phase_data):.4f}")
            
            print("4. 滤波处理...")
            respiratory_wave, heartbeat_wave = processor.apply_filters(phase_data)
            
            print("5. 提取生理参数...")
            heart_rate, respiratory_rate = processor.extract_vital_signs(i_data, q_data, phase_data)
            print(f"   心率: {heart_rate} bpm")
            print(f"   呼吸频率: {respiratory_rate} bpm")

if __name__ == "__main__":
    import numpy as np
    
    # 检查数据目录是否存在
    if not os.path.exists("./data"):
        print("错误: 未找到data目录")
        print("请确保在包含data目录的文件夹中运行此脚本")
        exit(1)
    
    try:
        # 运行示例
        example_single_file()
        example_batch_processing()
        example_custom_processing()
        
    except ImportError as e:
        print(f"导入错误: {e}")
        print("请确保已安装所需的Python包:")
        print("pip install numpy matplotlib scipy pandas")
    except Exception as e:
        print(f"运行错误: {e}")
