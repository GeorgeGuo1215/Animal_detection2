# 毫米波雷达数据处理程序

这是一个独立的数据处理程序，用于处理毫米波雷达采集的生理信号数据。程序从原始的main.py中提取了核心算法，可以离线处理data目录中的数据文件。

## 功能特性

- **数据读取**: 自动解析data目录中的txt格式数据文件
- **圆拟合校正**: 校正I/Q信号的直流偏移和幅度不平衡
- **相位解调**: 使用ARCSIN算法提取相位信息
- **数字滤波**: 分离呼吸和心跳信号
- **频谱分析**: 通过FFT提取心率和呼吸频率
- **批量处理**: 支持批量处理多个数据文件
- **结果可视化**: 生成处理结果图表
- **数据导出**: 将结果保存为CSV格式

## 安装依赖

```bash
pip install numpy matplotlib scipy pandas
```

### 中文字体支持

程序会自动检测并配置中文字体。如果遇到中文显示问题：

#### macOS系统
- 系统通常自带中文字体，程序会自动使用PingFang SC或Hiragino Sans GB
- 如果显示异常，可以安装Arial Unicode MS字体

#### Windows系统
- 确保系统安装了SimHei或Microsoft YaHei字体
- 通常Windows系统自带这些字体

#### Linux系统
- 安装中文字体包：
  ```bash
  sudo apt-get install fonts-wqy-microhei
  # 或者
  sudo apt-get install fonts-noto-cjk
  ```

#### 测试中文字体
运行字体测试程序检查配置：
```bash
python test_chinese_font.py
```

## 使用方法

### 1. 命令行使用

#### 批量处理所有数据文件
```bash
python data_processor.py --data_dir ./data --output results.csv --plot
```

#### 处理单个文件
```bash
python data_processor.py --single_file ./data/2022-03-08-11-22\ -\ 1.txt --plot
```

#### 命令行参数说明
- `--data_dir, -d`: 数据目录路径 (默认: ./data)
- `--output, -o`: 输出CSV文件路径 (默认: processing_results.csv)
- `--single_file, -f`: 处理单个文件
- `--plot, -p`: 显示处理结果图
- `--no_summary`: 不显示汇总图

### 2. Python脚本使用

#### 运行示例脚本
```bash
python process_example.py
```

#### 在代码中使用
```python
from data_processor import RadarDataProcessor

# 创建处理器
processor = RadarDataProcessor()

# 处理单个文件
result = processor.process_single_file('data/sample.txt', plot_results=True)

# 批量处理
results = processor.batch_process('./data', output_csv='results.csv')
```

## 数据格式

程序支持以下格式的数据文件：
```
2022-03-08-11-22-35  -0.496  -0.78
2022-03-08-11-22-35  -0.504  -0.788
...
```

每行包含：
- 时间戳
- I通道数据
- Q通道数据

## 处理流程

1. **数据读取**: 解析txt文件，提取I/Q数据
2. **圆拟合**: 计算I/Q信号的圆心和半径，校正硬件不完美性
3. **相位解调**: 使用ARCSIN算法提取相位信息
4. **信号分离**: 
   - 低通滤波提取呼吸信号
   - 高通滤波提取心跳信号
5. **频谱分析**: FFT分析提取频率特征
6. **参数提取**: 计算心率和呼吸频率

## 输出结果

### 单文件处理结果
- 心率 (bpm)
- 呼吸频率 (bpm)
- 圆拟合参数
- 处理后的波形数据

### 批量处理结果
- CSV文件包含所有文件的处理结果
- 汇总统计图表
- 各文件的详细参数

### 可视化图表
1. **原始I/Q信号图**: 显示原始数据
2. **I/Q星座图**: 显示圆拟合结果
3. **呼吸波形图**: 提取的呼吸信号
4. **心跳波形图**: 提取的心跳信号

## 算法原理

### 圆拟合算法
用于校正I/Q信号中的直流偏移和幅度不平衡问题。通过最小二乘法拟合I/Q数据点形成的圆，得到圆心和半径参数。

### ARCSIN解调
相位解调算法，通过计算相邻采样点之间的相位差来提取目标的微小运动信息：
```
φ(n) = φ(n-1) + arcsin((I(n-1)×Q(n) - I(n)×Q(n-1)) / (|Z(n-1)|×|Z(n)|))
```

### 频谱分析
使用FFT分析提取特定频率范围内的峰值：
- 呼吸频率范围: 0.1-0.5 Hz (6-30 bpm)
- 心率频率范围: 0.8-3.0 Hz (48-180 bpm)

## 注意事项

1. **数据质量**: 确保数据文件格式正确，包含足够的数据点
2. **参数调整**: 可根据实际情况调整频率范围和滤波参数
3. **环境干扰**: 处理结果可能受到环境噪声影响
4. **文件路径**: 确保数据文件路径正确，支持中文路径

## 故障排除

### 常见问题
1. **导入错误**: 检查是否安装了所需的Python包
2. **文件读取失败**: 检查文件路径和格式是否正确
3. **处理结果异常**: 可能是数据质量问题或参数设置不当
4. **中文显示异常**: 图表中的中文标签显示为方框或乱码

### 中文字体问题解决
如果遇到中文显示问题：

1. **运行字体测试**:
   ```bash
   python test_chinese_font.py
   ```

2. **手动安装字体**:
   - macOS: 下载并安装中文字体到字体册
   - Windows: 下载字体文件到 C:\Windows\Fonts\
   - Linux: 复制字体文件到 ~/.fonts/ 或 /usr/share/fonts/

3. **清除matplotlib缓存**:
   ```bash
   rm -rf ~/.matplotlib
   ```

4. **指定字体路径** (高级用户):
   ```python
   import matplotlib.font_manager as fm
   font_path = '/path/to/your/font.ttf'
   prop = fm.FontProperties(fname=font_path)
   plt.rcParams['font.family'] = prop.get_name()
   ```

### 调试建议
1. 使用单文件处理模式测试
2. 检查原始数据的I/Q信号质量
3. 调整滤波器参数或频率范围
4. 查看圆拟合结果是否合理
5. 运行 `python test_processor.py` 进行全面测试

## 扩展功能

程序设计为模块化结构，可以轻松扩展：
- 添加新的滤波算法
- 实现其他解调方法
- 增加更多的生理参数提取
- 支持其他数据格式

## 联系信息

如有问题或建议，请参考原始项目文档或联系开发团队。
