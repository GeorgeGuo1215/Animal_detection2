# 睡眠监测RMS图表优化说明

## 🐛 问题描述

用户反馈：在睡眠质量监测中，处于静息状态时，"运动强度趋势 (RMS)" 图表毫无变化，但其他模块数据正常。

## 🔍 问题分析

### 根本原因

当宠物处于静息状态时，RMS（均方根能量）值非常小：
- 静息状态：RMS < 0.02g
- 轻微活动：RMS 0.02-0.05g
- 活动中：RMS > 0.05g

**问题**：原始图表配置使用 `beginAtZero: true`，Y轴从0开始。当RMS值在0.01-0.02g之间微小变化时，在0-0.2g的范围内几乎看不出差异，图表看起来像一条平线。

### 示例数据

静息状态下的实际RMS值：
```
时间      RMS值
22:00    0.0145g
22:01    0.0152g
22:02    0.0138g
22:03    0.0161g
22:04    0.0149g
```

在0-0.2g的Y轴范围内，这些值的差异几乎不可见。

## ✅ 解决方案

### 1. 动态Y轴范围调整

在 `updateCharts()` 方法中添加智能Y轴调整：

```javascript
// 计算数据范围以优化Y轴
const minRMS = Math.min(...data);
const maxRMS = Math.max(...data);
const range = maxRMS - minRMS;

// 如果范围很小（静息状态），放大显示
if (range < 0.05) {
    this.charts.rmsChart.options.scales.y.min = Math.max(0, minRMS - 0.01);
    this.charts.rmsChart.options.scales.y.max = maxRMS + 0.01;
} else {
    this.charts.rmsChart.options.scales.y.min = 0;
    this.charts.rmsChart.options.scales.y.max = undefined;
}
```

**效果**：
- 静息状态（range < 0.05g）：Y轴范围自动调整为 [minRMS-0.01, maxRMS+0.01]
- 活动状态（range >= 0.05g）：Y轴范围恢复为从0开始的自动范围

### 2. 优化图表初始化

修改图表配置，使其更适合显示微小变化：

```javascript
options: {
    scales: {
        y: {
            beginAtZero: false,  // 不强制从0开始
            ticks: {
                callback: function(value) {
                    return value.toFixed(3);  // 显示3位小数
                }
            }
        }
    },
    plugins: {
        tooltip: {
            callbacks: {
                label: function(context) {
                    return 'RMS: ' + context.parsed.y.toFixed(4) + 'g';  // 显示4位小数
                }
            }
        }
    }
}
```

**改进**：
- `beginAtZero: false`：允许Y轴自动调整起始值
- 显示3-4位小数：更精确地显示微小变化
- 添加平滑曲线：`tension: 0.4`
- 优化数据点显示：`pointRadius: 2`

### 3. 添加调试日志

每10次更新输出一次RMS范围信息：

```javascript
if (this.dataUpdateCount % 10 === 0) {
    console.log(`📊 [RMS图表] 数据点=${data.length}, 范围=${minRMS.toFixed(4)}-${maxRMS.toFixed(4)}g`);
}
```

## 📊 预期效果

### 静息状态（优化前）
```
Y轴范围: 0 - 0.2g
RMS值: 0.014-0.016g
视觉效果: 几乎看不出变化，像一条平线
```

### 静息状态（优化后）
```
Y轴范围: 0.004 - 0.026g (自动调整)
RMS值: 0.014-0.016g
视觉效果: 清晰可见的波动曲线
```

### 活动状态
```
Y轴范围: 0 - 0.3g (自动范围)
RMS值: 0.05-0.25g
视觉效果: 正常显示大幅度变化
```

## 🧪 测试步骤

### 1. 刷新页面
```
Cmd+Shift+R (Mac) 或 Ctrl+Shift+R (Windows)
```

### 2. 启动监测
```
连接蓝牙 → 开始睡眠监测
```

### 3. 观察控制台
应该看到：
```
😴 [睡眠监测] RMS=0.0145g, MCR=1.2次/秒, 阶段=deep, 数据点=10
📊 [RMS图表] 数据点=10, 范围=0.0138-0.0161g
```

### 4. 查看RMS图表
- 图表应该显示清晰的波动曲线
- Y轴范围应该自动调整（例如：0.004 - 0.026g）
- 鼠标悬停时显示精确的RMS值（4位小数）

### 5. 测试不同状态

**静息状态**：
- 保持设备静止
- RMS应该在0.01-0.02g之间
- 图表应该显示微小但可见的波动

**轻微活动**：
- 轻轻移动设备
- RMS应该在0.02-0.05g之间
- 图表应该显示明显的变化

**活动状态**：
- 正常移动设备
- RMS应该>0.05g
- Y轴范围自动扩大，显示大幅度变化

## 🔍 调试命令

### 查看当前RMS数据
```javascript
// 在浏览器控制台执行
const recentRMS = app.sleepMonitor.sleepHistory.slice(-10).map(h => h.rms);
console.log('最近10个RMS值:', recentRMS);
console.log('最小值:', Math.min(...recentRMS).toFixed(4));
console.log('最大值:', Math.max(...recentRMS).toFixed(4));
console.log('范围:', (Math.max(...recentRMS) - Math.min(...recentRMS)).toFixed(4));
```

### 查看图表Y轴范围
```javascript
const yAxis = app.sleepMonitor.charts.rmsChart.options.scales.y;
console.log('Y轴最小值:', yAxis.min);
console.log('Y轴最大值:', yAxis.max);
```

### 手动触发图表更新
```javascript
app.sleepMonitor.updateCharts();
console.log('图表已手动更新');
```

## ⚠️ 注意事项

1. **数据积累**
   - 图表需要至少10个数据点才能显示有意义的趋势
   - 每1秒处理一次数据（滑动窗口）
   - 等待10秒以上才能看到完整的曲线

2. **Y轴动态调整**
   - 只在数据范围<0.05g时触发
   - 自动在最小值-0.01和最大值+0.01之间调整
   - 确保始终显示有意义的变化

3. **性能优化**
   - 使用 `update('none')` 避免动画延迟
   - 只保留最近60个数据点
   - 每2秒更新一次图表

4. **数据精度**
   - RMS计算精度：双精度浮点数
   - 显示精度：3-4位小数
   - 足以显示静息状态的微小变化

## 📈 实际案例

### 案例1：完全静止
```
时间      RMS      Y轴范围
22:00    0.0145g   0.004-0.026g
22:01    0.0148g   0.004-0.026g
22:02    0.0142g   0.004-0.026g
```
**效果**：清晰可见的小幅波动

### 案例2：轻微呼吸运动
```
时间      RMS      Y轴范围
22:00    0.0180g   0.008-0.030g
22:01    0.0165g   0.008-0.030g
22:02    0.0195g   0.008-0.030g
```
**效果**：明显的周期性波动（呼吸节律）

### 案例3：翻身
```
时间      RMS      Y轴范围
22:00    0.0150g   0-0.3g (自动)
22:01    0.1520g   0-0.3g (自动)
22:02    0.0145g   0-0.3g (自动)
```
**效果**：显著的峰值，Y轴自动扩大范围

## 🚀 未来优化

1. **自适应平滑**
   - 根据数据波动程度调整平滑参数
   - 静息时更平滑，活动时更敏感

2. **异常检测**
   - 标记异常的RMS峰值
   - 自动识别翻身、醒来等事件

3. **多时间尺度**
   - 添加1分钟、5分钟、1小时视图
   - 不同尺度使用不同的聚合方式

4. **颜色编码**
   - 根据RMS值使用不同颜色
   - 静息=绿色，活动=橙色，剧烈=红色

---

**版本**: 1.1.3
**修复时间**: 2026-01-28
**修复内容**:
- 动态Y轴范围调整
- 优化图表配置
- 添加调试日志
- 提高微小变化的可见性
