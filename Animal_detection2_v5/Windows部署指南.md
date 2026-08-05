# Windows电脑快速部署指南

## 📋 简介
本文档说明如何在全新的Windows电脑上快速部署和运行毫米波雷达Web端应用。

---

## 🎯 方案一：使用Python（推荐，最简单）

### 1. 检查Python环境
打开命令提示符（CMD）或PowerShell，检查是否已安装Python：
```bash
python --version
```

### 2. 安装Python（如果未安装）
- 访问：https://www.python.org/downloads/
- 下载最新版Python（建议Python 3.10+）
- 安装时**勾选 "Add Python to PATH"**
- 完成后重新打开命令提示符验证：
```bash
python --version
```

### 3. 获取代码
#### 方式A：使用Git（推荐）
```bash
# 安装Git（如果未安装）：https://git-scm.com/download/win
# 克隆仓库
git clone https://github.com/GeorgeGuo1215/Animal_detection.git
cd Animal_detection/web
```

#### 方式B：直接下载
1. 访问：https://github.com/GeorgeGuo1215/Animal_detection
2. 点击绿色 "Code" 按钮 → "Download ZIP"
3. 解压ZIP文件
4. 进入 `web` 文件夹

### 4. 启动Web服务器
在 `web` 文件夹中打开命令提示符，运行：
```bash
python -m http.server 8000
```

### 5. 访问应用
打开浏览器（Chrome/Edge），访问：
```
http://localhost:8000
```

### 6. 停止服务器
在命令提示符窗口按 `Ctrl + C`

---

## 🎯 方案二：使用Node.js

### 1. 安装Node.js
- 访问：https://nodejs.org/
- 下载LTS版本（推荐）
- 安装完成后验证：
```bash
node --version
npm --version
```

### 2. 获取代码
同方案一的步骤3

### 3. 安装http-server
```bash
npm install -g http-server
```

### 4. 启动Web服务器
在 `web` 文件夹中运行：
```bash
http-server -p 8000
```

或者使用更强大的选项：
```bash
http-server -p 8000 -c-1 --cors
```
- `-p 8000`: 指定端口
- `-c-1`: 禁用缓存（开发时有用）
- `--cors`: 启用跨域支持

### 5. 访问应用
打开浏览器访问：
```
http://localhost:8000
```

---

## 🎯 方案三：使用Live Server（VS Code用户）

### 1. 安装VS Code
- 访问：https://code.visualstudio.com/
- 下载并安装

### 2. 安装Live Server扩展
1. 打开VS Code
2. 点击左侧扩展图标（或按 `Ctrl + Shift + X`）
3. 搜索 "Live Server"
4. 点击安装

### 3. 获取代码
同方案一的步骤3

### 4. 启动服务器
1. 用VS Code打开 `web` 文件夹
2. 右键点击 `index.html`
3. 选择 "Open with Live Server"
4. 浏览器自动打开应用

---

## 🌐 在线部署版本（无需安装）

如果你只是想测试应用，可以直接访问已部署的在线版本：

```
https://georgeguo1215.github.io/Animal_detection/
```

**注意**：在线版本需要HTTPS才能使用蓝牙功能。

---

## 🔧 常见问题

### Q1: 提示"python不是内部或外部命令"
**解决方案**：
- Python未正确安装或未添加到PATH
- 重新安装Python并勾选 "Add Python to PATH"
- 或者手动添加Python到系统PATH

### Q2: 蓝牙连接失败
**原因**：
- Web Bluetooth只在HTTPS或localhost下工作
- 仅支持Chrome/Edge浏览器
- 电脑需要有蓝牙硬件

**解决方案**：
- 确保使用Chrome或Edge浏览器
- 使用localhost访问（不是IP地址）
- 检查Windows蓝牙设置是否启用

### Q3: 端口8000已被占用
**解决方案**：
```bash
# Python - 使用其他端口
python -m http.server 8080

# Node.js - 使用其他端口
http-server -p 8080
```

### Q4: 页面样式丢失或脚本不加载
**原因**：浏览器缓存问题

**解决方案**：
- 强制刷新：`Ctrl + F5`
- 清除浏览器缓存
- 使用隐私模式测试

### Q5: 防火墙阻止访问
**解决方案**：
- 在Windows防火墙中允许Python/Node.js
- 或者暂时关闭防火墙测试

---

## 📱 移动设备访问（可选）

### 1. 确保电脑和手机在同一WiFi
### 2. 查找电脑IP地址
```bash
ipconfig
```
找到 "IPv4 地址"，例如：192.168.1.100

### 3. 启动服务器时绑定所有接口
```bash
# Python
python -m http.server 8000 --bind 0.0.0.0

# Node.js
http-server -p 8000 -a 0.0.0.0
```

### 4. 在手机浏览器访问
```
http://192.168.1.100:8000
```

**注意**：移动端无法使用蓝牙功能，仅适用于文件上传分析。

---

## 🚀 生产环境部署（进阶）

如需在服务器上长期运行，推荐使用专业Web服务器：

### 选项1：Nginx（Windows版）
- 下载：http://nginx.org/en/download.html
- 配置静态文件服务
- 适合高性能需求

### 选项2：IIS（Windows自带）
- 启用IIS功能
- 配置静态网站
- 适合Windows Server环境

---

## 📞 技术支持

如遇到问题，请检查：
1. 浏览器控制台（F12）的错误信息
2. 命令提示符的错误输出
3. Windows防火墙和杀毒软件设置

GitHub Issues: https://github.com/GeorgeGuo1215/Animal_detection/issues

---

## ✅ 快速检查清单

- [ ] Python/Node.js已安装
- [ ] 代码已下载到本地
- [ ] 命令提示符在 `web` 文件夹中
- [ ] Web服务器成功启动
- [ ] 浏览器能访问 http://localhost:8000
- [ ] 使用Chrome或Edge浏览器
- [ ] 蓝牙已启用（如需使用蓝牙功能）

---

**祝你部署顺利！** 🎉