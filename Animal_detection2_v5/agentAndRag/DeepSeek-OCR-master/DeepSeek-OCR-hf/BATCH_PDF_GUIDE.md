# 📚 批量PDF处理指南

## 🎯 功能说明

这个脚本可以批量处理大量PDF文件，将每个PDF转换为一个Markdown文档。

**特点**:
- ✅ 自动扫描目录下所有PDF
- ✅ 逐页处理，避免内存溢出
- ✅ 支持超大型PDF（1000+页）
- ✅ 断点续传（可中断后继续）
- ✅ 详细的进度跟踪和日志
- ✅ 自动清理内存
- ✅ 错误处理和恢复

---

## 🚀 快速开始

### Step 1: 安装依赖

**1.1 安装Python库**

```bash
cd F:\BaiduNetdiskDownload\港城大\deepseek-ocr\DeepSeek-OCR\DeepSeek-OCR-master\DeepSeek-OCR-hf
pip install -r requirements_pdf.txt
```

**1.2 安装Poppler (Windows)**

pdf2image需要Poppler支持：

1. 下载Poppler for Windows:
   https://github.com/oschwartz10612/poppler-windows/releases/

2. 下载最新版本的 `Release-XX.XX.X-0.zip`

3. 解压到一个目录，例如: `C:\Program Files\poppler-xx.xx`

4. 添加到系统PATH:
   - 右键"此电脑" → "属性" → "高级系统设置"
   - "环境变量" → "系统变量" → "Path" → "编辑"
   - 添加: `C:\Program Files\poppler-xx.xx\Library\bin`

5. 重启命令行窗口

6. 验证安装:
   ```bash
   pdftoppm -h
   ```
   如果显示帮助信息，说明安装成功

---

### Step 2: 配置脚本

打开 `batch_process_pdfs.py`，根据需要修改配置：

```python
# 配置部分（在脚本开头）
PDF_SOURCE_DIR = r"F:\BaiduNetdiskDownload\港城大"  # PDF源目录
OUTPUT_DIR = r"F:\BaiduNetdiskDownload\港城大\OCR_Results"  # 输出目录

DPI = 200  # 图像质量（越高越清晰但越慢，推荐150-200）
MAX_PAGES_PER_BATCH = 5  # 每批处理页数（内存不足时减小）
SKIP_EXISTING = True  # 跳过已处理的文件
```

---

### Step 3: 运行脚本

```bash
conda activate deepseek-ocr
python batch_process_pdfs.py
```

---

## 📊 输出说明

### 文件结构

```
F:\BaiduNetdiskDownload\港城大\OCR_Results\
├── processing_log.json              # 处理日志（JSON格式）
├── processing_report.txt            # 处理报告（文本格式）
├── PDF文件名1.md                    # OCR结果1
├── PDF文件名2.md                    # OCR结果2
└── ...
```

### Markdown格式

每个生成的MD文件包含：

```markdown
# 原始PDF文件名

*OCR处理时间: 2024-xx-xx xx:xx:xx*
*原始文件: xxx.pdf*

---

## Page 1

[第1页的OCR内容]

---

## Page 2

[第2页的OCR内容]

---

...
```

---

## ⚙️ 高级配置

### 1. 调整内存使用

如果遇到内存不足，修改以下参数：

```python
DPI = 150  # 降低DPI（默认200）
MAX_PAGES_PER_BATCH = 3  # 减少批处理大小（默认5）
```

### 2. 只处理部分文件

在脚本中添加过滤逻辑：

```python
# 在扫描PDF后添加
pdf_files = [f for f in pdf_files if 'Veterinary' in f]  # 只处理包含特定关键词的
pdf_files = pdf_files[:5]  # 只处理前5个
```

### 3. 测试单个文件

修改配置只处理一个文件：

```python
# 在主循环前添加
pdf_files = [pdf_files[0]]  # 只处理第一个文件
```

### 4. 调整OCR质量

```python
BASE_SIZE = 1280  # 增加以提高质量（默认1024）
IMAGE_SIZE = 768  # 增加以提高质量（默认640）
DPI = 300  # 增加图像DPI（默认200）
```

⚠️ **注意**: 提高质量会显著增加处理时间和内存使用

---

## 🐛 故障排查

### 问题1: ImportError: pdf2image not installed

**解决**:
```bash
pip install pdf2image
```

### 问题2: PDFInfoNotInstalledError / Unable to get page count

**原因**: Poppler未安装或未添加到PATH

**解决**: 按照Step 1.2重新安装Poppler

### 问题3: CUDA out of memory

**原因**: GPU显存不足

**解决方案**:

1. 减少批处理大小:
   ```python
   MAX_PAGES_PER_BATCH = 2
   ```

2. 降低图像质量:
   ```python
   DPI = 150
   BASE_SIZE = 768
   ```

3. 使用CPU模式:
   ```python
   os.environ["CUDA_VISIBLE_DEVICES"] = ''  # 禁用GPU
   ```

### 问题4: 处理速度太慢

**优化建议**:

1. 降低DPI（质量和速度的平衡）:
   ```python
   DPI = 150  # 或更低
   ```

2. 确保使用GPU:
   ```bash
   nvidia-smi  # 检查GPU使用情况
   ```

3. 使用更快的模型配置:
   ```python
   CROP_MODE = False  # 禁用裁剪模式（更快但可能降低质量）
   BASE_SIZE = 768
   ```

### 问题5: 某些页面处理失败

**现象**: 日志显示某些页面错误

**处理**: 
- 脚本会继续处理其他页面
- 失败的页面在MD中标记为 `[OCR处理失败]`
- 查看 `processing_log.json` 了解详情

### 问题6: 中断后如何继续

**解决**: 
- 脚本支持断点续传
- 再次运行脚本会自动跳过已完成的文件
- 如果需要重新处理某个文件，删除 `processing_log.json` 中对应条目

---

## 📈 性能估算

基于典型配置（DPI=200, GPU处理）：

| PDF页数 | 预计时间 | 内存占用 |
|---------|----------|----------|
| 10页    | ~2分钟   | ~2GB     |
| 100页   | ~20分钟  | ~3GB     |
| 500页   | ~1.5小时 | ~4GB     |
| 1000页  | ~3小时   | ~5GB     |

**注意**: 实际时间取决于：
- PDF内容复杂度
- GPU性能
- DPI设置
- 是否使用CROP_MODE

---

## 💡 使用技巧

### 1. 批量处理策略

对于44个大型PDF:

**选项A**: 一次性处理（推荐用于无人值守）
```bash
# 直接运行，脚本会自动处理所有文件
python batch_process_pdfs.py
```

**选项B**: 分批处理
```python
# 修改脚本，每次处理10个
pdf_files = pdf_files[:10]  # 第一批
# 运行后修改为
pdf_files = pdf_files[10:20]  # 第二批
# 以此类推
```

**选项C**: 按大小排序处理
```python
# 按文件大小排序，先处理小文件
pdf_files.sort(key=lambda x: os.path.getsize(x))
```

### 2. 夜间批处理

```bash
# 后台运行（Windows）
start /B python batch_process_pdfs.py > log.txt 2>&1

# 或使用nohup（如果在WSL/Linux）
nohup python batch_process_pdfs.py > log.txt 2>&1 &
```

### 3. 监控进度

```bash
# 实时查看日志
tail -f log.txt

# 或在另一个终端查看输出目录
watch -n 5 'ls -lh F:\BaiduNetdiskDownload\港城大\OCR_Results\*.md'
```

### 4. 检查结果质量

```python
# 快速脚本：检查生成的MD文件大小
import os
import glob

md_files = glob.glob('F:/BaiduNetdiskDownload/港城大/OCR_Results/*.md')
for f in md_files:
    size_kb = os.path.getsize(f) / 1024
    print(f"{os.path.basename(f)}: {size_kb:.1f} KB")
```

---

## 📋 常见PDF类型处理建议

### 文本型PDF（扫描书籍）
```python
DPI = 200  # 标准质量
CROP_MODE = True
```

### 图表密集型PDF
```python
DPI = 250  # 提高质量
BASE_SIZE = 1280
CROP_MODE = True
```

### 混合型PDF（图文混排）
```python
DPI = 200
CROP_MODE = True  # 处理局部细节
```

---

## 🎯 批量处理checklist

开始前检查：

- [ ] 已安装 pdf2image
- [ ] 已安装 Poppler 并添加到PATH
- [ ] 已激活 deepseek-ocr 环境
- [ ] GPU驱动正常（如使用GPU）
- [ ] 磁盘空间充足（建议至少50GB）
- [ ] 已配置正确的源目录和输出目录
- [ ] 已测试单个PDF文件

---

## 📞 支持

如果遇到问题：

1. 检查 `processing_log.json` 查看详细错误
2. 查看 `processing_report.txt` 了解处理统计
3. 参考本文档的故障排查部分
4. 查看终端输出的错误信息

---

## ⏱️ 预计处理时间

44个PDF文件，假设平均每个200页：

- **总页数**: 约 8,800 页
- **预计时间**: 约 25-30 小时（使用GPU）
- **建议**: 分批处理或夜间运行

**优化后**（降低DPI到150）:
- **预计时间**: 约 15-20 小时

---

祝处理顺利！🎉




















