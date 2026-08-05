# 项目更新上下文（Web + BLE + 心率/呼吸）

适用仓库：`GeorgeGuo1215/Animal_detection2`  
本文件目的：下次更新时快速理解“现在是什么状态、为什么这么做、改动在哪、怎么验证与部署”。

## 1. 项目结构（与本次改动相关）

- `web/`：前端静态站点（GitHub Pages 部署的产物目录）
  - `index.html`：UI/脚本入口
  - `app.js`：主逻辑（BLE 接收、图表、统计、新旧心率/呼吸算法调度）
  - `bluetooth.js`：Web Bluetooth 通信与协议适配（FFF0/FFF1/FFF2）
  - `cwt-vitals.js`：新版心率/呼吸算法（参考 MATLAB `calcutePP.m`）
  - `start-server.sh`：本地启动脚本（会写入 `version.js`）
  - `version.js`：本地服务“最新更新时间”注入（由脚本自动覆盖）
- `calcutePP.m`：MATLAB 参考算法（CWT/ICWT + 谐波抑制 + findpeaks）

## 2. BLE 数据协议要点（V1.02）

上报为 CSV：`$MAC,Time,Lon,Lat,Ax,Ay,Az,Gx,Gy,Gz,Roll,Pitch,Yaw,V1,V2,V3*`  
Web 端解析：

- **加速度 Ax/Ay/Az**：单位 g（不再除以 9.81）
- **陀螺仪 Gx/Gy/Gz**：单位 deg/s（不再做 rad→deg 换算）
- **I/Q**：使用字段 `V2,V3`（即 `parts[14], parts[15]`）写入 `bleBufferI/Q`

## 3. 采样率与 50Hz（自动配置 + 监控）

连接后会尝试写指令将设备周期配置为 20ms（目标 50Hz），并在 UI 显示：

- 目标频率、配置状态、实测频率、丢帧等

## 4. 旧算法 vs 新算法（心率/呼吸）

### 4.1 旧算法（仍保留）

页面中的：

- `当前心率`
- `当前呼吸`

由原始算法（含相位解调/滤波/峰值）输出，通常对当前场景较稳。

### 4.2 新算法（已默认启用 CWT）

页面新增：

- `新算法心率`
- `新算法呼吸`

并提供下拉框切换：

- `频域带通(快/近似)`：`freqBP`
- `CWT/ICWT(严格/接近MATLAB)`：`cwt`（默认）

默认运行参数（固定）：

- 滑窗：**8s**
- 周期：**1s**
- 默认算法：**CWT**

### 4.3 新算法关键实现点（对应 MATLAB `calcutePP.m`）

- RR：`0.1–0.5 Hz` 带通 → FFT → `findpeaks`（prominence）
- HR：`0.5–3.0 Hz` 带通 → FFT → 对 `k*RR`（k=2..6）做高斯衰减 → `findpeaks`

Web 端为解决实时场景下的抖动/误峰：

- **RR 提示（rrHint）**：用上一窗口“稳定 RR”作为谐波抑制参考，避免单窗口 RR 误判把 HR 带偏
- **峰值诊断输出**：实时状态会显示 RR/HR 候选峰、以及“谐波抑制用 RR”
- **防抖稳定化**：
  - 中位数滤波（最近 N=5）
  - HR 跳变保护（阈值 25 bpm）
  - RR 跳变保护（阈值 8 bpm）
  - **HR 向下防抖**：轻微干扰导致 HR 短暂偏低时，不立刻下跳；若旧算法也同步下降则快速跟随

## 5. 诊断与验证方法

### 5.1 页面内诊断

- `完整诊断`：打开频谱图（显示原始谱、谐波抑制后谱、谐波位置、HR 峰位置等）
- `新算法运行`状态行：包含
  - raw/stable HR、raw/stable RR
  - RR 候选、HR 候选
  - 谐波抑制实际使用 RR（用于定位“为何突然偏高/偏低”）

### 5.2 本地启动与“更新时间”

进入 `web/` 运行：

```bash
./start-server.sh 8080
```

脚本会：

- 覆盖写入 `web/version.js` 的 `window.__SERVICE_UPDATE_TIME__`
- 启动 `python3 -m http.server`

页面最底部会显示“最新更新服务时间”。

## 6. GitHub Pages 部署（Actions）

工作流：`.github/workflows/deploy.yml`

- push 到 `main` 会自动部署
- 部署内容目录：`./web`

如果需要手动触发，可在 GitHub Actions 页面运行 `workflow_dispatch`。

