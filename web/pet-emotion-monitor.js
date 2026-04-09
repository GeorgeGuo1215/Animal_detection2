/**
 * 宠物情绪监测模块 - PetEmotionMonitor
 *
 * 基于实时心率(HR)、呼吸率(RR)、活动量(ENMO)和心率变异性(HRV-RMSSD)
 * 绘制趋势折线图，为后续情绪分类提供数据基础。
 *
 * HRV 计算方法：
 *   - 将 HR(bpm) 转换为 NN 间期(ms): NN = 60000 / HR
 *   - 取相邻 NN 间期之差的均方根: RMSSD = sqrt(mean(diff²))
 *   - 这是从平滑后的 HR 估算的近似值，适合趋势监测
 */

class PetEmotionMonitor {
    constructor() {
        this.enabled = false;
        this.species = 'dog'; // 'dog' | 'cat'
        this.maxPoints = 300; // 5 分钟（每秒 1 个点）

        // 时间序列历史: [{label: string, value: number}]
        this.hrHistory   = [];
        this.rrHistory   = [];
        this.enmoHistory = [];
        this.hrvHistory  = [];

        // HRV(RMSSD) 内部状态
        this.nnDiffs  = [];   // 相邻 NN 间期之差 (ms)，滑动窗口 120 秒
        this.lastNN   = null; // 上一个 NN 间期 (ms)
        this.currentHRV = 0;

        // 当前值（供外部读取）
        this.currentHR   = null;
        this.currentRR   = null;
        this.currentENMO = null;

        // Chart.js 图表对象
        this.charts = { hr: null, rr: null, enmo: null, hrv: null };

        this.initialized = false;
    }

    // ── 生命周期 ──────────────────────────────────────────

    /** 初始化图表（需在 DOM 渲染后调用） */
    init() {
        if (this.initialized) return;
        this._initCharts();
        this.initialized = true;
    }

    start() {
        if (!this.initialized) this.init();
        this.enabled = true;
        this._updateStatusUI(true);
    }

    stop() {
        this.enabled = false;
        this._updateStatusUI(false);
    }

    reset() {
        this.hrHistory   = [];
        this.rrHistory   = [];
        this.enmoHistory = [];
        this.hrvHistory  = [];
        this.nnDiffs     = [];
        this.lastNN      = null;
        this.currentHRV  = 0;
        this.currentHR   = null;
        this.currentRR   = null;
        this.currentENMO = null;
        this._updateAllCharts();
        this._updateValueDisplay(null, null, null, null);
    }

    /** 切换物种（猫/狗），更新 Y 轴范围 */
    setSpecies(species) {
        this.species = species;
        if (!this.initialized) return;
        const ranges = this._speciesRanges();
        if (this.charts.hr) {
            this.charts.hr.options.scales.y.min = ranges.hrMin;
            this.charts.hr.options.scales.y.max = ranges.hrMax;
            this.charts.hr.update('none');
        }
        if (this.charts.rr) {
            this.charts.rr.options.scales.y.min = ranges.rrMin;
            this.charts.rr.options.scales.y.max = ranges.rrMax;
            this.charts.rr.update('none');
        }
    }

    // ── 数据入口 ──────────────────────────────────────────

    /**
     * 每次有新生理参数时调用（建议频率：每秒 1 次）
     * @param {number} hr   心率 (bpm)
     * @param {number} rr   呼吸率 (次/分)
     * @param {number} enmo 活动量 ENMO (g)
     * @param {number} [timestamp] Unix 毫秒时间戳，默认 Date.now()
     */
    addVital(hr, rr, enmo, timestamp) {
        if (!this.enabled) return;

        const t     = timestamp || Date.now();
        const label = new Date(t).toLocaleTimeString('zh-CN', {
            hour: '2-digit', minute: '2-digit', second: '2-digit'
        });

        // ── HR ──
        if (hr > 0) {
            this._push(this.hrHistory, label, hr);
            this.currentHR = hr;

            // 计算 NN 间期并累积差值
            const nn = 60000 / hr; // ms
            if (this.lastNN !== null) {
                this.nnDiffs.push(nn - this.lastNN);
                if (this.nnDiffs.length > 120) this.nnDiffs.shift();
                if (this.nnDiffs.length >= 5) {
                    this.currentHRV = this._calcRMSSD();
                    this._push(this.hrvHistory, label, parseFloat(this.currentHRV));
                }
            }
            this.lastNN = nn;
        }

        // ── RR ──
        if (rr > 0) {
            this._push(this.rrHistory, label, rr);
            this.currentRR = rr;
        }

        // ── ENMO ──
        if (enmo !== null && enmo !== undefined && !isNaN(enmo)) {
            const enmoVal = parseFloat(enmo.toFixed(4));
            this._push(this.enmoHistory, label, enmoVal);
            this.currentENMO = enmoVal;
        }

        this._updateAllCharts();
        this._updateValueDisplay(this.currentHR, this.currentRR, this.currentENMO, this.currentHRV);
    }

    // ── 内部：HRV 计算 ────────────────────────────────────

    _calcRMSSD() {
        if (this.nnDiffs.length < 2) return 0;
        const sumSq = this.nnDiffs.reduce((s, d) => s + d * d, 0);
        return Math.sqrt(sumSq / this.nnDiffs.length).toFixed(1);
    }

    _push(arr, label, value) {
        arr.push({ label, value });
        if (arr.length > this.maxPoints) arr.shift();
    }

    // ── 内部：图表 ────────────────────────────────────────

    _speciesRanges() {
        if (this.species === 'cat') {
            return { hrMin: 80, hrMax: 260, rrMin: 5, rrMax: 100 };
        }
        // dog
        return { hrMin: 40, hrMax: 200, rrMin: 5, rrMax: 90 };
    }

    _makeChartConfig(yLabel, color, yMin, yMax) {
        return {
            type: 'line',
            data: {
                labels: [],
                datasets: [{
                    label: yLabel,
                    data: [],
                    borderColor: color,
                    backgroundColor: color + '22',
                    borderWidth: 2,
                    pointRadius: 0,
                    fill: true,
                    tension: 0.3
                }]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                animation: false,
                plugins: {
                    legend: { display: false },
                    tooltip: {
                        callbacks: {
                            label: ctx => `${ctx.parsed.y} ${yLabel.match(/\(([^)]+)\)/)?.[1] || ''}`
                        }
                    }
                },
                scales: {
                    x: {
                        display: true,
                        ticks: { maxTicksLimit: 6, maxRotation: 0, font: { size: 10 } },
                        grid: { color: 'rgba(0,0,0,0.05)' }
                    },
                    y: {
                        display: true,
                        min: yMin,
                        max: yMax,
                        title: { display: true, text: yLabel, font: { size: 11 } },
                        grid: { color: 'rgba(0,0,0,0.05)' }
                    }
                }
            }
        };
    }

    _initCharts() {
        const ranges = this._speciesRanges();

        const configs = [
            { id: 'emotionHRChart',   key: 'hr',   label: '心率 (bpm)',       color: '#e74c3c', yMin: ranges.hrMin, yMax: ranges.hrMax },
            { id: 'emotionRRChart',   key: 'rr',   label: '呼吸率 (次/分)',    color: '#3498db', yMin: ranges.rrMin, yMax: ranges.rrMax },
            { id: 'emotionENMOChart', key: 'enmo', label: 'ENMO (g)',          color: '#2ecc71', yMin: 0,            yMax: 1.5          },
            { id: 'emotionHRVChart',  key: 'hrv',  label: 'HRV-RMSSD (ms)',   color: '#9b59b6', yMin: 0,            yMax: 120          },
        ];

        for (const cfg of configs) {
            const canvas = document.getElementById(cfg.id);
            if (!canvas) { console.warn(`[PetEmotionMonitor] canvas #${cfg.id} 未找到`); continue; }
            this.charts[cfg.key] = new Chart(
                canvas,
                this._makeChartConfig(cfg.label, cfg.color, cfg.yMin, cfg.yMax)
            );
        }
    }

    _updateAllCharts() {
        this._updateChart(this.charts.hr,   this.hrHistory);
        this._updateChart(this.charts.rr,   this.rrHistory);
        this._updateChart(this.charts.enmo, this.enmoHistory);
        this._updateChart(this.charts.hrv,  this.hrvHistory);
    }

    _updateChart(chart, history) {
        if (!chart) return;
        chart.data.labels             = history.map(p => p.label);
        chart.data.datasets[0].data   = history.map(p => p.value);
        chart.update('none');
    }

    _updateValueDisplay(hr, rr, enmo, hrv) {
        const set = (id, text) => {
            const el = document.getElementById(id);
            if (el) el.textContent = text;
        };
        set('emotionCurrentHR',   hr   !== null ? `${hr} bpm`             : '-- bpm');
        set('emotionCurrentRR',   rr   !== null ? `${rr} 次/分`           : '-- 次/分');
        set('emotionCurrentENMO', enmo !== null ? `${enmo.toFixed(3)} g`  : '-- g');
        set('emotionCurrentHRV',  hrv  ? `${hrv} ms`                      : '-- ms');
    }

    _updateStatusUI(running) {
        const statusEl = document.getElementById('petEmotionStatus');
        if (statusEl) {
            statusEl.textContent = running ? '监测中...' : '已停止';
            statusEl.style.color = running ? '#27ae60' : '#7f8c8d';
        }
        const startBtn = document.getElementById('petEmotionStartBtn');
        const stopBtn  = document.getElementById('petEmotionStopBtn');
        if (startBtn) startBtn.style.display = running ? 'none'         : 'inline-block';
        if (stopBtn)  stopBtn.style.display  = running ? 'inline-block' : 'none';
    }
}
