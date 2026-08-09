(function () {
  'use strict';

  const finite = value => Number.isFinite(Number(value));
  const setText = (id, value) => {
    const element = document.getElementById(id);
    if (element) element.textContent = value;
  };

  class ImuVitalsPanel {
    constructor(options = {}) {
      this.workerUrl = options.workerUrl || 'imu/dist/imu-vitals.worker.js';
      this.sampleRateHz = options.sampleRateHz || 50;
      this.worker = null;
      this.sessionId = null;
      this.sampleIndex = 0;
      this.trendHistory = [];
      this.candidateHistory = [];
      this.trendChart = null;
      this.candidateChart = null;
      this._initializeCharts();
      this._clearValues();
      this._setState(typeof Worker === 'undefined' ? '浏览器不支持 Worker' : '等待蓝牙连接', 'idle');
    }

    startSession(reason = 'ble_connected') {
      this.reset(reason);
      if (typeof Worker === 'undefined') return;
      try {
        if (!this.worker) {
          this.worker = new Worker(this.workerUrl);
          this.worker.onmessage = event => this._handleMessage(event.data || {});
          this.worker.onerror = event => this._setState(`算法线程错误：${event.message || '未知错误'}`, 'error');
        }
        this.sessionId = `imu-${Date.now()}-${Math.random().toString(16).slice(2)}`;
        this.sampleIndex = 0;
        this.worker.postMessage({
          type: 'init', sessionId: this.sessionId,
          options: { sampleRateHz: this.sampleRateHz, stepSec: 1, replayHistorySec: 70, accelUnit: 'mps2', gyroUnit: 'deg' }
        });
        this._setState('算法初始化中', 'warming');
      } catch (error) {
        this._setState(`IMU 模块不可用：${error.message}`, 'error');
      }
    }

    pushSample(sample) {
      if (!this.worker || !this.sessionId || !sample) return;
      const keys = ['ax', 'ay', 'az', 'gx', 'gy', 'gz'];
      if (!keys.every(key => finite(sample[key]))) return;
      const normalized = { timestamp_s: this.sampleIndex / this.sampleRateHz };
      keys.forEach(key => { normalized[key] = Number(sample[key]); });
      this.sampleIndex += 1;
      try {
        this.worker.postMessage({ type: 'sample', sessionId: this.sessionId, sample: normalized });
      } catch (error) {
        this._setState(`实时送样失败：${error.message}`, 'error');
      }
    }

    reset(reason = 'manual') {
      if (this.worker && this.sessionId) {
        try { this.worker.postMessage({ type: 'reset', sessionId: this.sessionId, reason }); } catch (_) {}
      }
      this.sessionId = null;
      this.sampleIndex = 0;
      this.trendHistory = [];
      this.candidateHistory = [];
      this._clearValues();
      this._refreshCharts();
      this._setState(reason === 'ble_disconnected' ? '蓝牙已断开' : '等待实时数据', 'idle');
    }

    _handleMessage(message) {
      if (message.sessionId !== this.sessionId) return;
      if (message.type === 'ready') this._setState('预热中（首个结果约 10 秒）', 'warming');
      if (message.type === 'prediction') this._renderPrediction(message.result || {});
      if (message.type === 'error') this._setState(`算法错误：${message.message || '未知错误'}`, 'error');
    }

    _renderPrediction(result) {
      const elapsed = Number(result.session_elapsed_s) || 0;
      const hr = result.heart_valid && finite(result.HR_bpm) ? Number(result.HR_bpm) : null;
      const rr = result.respiratory_valid && finite(result.RR_bpm) ? Number(result.RR_bpm) : null;
      setText('imuVitalsHR', hr === null ? '-- bpm' : `${hr.toFixed(1)} bpm`);
      setText('imuVitalsRR', rr === null ? '-- bpm' : `${rr.toFixed(1)} bpm`);
      setText('imuVitalsHRConfidence', `${Math.round((Number(result.HR_confidence) || 0) * 100)}%`);
      setText('imuVitalsRRConfidence', `${Math.round((Number(result.RR_confidence) || 0) * 100)}%`);
      setText('imuVitalsMotion', result.motion_state || 'unknown');
      setText('imuVitalsQuality', result.quality_gate_passed ? '通过' : '拒绝');
      setText('imuVitalsFs', `${Number(result.fs || this.sampleRateHz).toFixed(0)} Hz`);
      setText('imuVitalsRuntime', `${Number(result.runtime_ms || 0).toFixed(0)} ms`);
      setText('imuVitalsWindows', (result.available_windows || []).map(value => `${value}s`).join(' / ') || '--');
      setText('imuVitalsElapsed', `${elapsed.toFixed(0)} s`);
      const reasons = (result.quality_reject_reasons || []).join('、');
      setText('imuVitalsDetail', reasons || (result.fully_warmed_up ? '60 秒长期轨迹已成熟' : `长期轨迹预热 ${Math.round((result.warmup_progress || 0) * 100)}%`));
      if (!result.quality_gate_passed) this._setState('质量门控拒绝', 'rejected');
      else if (!result.fully_warmed_up) this._setState('实时预测中 · 长期轨迹预热', 'warming');
      else this._setState('实时预测中', 'running');

      this.trendHistory.push({ x: elapsed, hr, rr });
      if (this.trendHistory.length > 300) this.trendHistory.shift();
      const candidates = Array.isArray(result.candidates) ? result.candidates.filter(finite).map(Number) : [];
      this.candidateHistory.push({ x: elapsed, values: candidates, hr });
      if (this.candidateHistory.length > 120) this.candidateHistory.shift();
      this._refreshCharts();
    }

    _initializeCharts() {
      if (typeof Chart === 'undefined') return;
      const common = { parsing: false, animation: false, responsive: true, maintainAspectRatio: false };
      const trend = document.getElementById('imuVitalsTrendChart');
      if (trend) this.trendChart = new Chart(trend, {
        type: 'line',
        data: { datasets: [
          { label: 'IMU HR', data: [], borderColor: '#2d7fb8', yAxisID: 'hr', spanGaps: false, pointRadius: 0, borderWidth: 2 },
          { label: 'IMU RR', data: [], borderColor: '#d8843b', yAxisID: 'rr', spanGaps: false, pointRadius: 0, borderWidth: 2 }
        ] },
        options: Object.assign({}, common, { scales: {
          x: { type: 'linear', title: { display: true, text: '会话时间 (s)' } },
          hr: { position: 'left', min: 30, max: 220, title: { display: true, text: 'HR (bpm)' } },
          rr: { position: 'right', min: 10, max: 30, grid: { drawOnChartArea: false }, title: { display: true, text: 'RR (bpm)' } }
        } })
      });
      const candidates = document.getElementById('imuVitalsCandidatesChart');
      if (candidates) this.candidateChart = new Chart(candidates, {
        type: 'scatter',
        data: { datasets: [
          { label: '候选峰', data: [], pointRadius: 2, backgroundColor: 'rgba(95,95,95,.55)' },
          { label: '最终 IMU HR', data: [], type: 'line', borderColor: '#2d7fb8', pointRadius: 0, borderWidth: 2, showLine: true }
        ] },
        options: Object.assign({}, common, { scales: {
          x: { type: 'linear', title: { display: true, text: '最近 120 秒' } },
          y: { min: 30, max: 220, title: { display: true, text: '心率候选 (bpm)' } }
        } })
      });
    }

    _refreshCharts() {
      if (this.trendChart) {
        this.trendChart.data.datasets[0].data = this.trendHistory.map(item => ({ x: item.x, y: item.hr }));
        this.trendChart.data.datasets[1].data = this.trendHistory.map(item => ({ x: item.x, y: item.rr }));
        this.trendChart.update('none');
      }
      if (this.candidateChart) {
        const candidates = [], final = [];
        this.candidateHistory.forEach(item => {
          item.values.forEach(value => candidates.push({ x: item.x, y: value }));
          final.push({ x: item.x, y: item.hr });
        });
        this.candidateChart.data.datasets[0].data = candidates;
        this.candidateChart.data.datasets[1].data = final;
        this.candidateChart.update('none');
      }
    }

    _clearValues() {
      ['imuVitalsHR', 'imuVitalsRR'].forEach(id => setText(id, '-- bpm'));
      ['imuVitalsHRConfidence', 'imuVitalsRRConfidence'].forEach(id => setText(id, '--%'));
      ['imuVitalsMotion', 'imuVitalsQuality', 'imuVitalsFs', 'imuVitalsRuntime', 'imuVitalsWindows', 'imuVitalsElapsed'].forEach(id => setText(id, '--'));
      setText('imuVitalsDetail', '仅使用当前连接后的 BLE 实时样本，不读取补传与金标准数据');
    }

    _setState(label, state) {
      setText('imuVitalsStatus', label);
      const badge = document.getElementById('imuVitalsStatus');
      if (badge) badge.dataset.state = state;
    }
  }

  window.ImuVitalsPanel = ImuVitalsPanel;
})();
