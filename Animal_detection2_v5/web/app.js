/**
 * 毫米波雷达数据处理Web应用主控制器
 */

class RadarWebApp {
    constructor() {
        if (typeof RadarDataProcessor === 'undefined') {
            throw new Error('RadarDataProcessor 未加载，请检查 radar-processor.js 是否成功加载。');
        }
        // 默认优先按 50Hz 处理；若 BLE 实际达不到，会在连接后自动降级为当前实际频率
        const samplingRate = 50;
        this.processor = new RadarDataProcessor(samplingRate);
        // Offline files may carry a different timestamp-derived rate. Keeping a
        // separate processor prevents opening a 50 Hz file from overwriting a
        // verified 100 Hz live BLE time base (or vice versa).
        this.offlineProcessor = new RadarDataProcessor(samplingRate);
        this.offlineSamplingRate = samplingRate;
        this.selectedFiles = [];
        this.processedResults = [];
        this.charts = {}; // 文件数据图表
        
        // 蓝牙数据相关
        this.bleConnected = false;
        this.bleCharts = {}; // 蓝牙数据图表
        this.bleBackfillCharts = {}; // 蓝牙补传图表
        this.bleBufferI = [];
        this.bleBufferQ = [];
        this.bleBackfillBuffers = this._createEmptyBleBackfillBuffers();
        this.bleBackfillState = this._createBleBackfillState();
        this.bleBackfillMaxBuffer = 3000;
        this.bleBackfillMaxBufferHard = this.bleBackfillMaxBuffer + 200;

        // 图表可用性（Chart.js 可能因网络/CDN加载失败）
        this._chartsEnabled = true;

        // 自适应Y轴相关属性
        this.adaptiveYAxisEnabled = true; // 启用自适应Y轴以放大显示微小变化
        this.adaptiveSampleCount = 0; // 已收集的样本数量
        this.adaptiveStabilizeThreshold = 30; // 稳定前需要的样本数（降低阈值以更快响应）
        this.adaptiveStabilizeWindow = 50; // 检测稳定的窗口大小
        this.adaptiveLastMinI = Infinity;
        this.adaptiveLastMaxI = -Infinity;
        this.adaptiveLastMinQ = Infinity;
        this.adaptiveLastMaxQ = -Infinity;
        this.adaptiveStabilized = false; // 是否已稳定
        // IMU(默认存陀螺仪)三轴缓存：gx/gy/gz
        this.bleBufferIMU_X = [];
        this.bleBufferIMU_Y = [];
        this.bleBufferIMU_Z = [];
        // 加速度计三轴缓存：ax/ay/az
        this.bleBufferACC_X = [];
        this.bleBufferACC_Y = [];
        this.bleBufferACC_Z = [];
        this.bleBufferTemperature = []; // 温度数据缓存
        this.bleBufferTimestamps = [];
        // Keep the device clock, browser arrival clock and BLE notification id
        // separately.  A single notification may carry a variable number of
        // frames, so no one of these clocks is sufficient on its own.
        this.bleBufferDeviceTimestamps = [];
        this.bleBufferArrivalTimestamps = [];
        this.bleBufferNotificationIds = [];
        this.vitalsSamplingEpoch = 0;
        this.bleMaxBuffer = 5000; // 逻辑窗口长度
        // 避免每条数据都 splice(0,1) 造成 O(n) 内存搬移：允许轻微超出，超出后一次性裁剪
        this.bleMaxBufferHard = this.bleMaxBuffer + 200;
        this.blePendingFloat = null; // 仅有单个浮点时用于配对
        this.bleDataCount = 0;
        this.bleConnectStartTime = null;
        this.bleConnectTimer = null;
        this.lastBleRxTs = 0;
        this.rxWatchdogTimer = null;
        this.bleRxStallWarned = false;
        this.allowBackfillOnNextConnect = false;
        this.lastLiveDeviceTsMs = null;
        this.pendingBackfillFromTsMs = null;
        this._simInterval = null;

        // Legacy field names are retained for callers, but no fabricated 70/18
        // observations are inserted. Canonical smoothing is handled by
        // VitalsTracker using valid, confidence-qualified measurements only.
        this.heartRateHistory = [];
        this.respiratoryHistory = [];
        this.historyIndex = 0;
        this.historyMaxLength = 3;
        this.heartRateDelta = 25;
        this.lastStableHeartRate = null;
        this.lastStableRespRate = null;

        // ===== 丢包/采样率统计（估算）=====
        // 说明：若设备每条数据=1个采样点，则可根据到达间隔估算丢包；
        // 若未来协议携带 seq，则可切换为 seq 更精准统计。
        this.bleStats = {
            startRxTs: 0,
            lastRxTs: 0,
            startRxMonoTs: 0,
            lastRxMonoTs: 0,
            received: 0,
            expected: 0,
            missed: 0,
            lastGapMs: 0,
            gapEmaMs: 0,
            gapJitterEmaMs: 0,
            lastSeq: null,
            seqBased: false
        };

        this.bleTargetFs = 50;
        this.bleConfigStatus = 'pending'; // pending | configuring | configured | fallback | failed
        this.bleProtocol = '未连接';
        this._lastGpsTs = null;
        this._gpsGapMissed = 0;

        // ===== 性能优化：日志/图表节流 =====
        this._bleLogLines = [];
        this._bleLogRenderTimer = null;
        this._bleRawLines = [];
        this._bleRawRenderTimer = null;
        this._bleBackfillLogLines = [];
        this._bleBackfillLogRenderTimer = null;

        this._bleChartRaf = null;
        this._bleChartLastUpdateTs = 0;
        this._bleChartMinIntervalMs = 150; // ~7Hz 刷新，平衡流畅度与性能

        this._bleVitalLogLastTs = 0; // 限制生理参数日志刷屏
        this._lastVitalUpdateTime = 0; // 上次生理参数更新时间（用于节流）

        // ===== 新版 CWT/频域带通心率呼吸率算法 =====
        this.vitalsMethod = 'cwt';
        this.vitalsExtractor = null;
        this.vitalsTimer = null;
        this.vitalsWindowSec = 20;
        this.vitalsPeriodSec = 1;
        this.vitalsSpectrumChart = null;
        this.lastVitalsResult = null;

        // 全局禁用 Chart.js 动画（避免实时曲线渲染卡顿）
        if (typeof Chart !== 'undefined') {
            Chart.defaults.animation = false;
            Chart.defaults.transitions = {};
        }

        // 实时保存相关 (参考main.py)
        this.bleRecordingFlag = 0;  // 0: 不记录, 1: 记录中
        this.bleRecordingData = []; // 记录的处理后数据缓存
        this.bleRecordingRawData = []; // 记录的原始蓝牙数据缓存
        this.bleRecordingStartTime = null;

        // ===== BLE 上报到 Integration =====
        this.bleUploadEnabled = false;
        this.bleUploadIntervalSec = 10;
        this.bleUploadWindowSec = 10;
        this.bleUploadTimer = null;
        this.bleLastUploadTs = 0;
        
        // 当前心率和呼吸率（供静息监测模块使用）
        this.currentHeartRate = null;
        this.currentRespiratoryRate = null;

        // ===== 新版 CWT/频域带通心率呼吸率算法 =====
        this.vitalsMethod = 'cwt';
        this.vitalsWindowSec = 20;
        this.vitalsPeriodSec = 1;
        this.vitalsExtractor = null;
        this.vitalsTimer = null;
        this.vitalsSpectrumChart = null;
        this.vitalsDiagnosticOpen = false;
        this.lastVitalsResult = null;
        this.vitalsFirstResultLogged = false;
        this.vitalsHrHistory = [];
        this.vitalsRrHistory = [];
        this.vitalsSmoothN = 3;
        this.vitalsLastStableHR = null;
        this.vitalsLastStableRR = null;
        this.vitalsHrDropGuardBpm = 25;
        this.vitalsWarmupMinCycles = 2;
        this.vitalsAcceptedCycles = 0;
        this.vitalsMinConfidence = 0.28;
        this.vitalsGuardWindowSec = 30;
        this.vitalsGuardMaxAgeMs = 2500;
        this.vitalsLastGuardResult = null;
        this.vitalsLastGuardTs = 0;
        this.vitalsGuardDiffBpm = 12;
        // Continuous-monitoring policy: motion must block untrustworthy new
        // estimates, but a short collar movement must not erase the last
        // trustworthy HR/RR from the display. Freshness is tracked separately
        // so held values cannot masquerade as new measurements.
        this.vitalsHoldMaxMs = 30000;
        this.vitalsLastReliable = {
            HR: { value: NaN, timestampMs: NaN, confidence: 0 },
            RR: { value: NaN, timestampMs: NaN, confidence: 0 }
        };
        this.vitalsDisplayState = null;
        this.vitalsMotionConfig = {
            blockSec: 0.5,
            gravityTauSec: 0.8,
            minValidFraction: 0.80,
            minNoiseSigmaG: 0.003,
            cleanAbsMarginG: 0.015,
            strongAbsMarginG: 0.080,
            artifactPadBeforeSec: 0.25,
            artifactPadAfterSec: 0.50,
            recentSec: 2,
            hrMinCleanSec: 8,
            rrMinCleanSec: 14,
            hrMinCleanFraction: 0.40,
            rrMinCleanFraction: 0.60
        };
        this._vitalsMotionBaseline = null;
        this.vitalsTracker = typeof VitalsTracker !== 'undefined'
            ? new VitalsTracker({
                minConfidence: this.vitalsMinConfidence,
                warmupWindows: 2,
                smoothWindows: 3,
                jumpConfirmWindows: 3,
                invalidExpireWindows: 5,
                holdMaxMs: this.vitalsHoldMaxMs,
                hrJumpThreshold: 25,
                rrJumpThreshold: 8,
                warmupMinSpanMs: 2000,
                jumpConfirmMinSpanMs: 4000,
                pendingJumpMaxMisses: 2,
                requireGuardForLowHrBelowBpm: 65
            })
            : null;

        // ===== 活动量与步数监测模块 =====
        this.activityMonitor = null;
        this.activityMonitorEnabled = false;

        // ===== 睡眠质量监测模块 =====
        this.sleepMonitor = null;
        this.sleepMonitorEnabled = false;

        // ===== GPS 定位追踪模块 =====
        this.gpsLocationMonitor = null;
        this.gpsLocationMonitorEnabled = false;

        // ===== 姿态解算模块 =====
        this.attitudeSolver = null;
        this.attitudeVisualizer = null;
        this.attitudeEnabled = false;

        this.initializeEventListeners();
        this.initBleUploadConfig();
        this.initBlePetProfile();
        this.initializeCharts();
        this.initializeBluetoothCharts();
        this.initializeBleBackfillCharts();
        this.initializeBLEECG();
        this.initializeFileECG();
        this.updateBleBackfillUI();

        // 初始化BLE事件
        this.initializeBLE();
        
        // 测试FFT是否正常工作
        this.testFFT();

        // 不再启用自动断连看门狗：仅允许用户手动断开蓝牙
    }

    /**
     * 初始化 Agent Endpoint（构造函数调用）
     */
    /**
     * 测试FFT功能
     */
    testFFT() {
        try {
            console.log('🔍 测试FFT功能...');
            
            if (typeof FFT === 'undefined') {
                console.error('❌ FFT对象未定义！');
                return;
            }
            
            // 创建测试信号: 10Hz + 30Hz 正弦波
            const testData = [];
            const fs = (this.processor && Number.isFinite(this.processor.fs)) ? this.processor.fs : 100;
            for (let i = 0; i < 256; i++) {
                const t = i / fs;
                const signal = Math.sin(2 * Math.PI * 10 * t) + 0.5 * Math.sin(2 * Math.PI * 30 * t);
                testData.push([signal, 0]); // 复数格式
            }
            
            const fftResult = FFT.fft(testData);
            const magnitude = fftResult.map(([real, imag]) => Math.sqrt(real * real + imag * imag));
            
            // 找到峰值
            const peakIdx1 = magnitude.slice(0, 128).indexOf(Math.max(...magnitude.slice(0, 128)));
            const peakIdx2 = magnitude.slice(peakIdx1 + 5, 128).indexOf(Math.max(...magnitude.slice(peakIdx1 + 5, 128))) + peakIdx1 + 5;
            
            const freq1 = peakIdx1 * fs / 256;
            const freq2 = peakIdx2 * fs / 256;
            
            console.log(`✅ FFT测试成功！检测到峰值频率: ${freq1.toFixed(1)}Hz, ${freq2.toFixed(1)}Hz (期望: 10Hz, 30Hz)`);
            
        } catch (error) {
            console.error('❌ FFT测试失败:', error);
        }
    }

    /**
     * 初始化事件监听器
     */
    initializeEventListeners() {
        // 文件上传相关
        const fileInput = document.getElementById('fileInput');
        const uploadArea = document.getElementById('uploadArea');

        if (fileInput) {
            fileInput.addEventListener('change', (e) => this.handleFileSelect(e));
        } else {
            console.warn('未找到 fileInput，文件上传功能将不可用。');
        }
        
        // 拖拽上传
        if (uploadArea) {
            uploadArea.addEventListener('dragover', (e) => {
                e.preventDefault();
                uploadArea.classList.add('dragover');
            });

            uploadArea.addEventListener('dragleave', (e) => {
                e.preventDefault();
                uploadArea.classList.remove('dragover');
            });

            uploadArea.addEventListener('drop', (e) => {
                e.preventDefault();
                uploadArea.classList.remove('dragover');
                this.handleFileSelect({ target: { files: e.dataTransfer.files } });
            });
        } else {
            console.warn('未找到 uploadArea，拖拽上传功能将不可用。');
        }

        // 设置面板
        const settingsToggle = document.querySelector('.settings-toggle');
        if (settingsToggle) {
            settingsToggle.addEventListener('click', () => this.toggleSettings());
        } else {
            console.warn('未找到 settings-toggle，设置面板按钮将不可用。');
        }
    }

    initBlePetProfile() {
        const fields = [
            ['blePetName', 'blePetName', ''],
            ['blePetSpecies', 'blePetSpecies', 'dog'],
            ['blePetBreed', 'blePetBreed', ''],
            ['blePetAgeYears', 'blePetAgeYears', '0'],
            ['blePetAgeMonths', 'blePetAgeMonths', '0'],
            ['blePetSex', 'blePetSex', 'unknown'],
            ['blePetWeight', 'blePetWeight', ''],
            ['blePetCondition', 'blePetCondition', '']
        ];

        fields.forEach(([id, key, fallback]) => {
            const el = document.getElementById(id);
            if (!el) return;
            const saved = localStorage.getItem(key);
            if (saved !== null) el.value = saved;
            else if (fallback !== '') el.value = fallback;
            el.addEventListener('change', () => this._saveBlePetProfile());
            el.addEventListener('blur', () => this._saveBlePetProfile());
        });
    }

    _getBlePetProfile() {
        const getVal = (id) => {
            const el = document.getElementById(id);
            return el ? el.value.trim() : '';
        };
        const years = parseInt(getVal('blePetAgeYears'), 10);
        const months = parseInt(getVal('blePetAgeMonths'), 10);
        const weight = parseFloat(getVal('blePetWeight'));
        const uploadCfg = this._getBleUploadConfig();

        return {
            animal_id: uploadCfg.animalId || 'unknown',
            name: getVal('blePetName') || 'unknown',
            species: getVal('blePetSpecies') || 'other',
            breed: getVal('blePetBreed') || 'unknown',
            sex: getVal('blePetSex') || 'unknown',
            age_years: Number.isFinite(years) && years >= 0 ? years : 0,
            age_months: (Number.isFinite(years) && years >= 0 ? years : 0) * 12
                + (Number.isFinite(months) && months >= 0 ? Math.min(months, 11) : 0),
            weight_kg: Number.isFinite(weight) && weight >= 0 ? weight : 0,
            medical_condition: getVal('blePetCondition') || ''
        };
    }

    _saveBlePetProfile() {
        const profile = this._getBlePetProfile();
        localStorage.setItem('blePetName', profile.name === 'unknown' ? '' : profile.name);
        localStorage.setItem('blePetSpecies', profile.species);
        localStorage.setItem('blePetBreed', profile.breed === 'unknown' ? '' : profile.breed);
        localStorage.setItem('blePetAgeYears', String(Math.floor(profile.age_months / 12)));
        localStorage.setItem('blePetAgeMonths', String(profile.age_months % 12));
        localStorage.setItem('blePetSex', profile.sex);
        localStorage.setItem('blePetWeight', profile.weight_kg > 0 ? String(profile.weight_kg) : '');
        localStorage.setItem('blePetCondition', profile.medical_condition);
    }

    _getLatestImuSnapshot() {
        const len = this.bleBufferI.length;
        if (len === 0) return null;

        const i = len - 1;
        const snapshot = {
            acc: {
                x: Number(this.bleBufferACC_X[i] || 0),
                y: Number(this.bleBufferACC_Y[i] || 0),
                z: Number(this.bleBufferACC_Z[i] || 0)
            },
            gyr: {
                x: Number(this.bleBufferIMU_X[i] || 0),
                y: Number(this.bleBufferIMU_Y[i] || 0),
                z: Number(this.bleBufferIMU_Z[i] || 0)
            },
            temperature: this.bleBufferTemperature[i]
        };

        if (this.attitudeSolver) {
            const euler = this.attitudeSolver.getEulerAngles();
            const quat = this.attitudeSolver.getQuaternion();
            const angVel = this.attitudeSolver.getAngularVelocity();
            snapshot.attitude = {
                roll: Number(euler?.roll || 0),
                pitch: Number(euler?.pitch || 0),
                yaw: Number(euler?.yaw || 0),
                quaternion: {
                    w: Number(quat?.w || 1),
                    x: Number(quat?.x || 0),
                    y: Number(quat?.y || 0),
                    z: Number(quat?.z || 0)
                },
                angular_velocity: {
                    x: Number(angVel?.x || 0),
                    y: Number(angVel?.y || 0),
                    z: Number(angVel?.z || 0)
                }
            };
        }

        return snapshot;
    }

    _getBleVitalsSnapshot() {
        return {
            heartRate: Number.isFinite(this.currentHeartRate) ? Number(this.currentHeartRate) : null,
            respiratoryRate: Number.isFinite(this.currentRespiratoryRate) ? Number(this.currentRespiratoryRate) : null,
            heartRateStatus: this.vitalsDisplayState?.HR?.status || 'warming',
            respiratoryRateStatus: this.vitalsDisplayState?.RR?.status || 'warming',
            heartRateAgeMs: this.vitalsDisplayState?.HR?.ageMs ?? null,
            respiratoryRateAgeMs: this.vitalsDisplayState?.RR?.ageMs ?? null,
            motionState: this.vitalsDisplayState?.motion?.state || 'unknown'
        };
    }

    _appendBleRecordingMetadataComments(profile, vitals, imu, phase = 'start') {
        const sexLabel = profile.sex === 'male' ? '公' : profile.sex === 'female' ? '母' : '未知';
        const speciesLabel = profile.species === 'dog' ? '狗' : profile.species === 'cat' ? '猫' : '其他';
        const ageText = `${Math.floor(profile.age_months / 12)}岁${profile.age_months % 12}个月`;

        this.bleRecordingData.push(`# 宠物名称: ${profile.name}`);
        this.bleRecordingData.push(`# 宠物物种: ${speciesLabel} (${profile.species})`);
        this.bleRecordingData.push(`# 宠物品种: ${profile.breed}`);
        this.bleRecordingData.push(`# 宠物年龄: ${ageText} (${profile.age_months} months)`);
        this.bleRecordingData.push(`# 宠物性别: ${sexLabel}`);
        this.bleRecordingData.push(`# 宠物体重: ${profile.weight_kg} kg`);
        this.bleRecordingData.push(`# 病情/备注: ${profile.medical_condition || '无'}`);
        this.bleRecordingData.push(`# animal_id: ${profile.animal_id}`);

        const hr = vitals.heartRate ?? '--';
        const rr = vitals.respiratoryRate ?? '--';
        this.bleRecordingData.push(`# ${phase === 'start' ? '开始' : '结束'}时心率: ${hr} bpm, 呼吸率: ${rr} bpm`);

        if (imu) {
            this.bleRecordingData.push(
                `# ${phase === 'start' ? '开始' : '结束'}时IMU Acc(g): ${imu.acc.x.toFixed(4)}, ${imu.acc.y.toFixed(4)}, ${imu.acc.z.toFixed(4)}`
            );
            this.bleRecordingData.push(
                `# ${phase === 'start' ? '开始' : '结束'}时IMU Gyr(deg/s): ${imu.gyr.x.toFixed(3)}, ${imu.gyr.y.toFixed(3)}, ${imu.gyr.z.toFixed(3)}`
            );
            if (imu.attitude) {
                this.bleRecordingData.push(
                    `# ${phase === 'start' ? '开始' : '结束'}时姿态 Roll/Pitch/Yaw(deg): ${imu.attitude.roll.toFixed(2)}, ${imu.attitude.pitch.toFixed(2)}, ${imu.attitude.yaw.toFixed(2)}`
                );
            }
            if (imu.temperature !== null && imu.temperature !== undefined) {
                this.bleRecordingData.push(`# ${phase === 'start' ? '开始' : '结束'}时温度: ${Number(imu.temperature).toFixed(2)} °C`);
            }
        }
    }

    _buildBleRecordingSessionMeta({ profile, startTime, endTime, durationSeconds, startVitals, endVitals, startImu, endImu, dataPoints }) {
        const uploadCfg = this._getBleUploadConfig();
        return {
            pet: profile,
            device: {
                device_id: uploadCfg.deviceId || 'unknown',
                protocol: this.bleProtocol || '未连接'
            },
            recording: {
                startTime: startTime.toISOString(),
                endTime: endTime.toISOString(),
                durationSeconds
            },
            vitals: {
                start: startVitals,
                end: endVitals
            },
            imu: {
                start: startImu,
                end: endImu
            },
            dataPoints,
            algorithm: this.buildBleDiagnostics(),
            windowStatistics: this._buildBluetoothSessionStats(),
            note: '宠物档案、HR/RR窗口轨迹、候选峰、采样时钟与IMU快照随录制一并保存'
        };
    }

    initBleUploadConfig() {
        const urlEl = document.getElementById('bleUploadUrl');
        const animalEl = document.getElementById('bleAnimalId');
        const deviceEl = document.getElementById('bleDeviceId');
        const intervalEl = document.getElementById('bleUploadInterval');

        if (urlEl) {
            urlEl.value = localStorage.getItem('bleUploadUrl') || 'http://127.0.0.1:9001/ingest';
        }
        if (animalEl) {
            animalEl.value = localStorage.getItem('bleAnimalId') || '';
        }
        if (deviceEl) {
            deviceEl.value = localStorage.getItem('bleDeviceId') || '';
        }
        if (intervalEl) {
            intervalEl.value = localStorage.getItem('bleUploadInterval') || String(this.bleUploadIntervalSec);
        }
    }


    /**
     * 初始化 BLE 事件
     */
    initializeBLE() {
        if (!window.BLE) return;
        BLE.onConnect = (device) => {
            this.bleConnected = true;
            this.bleRxStallWarned = false;
            this.bleBackfillState.enabled = !!this.allowBackfillOnNextConnect;
            this.bleBackfillState.fromDeviceTsMs = this.pendingBackfillFromTsMs;
            this.allowBackfillOnNextConnect = false;
            this.pendingBackfillFromTsMs = null;
            this.bleProtocol = this._getBleProtocolLabel();
            this.addBLELog(`✓ 已连接: ${device.name || '未知设备'} (${device.id})`);
            this.addBLELog(`🧭 当前通信协议: ${this.bleProtocol}`);
            if (this.bleBackfillState.enabled) {
                this.addBLELog('🕘 本次连接已启用补传识别（由上次手动断开触发）');
            }
            this._updateBleProtocolUI();

            try {
                if (typeof updateBLESavedDeviceUI === 'function') {
                    updateBLESavedDeviceUI();
                }
            } catch (e) {
                console.warn('updateBLESavedDeviceUI 异常(已忽略):', e.message);
            }

            const rtData = document.getElementById('bleRealTimeData');
            if (rtData) rtData.style.display = 'block';

            this.bleConnectStartTime = Date.now();
            this.startBluetoothTimer();
            this.resetBluetoothData();

            try {
                this.updateBLEButtons();
            } catch(e) {
                console.error('updateBLEButtons error:', e);
            }

            // 连接后自动展开蓝牙图表（避免用户觉得“没有gx/gy/gz可视图”）
            const chartsSection = document.getElementById('bluetoothChartsSection');
            if (chartsSection) {
                chartsSection.style.display = 'block';
            }
            try {
                if (!this.bleCharts.iSignal || !this.bleCharts.qSignal) {
                    this.initializeBluetoothCharts();
                }
                if (!this.bleBackfillCharts.iq || !this.bleBackfillCharts.lag) {
                    this.initializeBleBackfillCharts();
                }
            } catch (e) { console.warn('BLE图表初始化异常:', e.message); }

            setTimeout(() => {
                try {
                    Object.values(this.bleCharts || {}).forEach(ch => {
                        if (ch && typeof ch.resize === 'function') ch.resize();
                        if (ch && typeof ch.update === 'function') ch.update('none');
                    });
                } catch (error) {
                    console.warn('图表刷新异常:', error.message);
                }
            }, 100);

            try { this.initializeBLEECG(); } catch (e) { console.warn('BLE ECG初始化异常:', e.message); }
            setTimeout(() => { this.forceReinitializeCharts(); }, 200);

            const playBtn = document.getElementById('blePlayBtn');
            const pauseBtn = document.getElementById('blePauseBtn');
            if (this._bleECG) {
                this._bleECG.res.playing = true;
                this._bleECG.hb.playing = true;
                if (playBtn && pauseBtn) { playBtn.style.display = 'none'; pauseBtn.style.display = 'inline-block'; }
                if (!this._bleECG.raf) this._bleECG.draw();
            }

            this.bleConfigStatus = 'configuring';
            this._updateConfigStatusUI();
            this._autoConfigTcycleWithRetry();
            this.startVitalsRealtime();
        };
        BLE.onDisconnect = () => {
            this.bleConnected = false;
            this.bleRxStallWarned = false;
            this.bleProtocol = '未连接';
            this.addBLELog('⚠️ 已断开连接');
            this._updateBleProtocolUI();
            this.stopVitalsRealtime();
            if (this.bleBackfillState.backfillCount > 0) {
                this.addBleBackfillLog('🔌 蓝牙已断开，本次补传会话结束');
            }
            
            // 隐藏实时数据区域
            document.getElementById('bleRealTimeData').style.display = 'none';
            
            // 停止计时
            this.stopBluetoothTimer();
            // 停止任何模拟数据
            this.stopSimulation();

            // 断开后停止上报
            this.stopBleUpload();
            
            this.updateBLEButtons();
        };
        BLE.onError = (err) => {
            this.addBLELog(`❌ 错误: ${err.message}`);
        };
        BLE.onServiceDiscovered = (info) => {
            this.addBLELog(info);
            this.bleProtocol = this._getBleProtocolLabel();
            this._updateBleProtocolUI();
        };
        BLE.onLine = (line, transportMeta) => this.handleBLELine(line, transportMeta);
        this._updateBleProtocolUI();
        this.updateBLEButtons();
    }

    updateBLEButtons() {
        const c = document.getElementById('bleConnectBtn');
        const stopBtn = document.getElementById('bleStopRecordBtn');
        if (!c) return;
        c.style.display = this.bleConnected ? 'none' : 'inline-block';
        if (stopBtn) {
            stopBtn.style.display = this.bleRecordingFlag === 1 ? 'inline-block' : 'none';
        }
    }

    _setBleUploadStatus(text) {
        const statusEl = document.getElementById('bleUploadStatus');
        if (statusEl) statusEl.textContent = text;
    }

    _getBleUploadConfig() {
        const urlEl = document.getElementById('bleUploadUrl');
        const animalEl = document.getElementById('bleAnimalId');
        const deviceEl = document.getElementById('bleDeviceId');
        const intervalEl = document.getElementById('bleUploadInterval');

        const url = urlEl ? urlEl.value.trim() : '';
        const animalId = animalEl ? animalEl.value.trim() : '';
        const deviceId = deviceEl ? deviceEl.value.trim() : '';
        const intervalSec = intervalEl ? parseInt(intervalEl.value, 10) : this.bleUploadIntervalSec;

        return {
            url,
            animalId,
            deviceId,
            intervalSec: Number.isFinite(intervalSec) && intervalSec > 0 ? intervalSec : this.bleUploadIntervalSec
        };
    }

    startBleUpload() {
        if (!this.bleConnected) {
            alert('请先连接蓝牙设备');
            return;
        }
        const cfg = this._getBleUploadConfig();
        if (!cfg.url) {
            alert('请填写上报接口地址');
            return;
        }
        if (!cfg.animalId) {
            alert('请填写 animal_id');
            return;
        }
        if (!cfg.deviceId) {
            alert('请填写 device_id');
            return;
        }

        localStorage.setItem('bleUploadUrl', cfg.url);
        localStorage.setItem('bleAnimalId', cfg.animalId);
        localStorage.setItem('bleDeviceId', cfg.deviceId);
        localStorage.setItem('bleUploadInterval', String(cfg.intervalSec));

        this.bleUploadEnabled = true;
        this.bleUploadIntervalSec = cfg.intervalSec;
        this._setBleUploadStatus('上传中');
        this.updateBLEButtons();

        if (this.bleUploadTimer) clearInterval(this.bleUploadTimer);
        this._sendBleUploadOnce();
        this.bleUploadTimer = setInterval(() => this._sendBleUploadOnce(), this.bleUploadIntervalSec * 1000);
    }

    stopBleUpload() {
        this.bleUploadEnabled = false;
        if (this.bleUploadTimer) {
            clearInterval(this.bleUploadTimer);
            this.bleUploadTimer = null;
        }
        this._setBleUploadStatus('未上传');
        this.updateBLEButtons();
    }

    _toEpochMs(ts) {
        if (Number.isFinite(ts)) return Number(ts);
        if (typeof ts === 'string') {
            const parsed = Date.parse(ts);
            if (!Number.isNaN(parsed)) return parsed;
        }
        return Date.now();
    }

    _formatTimezoneOffset() {
        const offsetMin = -new Date().getTimezoneOffset();
        const sign = offsetMin >= 0 ? '+' : '-';
        const abs = Math.abs(offsetMin);
        const hh = String(Math.floor(abs / 60)).padStart(2, '0');
        const mm = String(abs % 60).padStart(2, '0');
        return `${sign}${hh}:${mm}`;
    }

    _buildBleEventPayload() {
        const cfg = this._getBleUploadConfig();
        const pet = this._getBlePetProfile();
        const fs = (this.processor && Number.isFinite(this.processor.fs)) ? this.processor.fs : 50;
        const len = this.bleBufferI.length;
        if (len < Math.max(10, fs * 2)) {
            this.addBLELog('⚠️ 上报跳过：数据点不足');
            return null;
        }

        const windowSize = Math.min(len, Math.max(10, fs * this.bleUploadWindowSec));
        const startIndex = len - windowSize;
        const endIndex = len - 1;

        const startTsMs = this._toEpochMs(this.bleBufferTimestamps[startIndex]);
        const endTsMs = this._toEpochMs(this.bleBufferTimestamps[endIndex]);
        const timezone = this._formatTimezoneOffset();

        const accelSamples = [];
        const tempSamples = [];
        let lastTempSecond = -1;
        for (let i = startIndex; i <= endIndex; i++) {
            const tMs = Math.round(((i - startIndex) / fs) * 1000);
            const tS = Math.floor((i - startIndex) / fs);
            accelSamples.push({
                t_ms: tMs,
                x: Number(this.bleBufferIMU_X[i] || 0),
                y: Number(this.bleBufferIMU_Y[i] || 0),
                z: Number(this.bleBufferIMU_Z[i] || 0)
            });
            if (tS !== lastTempSecond) {
                tempSamples.push({
                    t_s: tS,
                    value: Number(this.bleBufferTemperature[i] || 0)
                });
                lastTempSecond = tS;
            }
        }

        const vitalsSamples = [];
        if (Number.isFinite(this.currentHeartRate) || Number.isFinite(this.currentRespiratoryRate)) {
            vitalsSamples.push({
                t_s: 0,
                hr: Number.isFinite(this.currentHeartRate) ? Number(this.currentHeartRate) : null,
                rr: Number.isFinite(this.currentRespiratoryRate) ? Number(this.currentRespiratoryRate) : null,
                hr_status: this.vitalsDisplayState?.HR?.status || 'warming',
                rr_status: this.vitalsDisplayState?.RR?.status || 'warming',
                hr_age_ms: this.vitalsDisplayState?.HR?.ageMs ?? null,
                rr_age_ms: this.vitalsDisplayState?.RR?.ageMs ?? null,
                motion_state: this.vitalsDisplayState?.motion?.state || 'unknown'
            });
        }

        return {
            event_id: `ble_${Date.now()}`,
            ts: new Date(endTsMs).toISOString(),
            animal: {
                animal_id: pet.animal_id,
                species: pet.species,
                name: pet.name,
                breed: pet.breed,
                sex: pet.sex,
                age_months: pet.age_months,
                weight_kg: pet.weight_kg,
                medical_condition: pet.medical_condition
            },
            device: {
                device_id: cfg.deviceId,
                firmware: 'unknown',
                sampling_hz: { accel: fs, temperature: fs, temp: fs, vitals: 1 }
            },
            window: {
                start_ts: new Date(startTsMs).toISOString(),
                end_ts: new Date(endTsMs).toISOString(),
                timezone
            },
            context: {
                notes: pet.medical_condition || 'web ble upload',
                tags: ['web', 'ble'],
                location: { lat: 0, lng: 0, accuracy_m: 0 }
            },
            signals: {
                accel: { samples: accelSamples },
                temperature: { samples: tempSamples },
                vitals: { samples: vitalsSamples }
            }
        };
    }

    async _sendBleUploadOnce() {
        if (!this.bleUploadEnabled) return;
        const cfg = this._getBleUploadConfig();
        if (!cfg.url) return;
        const payload = this._buildBleEventPayload();
        if (!payload) return;

        try {
            const resp = await fetch(cfg.url, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(payload)
            });
            if (!resp.ok) {
                this.addBLELog(`❌ 上报失败: HTTP ${resp.status}`);
                this._setBleUploadStatus(`失败(${resp.status})`);
                return;
            }
            this.bleLastUploadTs = Date.now();
            this._setBleUploadStatus(`上传中(最近: ${new Date(this.bleLastUploadTs).toLocaleTimeString()})`);
        } catch (e) {
            this.addBLELog(`❌ 上报异常: ${e.message}`);
            this._setBleUploadStatus('异常');
        }
    }

    /**
     * 构建“连接诊断”信息（用于排查：采样率配置、是否真50Hz、是否有IMU、是否丢包/抖动等）
     */
    buildBleDiagnostics() {
        const now = Date.now();
        const fsCfg = (this.processor && Number.isFinite(this.processor.fs)) ? this.processor.fs : null;
        const stats = this.bleStats || {};
        const elapsedSec = stats.startRxTs ? Math.max(0.001, (now - stats.startRxTs) / 1000) : null;
        const actualFs = (elapsedSec && stats.received) ? (stats.received / elapsedSec) : null;
        const lossRate = (stats.expected && stats.missed !== undefined) ? (stats.missed / Math.max(1, stats.expected)) : null;

        const lastGX = this.bleBufferIMU_X?.length ? this.bleBufferIMU_X[this.bleBufferIMU_X.length - 1] : null;
        const lastGY = this.bleBufferIMU_Y?.length ? this.bleBufferIMU_Y[this.bleBufferIMU_Y.length - 1] : null;
        const lastGZ = this.bleBufferIMU_Z?.length ? this.bleBufferIMU_Z[this.bleBufferIMU_Z.length - 1] : null;
        const last = this.lastVitalsResult || null;
        const compactPeaks = peaks => (peaks || []).slice(0, 5).map(peak => ({
            bpm: Number.isFinite(peak?.bpm)
                ? peak.bpm
                : Number.isFinite(peak?.freq) ? peak.freq * 60 : null,
            score: Number.isFinite(peak?.score) ? peak.score : null,
            rawRelative: Number.isFinite(peak?.rawRelative) ? peak.rawRelative : null,
            robustRelative: Number.isFinite(peak?.robustRelative) ? peak.robustRelative : null,
            prominenceRelative: Number.isFinite(peak?.prominenceRelative)
                ? peak.prominenceRelative
                : null,
            harmonicOrder: peak?.harmonicOrder ?? peak?.harmonic?.order ?? null,
            harmonicCloseness: Number.isFinite(peak?.harmonicCloseness)
                ? peak.harmonicCloseness
                : Number.isFinite(peak?.harmonic?.closeness) ? peak.harmonic.closeness : null
        }));

        return {
            algorithmBuild: 'v5-low-bias-guard-20260721',
            ts: new Date().toISOString(),
            bleConnected: !!this.bleConnected,
            samplingRateConfigHz: fsCfg,
            receivedSamples: stats.received ?? 0,
            expectedSamples: stats.expected ?? 0,
            missedSamples: stats.missed ?? 0,
            lossRateEstimated: lossRate,
            actualReceiveRateHz: actualFs,
            jitterEmaMs: stats.gapJitterEmaMs ?? null,
            seqBased: !!stats.seqBased,
            buffers: {
                lenI: this.bleBufferI?.length ?? 0,
                lenQ: this.bleBufferQ?.length ?? 0,
                lenGX: this.bleBufferIMU_X?.length ?? 0,
                lenAX: this.bleBufferACC_X?.length ?? 0
            },
            imuLast: { gx: lastGX, gy: lastGY, gz: lastGZ },
            vitals: {
                samplingRate: this.vitalsFsDiagnostic || null,
                displayState: this.vitalsDisplayState || null,
                motion: last?.motion || null,
                continuity: last?.continuity || null,
                lastWindow: last ? {
                    method: last.method || this.vitalsMethod || null,
                    samples: last.samples ?? null,
                    durationSeconds: last.durationSeconds ?? null,
                    fs: last.fs ?? null,
                    rawHR: Number.isFinite(last.raw_HR_bpm) ? last.raw_HR_bpm : null,
                    rawRR: Number.isFinite(last.raw_RR_bpm) ? last.raw_RR_bpm : null,
                    candidateHR: Number.isFinite(last.candidate_HR_bpm) ? last.candidate_HR_bpm : null,
                    candidateRR: Number.isFinite(last.candidate_RR_bpm) ? last.candidate_RR_bpm : null,
                    displayedHR: Number.isFinite(last.HR_bpm) ? last.HR_bpm : null,
                    displayedRR: Number.isFinite(last.RR_bpm) ? last.RR_bpm : null,
                    HR_confidence: last.HR_confidence ?? null,
                    RR_confidence: last.RR_confidence ?? null,
                    signalQuality: last.quality?.score ?? null,
                    circleResidual: last.quality?.circleResidual ?? null,
                    reason: last.reason || '',
                    octaveDecision: last.diagnostics?.octaveCorrection || null,
                    rrUsedForHarmonicBpm: Number.isFinite(last.diagnostics?.rrUsedForHarmonic)
                        ? last.diagnostics.rrUsedForHarmonic * 60
                        : null,
                    harmonicStrength: last.diagnostics?.harmonicStrength ?? null,
                    hrPeaks: compactPeaks(last.diagnostics?.hrPeaksRobustTop),
                    rrPeaks: compactPeaks(last.diagnostics?.rrPeaksTop),
                    guard: last.stability?.guard || null,
                    trackerMessages: last.stability?.messages || []
                } : null
            },
            ui: {
                runtimeText: document.getElementById('vitalsRuntime')?.textContent || '',
                hasBleIMUChartCanvas: !!document.getElementById('bleIMUChart'),
                bluetoothChartsSectionDisplay: document.getElementById('bluetoothChartsSection')
                    ? getComputedStyle(document.getElementById('bluetoothChartsSection')).display
                    : null
            }
        };
    }

    addBLELog(msg) {
        const log = document.getElementById('bleLog');
        if (!log) return;
        const ts = new Date().toLocaleTimeString();
        this._bleLogLines.push(`[${ts}] ${msg}`);
        if (this._bleLogLines.length > 120) this._bleLogLines.splice(0, this._bleLogLines.length - 120);

        // 节流渲染（避免每次都触发 DOM 重排）
        if (this._bleLogRenderTimer) return;
        this._bleLogRenderTimer = setTimeout(() => {
            this._bleLogRenderTimer = null;
            log.style.whiteSpace = 'pre-line';
            log.textContent = this._bleLogLines.join('\n');
            log.scrollTop = log.scrollHeight;
        }, 200); // 5Hz
    }

    _createEmptyBleBackfillBuffers() {
        return {
            labels: [],
            timestamps: [],
            lagMs: [],
            i: [],
            q: [],
            imuX: [],
            imuY: [],
            imuZ: [],
            accX: [],
            accY: [],
            accZ: []
        };
    }

    _createBleBackfillState() {
        return {
            mode: 'idle',
            enabled: false,
            fromDeviceTsMs: null,
            deviceKey: '',
            deviceMac: '',
            checkpoint: null,
            sessionCheckpointTsMs: null,
            backfillCount: 0,
            liveCount: 0,
            overlapCount: 0,
            backfillStartTsMs: null,
            backfillEndTsMs: null,
            currentDeviceTsMs: null,
            currentLagMs: null,
            catchupStreak: 0,
            liveRateStreak: 0,
            lastArrivalTsMs: null,
            lastSampleDeviceTsMs: null,
            connectionStartArrivalTsMs: Date.now(),
            detectionWindowMs: 15000,
            enterReplayStableCount: 3,
            settleLiveStableCount: 8,
            enterLagThresholdMs: 3000,
            exitLagThresholdMs: 1200,
            exitLagStableCount: 5
        };
    }

    addBleBackfillLog(msg) {
        const log = document.getElementById('bleBackfillLog');
        const ts = new Date().toLocaleTimeString();
        this._bleBackfillLogLines.push(`[${ts}] ${msg}`);
        if (this._bleBackfillLogLines.length > 120) {
            this._bleBackfillLogLines.splice(0, this._bleBackfillLogLines.length - 120);
        }

        if (!log || this._bleBackfillLogRenderTimer) return;
        this._bleBackfillLogRenderTimer = setTimeout(() => {
            this._bleBackfillLogRenderTimer = null;
            log.style.whiteSpace = 'pre-line';
            log.textContent = this._bleBackfillLogLines.join('\n');
            log.scrollTop = log.scrollHeight;
        }, 200);
    }

    _ensureBleBackfillSectionVisible() {
        const section = document.getElementById('bleBackfillSection');
        if (section) section.style.display = 'block';
    }

    _buildBleDeviceKey(sample) {
        if (sample.deviceMac) return `mac:${sample.deviceMac}`;
        const bleDeviceId = window.BLE?.device?.id || '';
        if (bleDeviceId) return `ble:${bleDeviceId}`;
        return 'ble:unknown-device';
    }

    _parseBleProtocolTimestamp(rawTs) {
        if (rawTs === null || rawTs === undefined) return null;
        const digits = String(rawTs).trim().replace(/\D/g, '');
        if (!/^\d{15}$/.test(digits)) return null;
        if (/^0{15}$/.test(digits)) return null;

        const yy = Number(digits.slice(0, 2));
        const mm = Number(digits.slice(2, 4));
        const dd = Number(digits.slice(4, 6));
        const hh = Number(digits.slice(6, 8));
        const mi = Number(digits.slice(8, 10));
        const ss = Number(digits.slice(10, 12));
        const ms = Number(digits.slice(12, 15));

        if (mm < 1 || mm > 12 || dd < 1 || dd > 31 || hh > 23 || mi > 59 || ss > 59) {
            return null;
        }

        const date = new Date(2000 + yy, mm - 1, dd, hh, mi, ss, ms);
        if (!Number.isFinite(date.getTime())) return null;
        // JavaScript normalizes invalid dates (for example 250231 -> March 3)
        // instead of rejecting them. Do not silently turn a malformed device
        // clock into a plausible sampling-rate source.
        if (date.getFullYear() !== 2000 + yy
            || date.getMonth() !== mm - 1
            || date.getDate() !== dd
            || date.getHours() !== hh
            || date.getMinutes() !== mi
            || date.getSeconds() !== ss
            || date.getMilliseconds() !== ms) return null;
        return date.getTime();
    }

    _formatBleDeviceTime(tsMs) {
        if (!Number.isFinite(tsMs)) return '--';
        return new Date(tsMs).toLocaleString('zh-CN', {
            hour12: false,
            month: '2-digit',
            day: '2-digit',
            hour: '2-digit',
            minute: '2-digit',
            second: '2-digit'
        });
    }

    _formatBleDuration(ms) {
        if (!Number.isFinite(ms) || ms <= 0) return '0 秒';
        const totalSec = Math.floor(ms / 1000);
        const hh = Math.floor(totalSec / 3600);
        const mm = Math.floor((totalSec % 3600) / 60);
        const ss = totalSec % 60;
        if (hh > 0) return `${hh}小时 ${mm}分 ${ss}秒`;
        if (mm > 0) return `${mm}分 ${ss}秒`;
        return `${ss}秒`;
    }

    _setBleBackfillMode(mode, reason = '') {
        if (this.bleBackfillState.mode === mode) return;
        this.bleBackfillState.mode = mode;
        this.bleBackfillState.catchupStreak = 0;

        const messages = {
            backfill: '🕘 检测到历史补传，正在回补断线期间数据',
            overlap: '🧩 收到断点附近重叠数据'
        };

        if (messages[mode]) {
            this.addBleBackfillLog(reason ? `${messages[mode]}（${reason}）` : messages[mode]);
        }
        this.updateBleBackfillUI();
    }

    _prepareBleBackfillContext(sample) {
        const state = this.bleBackfillState;
        if (!sample.deviceKey) sample.deviceKey = this._buildBleDeviceKey(sample);

        if (!state.deviceKey || state.deviceKey !== sample.deviceKey) {
            state.deviceKey = sample.deviceKey;
            state.deviceMac = sample.deviceMac || '';
            state.checkpoint = null;
            state.sessionCheckpointTsMs = null;
        }
    }

    _classifyBleSample(sample) {
        const state = this.bleBackfillState;
        this._prepareBleBackfillContext(sample);
        const arrivalNow = Date.now();
        if (!Number.isFinite(state.connectionStartArrivalTsMs)) {
            state.connectionStartArrivalTsMs = arrivalNow;
        }

        if (!state.enabled) {
            state.mode = 'idle';
            this.updateBleBackfillUI();
            return 'live';
        }

        if (!Number.isFinite(sample.deviceTsMs)) {
            state.catchupStreak = 0;
            state.liveCount += 1;
            if (state.mode === 'backfill') {
                state.mode = 'completed';
                this.updateBleBackfillUI();
            } else if (state.mode === 'idle' && state.liveCount >= state.settleLiveStableCount) {
                state.mode = 'completed';
                this.updateBleBackfillUI();
            }
            return 'live';
        }

        const previousDeviceTsMs = state.lastSampleDeviceTsMs;
        const previousArrivalTsMs = state.lastArrivalTsMs;
        const deviceDeltaMs = Number.isFinite(previousDeviceTsMs) ? (sample.deviceTsMs - previousDeviceTsMs) : null;
        const arrivalDeltaMs = Number.isFinite(previousArrivalTsMs) ? (arrivalNow - previousArrivalTsMs) : null;
        const lagMs = Math.max(0, arrivalNow - sample.deviceTsMs);
        const withinDetectionWindow = (arrivalNow - state.connectionStartArrivalTsMs) <= state.detectionWindowMs;
        const hasValidReplayWindow = Number.isFinite(state.fromDeviceTsMs) && sample.deviceTsMs > state.fromDeviceTsMs;
        const paceLooksLikeReplay = (
            Number.isFinite(deviceDeltaMs) &&
            Number.isFinite(arrivalDeltaMs) &&
            deviceDeltaMs > 50 &&
            arrivalDeltaMs > 0 &&
            deviceDeltaMs > arrivalDeltaMs * 1.8
        );
        const paceLooksRealtime = (
            Number.isFinite(deviceDeltaMs) &&
            Number.isFinite(arrivalDeltaMs) &&
            deviceDeltaMs > 0 &&
            arrivalDeltaMs > 0 &&
            deviceDeltaMs <= arrivalDeltaMs * 1.4
        );
        const wallClockLooksReasonable = Math.abs(arrivalNow - sample.deviceTsMs) <= 5 * 60 * 1000;
        const replayCandidate = (
            withinDetectionWindow &&
            hasValidReplayWindow &&
            (
                lagMs >= state.enterLagThresholdMs ||
                (paceLooksLikeReplay && lagMs >= 500)
            )
        );

        state.lastSampleDeviceTsMs = sample.deviceTsMs;
        state.lastArrivalTsMs = arrivalNow;

        if (state.mode === 'backfill') {
            state.backfillCount += 1;
            state.backfillEndTsMs = sample.deviceTsMs;
            state.currentDeviceTsMs = sample.deviceTsMs;
            state.currentLagMs = lagMs;
            state.liveRateStreak = paceLooksRealtime || (wallClockLooksReasonable && lagMs <= state.exitLagThresholdMs)
                ? (state.liveRateStreak + 1)
                : 0;

            if (state.liveRateStreak >= state.exitLagStableCount) {
                state.mode = 'completed';
                this.addBleBackfillLog('✅ 补传完成，后续数据恢复到蓝牙实时数据图表');
                this.updateBleBackfillUI();
                return 'live';
            }

            this.updateBleBackfillUI();
            return 'backfill';
        }

        if (state.mode === 'completed') {
            this.updateBleBackfillUI();
            return 'live';
        }

        if (replayCandidate) {
            state.catchupStreak += 1;
            state.liveCount = 0;
        } else {
            state.catchupStreak = 0;
            state.liveCount += 1;
        }

        if (
            state.mode === 'idle' &&
            (
                !withinDetectionWindow ||
                !hasValidReplayWindow ||
                state.liveCount >= state.settleLiveStableCount
            )
        ) {
            state.mode = 'completed';
            if (state.backfillCount === 0 && state.liveCount === state.settleLiveStableCount) {
                this.addBleBackfillLog('ℹ️ 本次重连未检测到补传，实时数据继续显示在蓝牙实时数据图表');
            }
            this.updateBleBackfillUI();
            return 'live';
        }

        if (state.catchupStreak >= state.enterReplayStableCount) {
            state.backfillStartTsMs = sample.deviceTsMs;
            state.backfillEndTsMs = sample.deviceTsMs;
            state.backfillCount += 1;
            state.currentDeviceTsMs = sample.deviceTsMs;
            state.currentLagMs = lagMs;
            state.liveRateStreak = 0;
            state.liveCount = 0;
            this._setBleBackfillMode('backfill', `设备时间落后当前约 ${Math.round(lagMs / 1000)} 秒`);
            this._ensureBleBackfillSectionVisible();
            this.updateBleBackfillUI();
            return 'backfill';
        }

        this.updateBleBackfillUI();
        return 'live';
    }

    _appendBleBackfillSample(sample) {
        this._ensureBleBackfillSectionVisible();
        const buffers = this.bleBackfillBuffers;
        buffers.labels.push(this._formatBleDeviceTime(sample.deviceTsMs || sample.ts));
        buffers.timestamps.push(sample.deviceTsMs || sample.ts || Date.now());
        buffers.lagMs.push(Number.isFinite(sample.deviceTsMs) ? Math.max(0, Date.now() - sample.deviceTsMs) : 0);
        buffers.i.push(sample.iVal);
        buffers.q.push(sample.qVal);
        buffers.imuX.push(Number.isFinite(sample.imuX) ? sample.imuX : 0);
        buffers.imuY.push(Number.isFinite(sample.imuY) ? sample.imuY : 0);
        buffers.imuZ.push(Number.isFinite(sample.imuZ) ? sample.imuZ : 0);
        buffers.accX.push(Number.isFinite(sample.accX) ? sample.accX : 0);
        buffers.accY.push(Number.isFinite(sample.accY) ? sample.accY : 0);
        buffers.accZ.push(Number.isFinite(sample.accZ) ? sample.accZ : 0);
        this._trimBleBackfillBuffersIfNeeded();
    }

    _trimBleBackfillBuffersIfNeeded() {
        const len = this.bleBackfillBuffers.timestamps.length;
        if (len <= this.bleBackfillMaxBufferHard) return;
        const removeCount = len - this.bleBackfillMaxBuffer;
        Object.values(this.bleBackfillBuffers).forEach(arr => {
            if (Array.isArray(arr) && arr.length >= removeCount) arr.splice(0, removeCount);
        });
    }

    _appendBleLiveSample(sample) {
        this.bleBufferTimestamps.push(sample.ts);
        this.bleBufferDeviceTimestamps.push(
            Number.isFinite(sample.deviceTsMs) ? sample.deviceTsMs : null
        );
        this.bleBufferArrivalTimestamps.push(
            Number.isFinite(sample.arrivalMonoMs) ? sample.arrivalMonoMs : performance.now()
        );
        this.bleBufferNotificationIds.push(
            Number.isFinite(sample.notificationId) ? sample.notificationId : null
        );
        this.bleBufferI.push(sample.iVal);
        this.bleBufferQ.push(sample.qVal);
        this.bleBufferIMU_X.push(Number.isFinite(sample.imuX) ? sample.imuX : 0);
        this.bleBufferIMU_Y.push(Number.isFinite(sample.imuY) ? sample.imuY : 0);
        this.bleBufferIMU_Z.push(Number.isFinite(sample.imuZ) ? sample.imuZ : 0);
        this.bleBufferACC_X.push(Number.isFinite(sample.accX) ? sample.accX : 0);
        this.bleBufferACC_Y.push(Number.isFinite(sample.accY) ? sample.accY : 0);
        this.bleBufferACC_Z.push(Number.isFinite(sample.accZ) ? sample.accZ : 0);
        if (sample.temperature !== null && Number.isFinite(sample.temperature)) {
            this.bleBufferTemperature.push(sample.temperature);
        } else {
            this.bleBufferTemperature.push(null);
        }
    }

    updateBleBackfillUI() {
        const state = this.bleBackfillState;
        const modeEl = document.getElementById('bleBackfillMode');
        const countEl = document.getElementById('bleBackfillCount');
        const durationEl = document.getElementById('bleBackfillDuration');
        const lagEl = document.getElementById('bleBackfillLag');
        const checkpointEl = document.getElementById('bleBackfillCheckpoint');
        const currentTsEl = document.getElementById('bleBackfillCurrentTs');
        const overlapEl = document.getElementById('bleBackfillOverlap');

        const modeMap = state.enabled ? {
            idle: ['待补传', '#6c757d'],
            live: ['待补传', '#6c757d'],
            completed: [state.backfillCount > 0 ? '补传结束' : '未检测到补传', state.backfillCount > 0 ? '#34a853' : '#6c757d'],
            backfill: ['补传中', '#ff9900'],
            overlap: ['断点重叠', '#6f42c1']
        } : {
            idle: ['未启用', '#6c757d'],
            live: ['未启用', '#6c757d'],
            completed: ['未启用', '#6c757d'],
            backfill: ['补传中', '#ff9900'],
            overlap: ['断点重叠', '#6f42c1']
        };
        const [modeText, modeColor] = modeMap[state.mode] || ['待检测', '#6c757d'];

        if (modeEl) {
            modeEl.textContent = modeText;
            modeEl.style.color = modeColor;
        }
        if (countEl) countEl.textContent = `${state.backfillCount} 条`;

        const baseTs = state.sessionCheckpointTsMs ?? state.backfillStartTsMs;
        const endTs = state.backfillEndTsMs ?? state.currentDeviceTsMs;
        if (durationEl) {
            const coveredMs = state.backfillCount > 0 && Number.isFinite(baseTs) && Number.isFinite(endTs)
                ? Math.max(0, endTs - baseTs)
                : 0;
            durationEl.textContent = this._formatBleDuration(coveredMs);
        }
        if (lagEl) lagEl.textContent = (state.mode === 'backfill' || state.mode === 'completed') && Number.isFinite(state.currentLagMs)
            ? `${(state.currentLagMs / 1000).toFixed(1)} 秒`
            : '--';
        if (checkpointEl) {
            checkpointEl.textContent = state.enabled
                ? (state.mode === 'completed' && state.backfillCount === 0 ? '本次连接按实时数据处理' : '本次连接已启用')
                : '需先手动断开再重连';
        }
        if (currentTsEl) currentTsEl.textContent = (state.mode === 'backfill' || state.mode === 'completed') && Number.isFinite(state.currentDeviceTsMs)
            ? this._formatBleDeviceTime(state.currentDeviceTsMs)
            : '--';
        if (overlapEl) overlapEl.textContent = `${state.overlapCount} 条`;
    }

    /**
     * 处理 BLE 行数据 - 蓝牙实时数据接口
     * 默认逐行格式: ts i q
     */
    handleBLELine(line, transportMeta = null) {
        const normalizedLine = String(line ?? '').trim();
        // AT acknowledgements/status lines share the same BLE channel but are
        // not radar samples.  Feeding them into the numeric fallback parser can
        // create artificial I/Q impulses and false spectral peaks.
        if (/^\$AT(?:[+,:=*]|$)/i.test(normalizedLine)) {
            this.printRawData(normalizedLine);
            this.lastBleRxTs = Date.now();
            this.bleRxStallWarned = false;
            return;
        }

        // 保存原始蓝牙数据（如果正在录制）
        if (this.bleRecordingFlag === 1) {
            this.bleRecordingRawData.push(normalizedLine);
        }

        // 打印原始数据
        this.printRawData(normalizedLine);
        this.lastBleRxTs = Date.now();
        this.bleRxStallWarned = false;
        // 允许 JSON 格式 {ts:..., i:..., q:...}；也兼容无空格双小数如 "1.6421.588"
        let ts, iVal, qVal;
        let seq = null;
        let deviceMac = '';
        let deviceTsRaw = '';
        let deviceTsMs = null;
        let imuX = 0, imuY = 0, imuZ = 0; // gx/gy/gz（优先取 Gyr:）
        let temperature = null; // 温度数据
        let accX = 0, accY = 0, accZ = 0; // Acc原始值
        let roll = 0, pitch = 0, yaw = 0; // Roll/Pitch/Yaw 姿态角
        let gpsLonRaw = '', gpsLatRaw = ''; // GPS 经纬度原始字符串
        try {
            const trimmed = normalizedLine;

            // ── 新协议 V1.02：$MAC,Time,Lon,Lat,Ax,Ay,Az,Gx,Gy,Gz,Roll,Pitch,Yaw,V1,V2,V3* ──
            if (trimmed.startsWith('$') && !trimmed.startsWith('$AT')) {
                // bluetooth.js strips '*' as delimiter; handle both $...*  and $... formats
                const raw = trimmed.endsWith('*') ? trimmed.slice(1, -1) : trimmed.slice(1);
                const parts   = raw.split(',');
                if (parts.length >= 16) {
                    deviceMac = parts[0] || '';
                    deviceTsRaw = parts[1] || '';
                    deviceTsMs = this._parseBleProtocolTimestamp(deviceTsRaw);
                    // Acc 已是 g，Gyr 已是 deg/s，直接使用
                    accX = parseFloat(parts[4]);
                    accY = parseFloat(parts[5]);
                    accZ = parseFloat(parts[6]);
                    imuX = parseFloat(parts[7]);
                    imuY = parseFloat(parts[8]);
                    imuZ = parseFloat(parts[9]);
                    roll  = parseFloat(parts[10]);
                    pitch = parseFloat(parts[11]);
                    yaw   = parseFloat(parts[12]);
                    iVal = parseFloat(parts[14]);
                    qVal = parseFloat(parts[15]);
                    ts   = Number.isFinite(deviceTsMs) ? deviceTsMs : Date.now();
                    temperature = null;

                    // GPS 时间戳连续性检测丢帧
                    const gpsTs = deviceTsMs;
                    if (Number.isFinite(gpsTs) && this._lastGpsTs !== null) {
                        const gpsDeltaMs = gpsTs - this._lastGpsTs;
                        const expectedGapMs = this.bleTargetFs > 0 ? (1000 / this.bleTargetFs) : 20;
                        if (gpsDeltaMs > expectedGapMs * 1.8 && gpsDeltaMs < expectedGapMs * 100) {
                            const missed = Math.round(gpsDeltaMs / expectedGapMs) - 1;
                            if (missed > 0) this._gpsGapMissed += missed;
                        }
                    }
                    if (Number.isFinite(gpsTs)) this._lastGpsTs = gpsTs;
                } else {
                    return;
                }

                gpsLonRaw = parts[2] || '';
                gpsLatRaw = parts[3] || '';

                if (this.bleRecordingFlag === 1) {
                    const stamp = new Date().toISOString().replace('T', ' ').slice(0, 23);
                    const mac   = deviceMac;
                    const lon   = parts[2] || '';
                    const lat   = parts[3] || '';
                    const v1    = parts[13] || '';
                    const v2    = parts[14] || '';
                    const v3    = parts[15] || '';
                    // Payload Format: MAC,Time,Lon,Lat,Ax,Ay,Az,Gx,Gy,Gz,Roll,Pitch,Yaw,V1,V2,V3
                    this.bleRecordingData.push(
                        `${mac},${stamp},${lon},${lat},${accX.toFixed(4)},${accY.toFixed(4)},${accZ.toFixed(4)},${imuX.toFixed(3)},${imuY.toFixed(3)},${imuZ.toFixed(3)},${roll.toFixed(4)},${pitch.toFixed(4)},${yaw.toFixed(4)},${v1},${v2},${v3}`
                    );
                }

            } else {
                // ── 旧协议兼容：ADC:xxx xxx|Acc:x y z|Gyr:x y z|T:xx ──
                const floatRe = /[+-]?(?:\d+\.\d+|\d+|\.\d+)(?:[eE][+-]?\d+)?/g;

                const parsePairAfterLabel = (label) => {
                    const idx = trimmed.indexOf(label);
                    if (idx < 0) return null;
                    const seg = trimmed.slice(idx + label.length);
                    const firstField = seg.split('|')[0] || '';
                    const nums = firstField.match(floatRe)?.map(v => parseFloat(v)) || [];
                    return nums.length >= 2 ? [nums[0], nums[1]] : null;
                };
                const parseTripletAfterLabel = (label) => {
                    const idx = trimmed.indexOf(label);
                    if (idx < 0) return null;
                    const seg = trimmed.slice(idx + label.length);
                    const firstField = seg.split('|')[0] || '';
                    const nums = firstField.match(floatRe)?.map(v => parseFloat(v)) || [];
                    return nums.length >= 3 ? [nums[0], nums[1], nums[2]] : null;
                };

                const adc = parsePairAfterLabel('ADC:') || parsePairAfterLabel('adc:');
                if (adc) {
                    iVal = ((adc[0] / 32767) + 1) * 3.3 / 2;
                    qVal = ((adc[1] / 32767) + 1) * 3.3 / 2;
                    ts   = Date.now();
                }

                const gyr = parseTripletAfterLabel('Gyr:') || parseTripletAfterLabel('GYR:');
                const acc = parseTripletAfterLabel('Acc:') || parseTripletAfterLabel('ACC:');
                if (acc) { accX = acc[0]; accY = acc[1]; accZ = acc[2]; }
                if (gyr) { [imuX, imuY, imuZ] = gyr; }
                else if (acc) { [imuX, imuY, imuZ] = acc; }

                const tempIdx = trimmed.indexOf('T:');
                if (tempIdx >= 0) {
                    const tempMatch = trimmed.slice(tempIdx + 2).match(floatRe);
                    if (tempMatch) temperature = parseFloat(tempMatch[0]);
                }

                const seqMatch = trimmed.match(/(?:\bSEQ\b|\bseq\b|\bidx\b|\bindex\b)\s*[:=]\s*(\d+)/);
                if (seqMatch) seq = parseInt(seqMatch[1], 10);

                if (!Number.isFinite(iVal) || !Number.isFinite(qVal)) {
                    if (trimmed.startsWith('{') && trimmed.endsWith('}')) {
                        const obj = JSON.parse(trimmed);
                        ts = obj.ts ?? Date.now();
                        iVal = parseFloat(obj.i);
                        qVal = parseFloat(obj.q);
                        if (seq === null && obj.seq !== undefined) seq = parseInt(obj.seq, 10);
                    } else {
                        const parts2 = trimmed.split(/\s+/);
                        if (parts2.length >= 3) {
                            ts   = parts2[0];
                            iVal = parseFloat(parts2[1]);
                            qVal = parseFloat(parts2[2]);
                        } else {
                            const matches = [...trimmed.matchAll(floatRe)];
                            if (matches.length >= 2) {
                                ts   = Date.now();
                                iVal = parseFloat(matches[0][0]);
                                qVal = parseFloat(matches[1][0]);
                            } else if (matches.length === 1) {
                                const val = parseFloat(matches[0][0]);
                                if (!Number.isFinite(val)) return;
                                if (this.blePendingFloat === null) { this.blePendingFloat = val; return; }
                                ts   = Date.now();
                                iVal = this.blePendingFloat;
                                qVal = val;
                                this.blePendingFloat = null;
                            } else { return; }
                        }
                    }
                }

                if (this.bleRecordingFlag === 1) {
                    const stamp = new Date().toISOString().replace('T', ' ').slice(0, 19);
                    const adcMatch = trimmed.match(/ADC:([-\d]+)\s+([-\d]+)/);
                    let rAdcI = 0, rAdcQ = 0;
                    if (adcMatch) { rAdcI = parseInt(adcMatch[1]); rAdcQ = parseInt(adcMatch[2]); }
                    const lastTemp = this.bleBufferTemperature[this.bleBufferTemperature.length - 1];
                    const tempStr = lastTemp !== null && lastTemp !== undefined ? lastTemp.toFixed(2) : 'N/A';
                    // Payload Format: timestamp,ADC_I,ADC_Q,Acc_X,Acc_Y,Acc_Z,I_voltage,Q_voltage,Gyr_x,Gyr_y,Gyr_z,temperature
                    this.bleRecordingData.push(
                        `${stamp},${rAdcI},${rAdcQ},${accX.toFixed(3)},${accY.toFixed(3)},${accZ.toFixed(3)},${iVal.toFixed(6)},${qVal.toFixed(6)},${imuX.toFixed(3)},${imuY.toFixed(3)},${imuZ.toFixed(3)},${tempStr}`
                    );
                }
            }
        } catch (err) { 
            // 🔍 调试：捕获异常
            if (this.bleDataCount < 10) {
                console.log(`  ❌ 解析异常:`, err);
            }
            return; 
        }

        // 🔍 调试：检查最终的 iVal 和 qVal
        if (this.bleDataCount < 10) {
            console.log(`  最终检查: iVal=${iVal}, qVal=${qVal}`);
            console.log(`  iVal有效: ${Number.isFinite(iVal)}, qVal有效: ${Number.isFinite(qVal)}`);
        }

        if (!Number.isFinite(iVal) || !Number.isFinite(qVal)) {
            if (this.bleDataCount < 10) {
                console.log(`  ❌ 数据无效，丢弃此行`);
                console.log(`========================================\n`);
            }
            return;
        }

        const sample = {
            ts,
            seq,
            iVal,
            qVal,
            imuX,
            imuY,
            imuZ,
            accX,
            accY,
            accZ,
            roll,
            pitch,
            yaw,
            temperature,
            deviceMac,
            deviceTsRaw,
            deviceTsMs,
            arrivalTsMs: Number.isFinite(transportMeta?.arrivalMs)
                ? Number(transportMeta.arrivalMs)
                : Date.now(),
            arrivalMonoMs: Number.isFinite(transportMeta?.arrivalMonoMs)
                ? Number(transportMeta.arrivalMonoMs)
                : performance.now(),
            notificationId: Number.isFinite(transportMeta?.notificationId)
                ? Number(transportMeta.notificationId)
                : null
        };
        sample.deviceKey = this._buildBleDeviceKey(sample);
        const sampleMode = this._classifyBleSample(sample);

        // 🔍 调试：确认数据被添加
        if (this.bleDataCount < 10) {
            console.log(`  ✅ 准备添加到buffer: I=${iVal.toFixed(4)}V, Q=${qVal.toFixed(4)}V`);
            console.log(`  当前buffer长度: I=${this.bleBufferI.length}, Q=${this.bleBufferQ.length}`);
        }
        if (sampleMode === 'live') {
            this._appendBleLiveSample(sample);
            this._updateBleLossStats(seq);
            if (Number.isFinite(sample.deviceTsMs)) {
                this.lastLiveDeviceTsMs = sample.deviceTsMs;
            }

            // 🔍 调试：验证数据确实被添加
            if (this.bleDataCount < 10) {
                const lastI = this.bleBufferI[this.bleBufferI.length - 1];
                const lastQ = this.bleBufferQ[this.bleBufferQ.length - 1];
                console.log(`  ✅ 添加后验证: I数组最后一个=${lastI?.toFixed(4)}, Q数组最后一个=${lastQ?.toFixed(4)}`);
                console.log(`  添加后buffer长度: I=${this.bleBufferI.length}, Q=${this.bleBufferQ.length}`);
                console.log(`========================================\n`);
            }
            this._ensureVitalsRealtime();
        } else if (sampleMode === 'backfill') {
            this._appendBleBackfillSample(sample);
        }

        this.bleDataCount++;
        document.getElementById('bleDataCount').textContent = this.bleDataCount;
        document.getElementById('bleTotalDataPoints').textContent = this.bleDataCount;
        this.updateBleBackfillUI();

        if (sampleMode === 'backfill') {
            if (this.bleBackfillState.backfillCount === 1) {
                this.addBleBackfillLog('📦 已开始接收 Flash 补传数据');
            }
            this._scheduleBleChartUpdate();
            return;
        }

        // ===== 活动量监测：将加速度数据传递给ActivityMonitor =====
        if (this.activityMonitorEnabled && this.activityMonitor && accX !== null && accY !== null && accZ !== null) {
            this.activityMonitor.addAccelerometerData(accX, accY, accZ, Date.now());
        }

        // 睡眠监测模块：使用加速度计数据
        if (this.sleepMonitorEnabled && this.sleepMonitor && accX !== null && accY !== null && accZ !== null) {
            this.sleepMonitor.addAccelerometerData(accX, accY, accZ, Date.now());
        }

        // ===== GPS 定位追踪：将经纬度数据传递给GPSLocationMonitor =====
        if (this.gpsLocationMonitorEnabled && this.gpsLocationMonitor && gpsLonRaw && gpsLatRaw) {
            this.gpsLocationMonitor.addGPSData(gpsLonRaw, gpsLatRaw, Date.now());
        }

        // ===== 姿态解算：使用陀螺仪和加速度计数据 =====
        if (this.attitudeEnabled && this.attitudeSolver) {
            const hasValidAcc = accX !== 0 || accY !== 0 || accZ !== 0;
            const hasValidGyr = imuX !== 0 || imuY !== 0 || imuZ !== 0;

            if (this.bleDataCount < 5) {
                console.log(`📊 姿态数据 #${this.bleDataCount}:`, {
                    gyr: { x: imuX, y: imuY, z: imuZ },
                    acc: { x: accX, y: accY, z: accZ },
                    hasValidAcc: hasValidAcc,
                    hasValidGyr: hasValidGyr
                });
            }

            if (hasValidAcc) {
                this.attitudeSolver.update(imuX, imuY, imuZ, accX, accY, accZ);
                updateAttitudeDisplay();
            } else if (this.bleDataCount === 5) {
                console.warn('⚠️ 前5条数据中没有检测到有效的 Acc 数据！');
                console.warn('请确保蓝牙设备发送的数据包含 Acc: 字段');
                console.warn('期望格式: ADC:xxx xxx|Acc:x.xxx y.yyy z.zzz|Gyr:x.x y.y z.z|T:xx.x');
            }
        }

        if (this.bleDataCount <= 5) {
            const bleRealTimeData = document.getElementById('bleRealTimeData');
            if (bleRealTimeData && bleRealTimeData.style.display === 'none') {
                bleRealTimeData.style.display = 'block';
            }
            const chartsSection = document.getElementById('bluetoothChartsSection');
            if (chartsSection && chartsSection.style.display === 'none') {
                chartsSection.style.display = 'block';
            }
            if (!this.bleCharts.iSignal || !this.bleCharts.qSignal) {
                this.initializeBluetoothCharts();
            }
        }

        if (typeof restingMonitor !== 'undefined' && restingMonitor) {
            restingMonitor.update();
        }

        if (this.bleDataCount % 10 === 0) {
            const lastI = this.bleBufferI[this.bleBufferI.length - 1];
            const lastQ = this.bleBufferQ[this.bleBufferQ.length - 1];
            const debugInfo = `I=${lastI?.toFixed(4)}V, Q=${lastQ?.toFixed(4)}V (共${this.bleBufferI.length}点)`;
            const debugEl = document.getElementById('bleCurrentIQ');
            if (debugEl) {
                debugEl.textContent = debugInfo;
            }
            const gyrEl = document.getElementById('bleCurrentGyr');
            const accEl = document.getElementById('bleCurrentAcc');
            const tempEl = document.getElementById('bleCurrentTemp');
            if (gyrEl && this.bleBufferIMU_X.length > 0) {
                const gx = this.bleBufferIMU_X[this.bleBufferIMU_X.length - 1];
                const gy = this.bleBufferIMU_Y[this.bleBufferIMU_Y.length - 1];
                const gz = this.bleBufferIMU_Z[this.bleBufferIMU_Z.length - 1];
                gyrEl.textContent = `${gx?.toFixed(2)}, ${gy?.toFixed(2)}, ${gz?.toFixed(2)}`;
            }
            if (accEl && this.bleBufferACC_X.length > 0) {
                const ax = this.bleBufferACC_X[this.bleBufferACC_X.length - 1];
                const ay = this.bleBufferACC_Y[this.bleBufferACC_Y.length - 1];
                const az = this.bleBufferACC_Z[this.bleBufferACC_Z.length - 1];
                accEl.textContent = `${ax?.toFixed(3)}, ${ay?.toFixed(3)}, ${az?.toFixed(3)}`;
            }
            if (tempEl && this.bleBufferTemperature.length > 0) {
                const t = this.bleBufferTemperature[this.bleBufferTemperature.length - 1];
                if (t !== null) tempEl.textContent = `${t.toFixed(1)} °C`;
            }
        }

        // 滑窗（分块裁剪，显著降低长期运行时的卡顿）
        this._trimBleBuffersIfNeeded();

        // 图表更新节流（每条都刷会卡）
        this._scheduleBleChartUpdate();

        // 每累计一段再做一次完整生理参数估计
        const fs = (this.processor && Number.isFinite(this.processor.fs)) ? this.processor.fs : 50;
        if (this.bleBufferI.length % fs === 0 && this.bleBufferI.length >= fs * 5) {
            this.updateBluetoothVitalSigns();
        }
    }

    _trimBleBuffersIfNeeded() {
        const len = this.bleBufferI.length;
        if (len <= this.bleMaxBufferHard) return;
        const removeCount = len - this.bleMaxBuffer;
        if (removeCount <= 0) return;

        // 保持各数组长度一致
        [
            this.bleBufferTimestamps,
            this.bleBufferI,
            this.bleBufferQ,
            this.bleBufferIMU_X,
            this.bleBufferIMU_Y,
            this.bleBufferIMU_Z,
            this.bleBufferACC_X,
            this.bleBufferACC_Y,
            this.bleBufferACC_Z,
            this.bleBufferTemperature,
            this.bleBufferDeviceTimestamps,
            this.bleBufferArrivalTimestamps,
            this.bleBufferNotificationIds
        ].forEach(arr => {
            if (Array.isArray(arr) && arr.length >= removeCount) arr.splice(0, removeCount);
        });
    }

    _scheduleBleChartUpdate() {
        if (this._bleChartRaf) return;
        this._bleChartRaf = requestAnimationFrame(() => {
            this._bleChartRaf = null;
            const now = performance.now();
            if (now - this._bleChartLastUpdateTs < this._bleChartMinIntervalMs) return;
            this._bleChartLastUpdateTs = now;
            this.updateBluetoothLiveCharts();
            this.updateBleBackfillCharts();
        });
    }

    /**
     * 更新 BLE 丢包/实际采样率/抖动（估算）
     * - 默认假设：每调用一次 handleBLELine = 1 个采样点（你的设备目前看起来是这样）
     * - 若提供 seq：使用 seq 计算丢包更准确
     */
    /**
     * 更新 BLE 丢包/实际采样率/抖动，并在稳定后自动同步 processor.fs
     * 新协议 V1.02 默认 Tcycle=500ms (2Hz)，心率检测需至少 6Hz。
     * 收到 >=20 帧后自动测量实际频率并同步到 processor.fs。
     */
    _updateBleLossStats(seq = null) {
        const now = Date.now();
        const nowMono = performance.now();
        const s = this.bleStats;
        if (!s.startRxTs) s.startRxTs = now;
        if (!s.startRxMonoTs) s.startRxMonoTs = nowMono;

        s.received += 1;

        if (Number.isFinite(seq)) {
            if (s.lastSeq !== null) {
                const gap = seq - s.lastSeq - 1;
                if (gap > 0) { s.missed += gap; s.seqBased = true; }
            }
            s.lastSeq = seq;
        }

        if (s.lastRxMonoTs > 0) {
            const gapMs = nowMono - s.lastRxMonoTs;
            s.lastGapMs = gapMs;
            const alpha = 0.1;
            s.gapEmaMs       = s.gapEmaMs       ? (alpha * gapMs  + (1 - alpha) * s.gapEmaMs)       : gapMs;
            const jitter     = Math.abs(gapMs - s.gapEmaMs);
            s.gapJitterEmaMs = s.gapJitterEmaMs ? (alpha * jitter + (1 - alpha) * s.gapJitterEmaMs) : jitter;
        }
        s.lastRxTs = now;
        s.lastRxMonoTs = nowMono;

        const fs = (this.processor && Number.isFinite(this.processor.fs)) ? this.processor.fs : 50;
        const elapsedSec = Math.max(0.001, (nowMono - s.startRxMonoTs) / 1000);
        if (s.seqBased) {
            s.expected = s.received + s.missed;
        } else {
            s.expected = Math.round(elapsedSec * fs);
            s.missed   = Math.max(0, s.expected - s.received);
        }

        if (s.received % Math.max(1, Math.floor(Math.max(fs, 1) / 2)) !== 0) return;

        const actualFs = s.received / elapsedSec;
        const lossRate = s.expected > 0 ? (s.missed / s.expected) : 0;

        const fsEl     = document.getElementById('bleActualFs');
        const lossEl   = document.getElementById('blePacketLoss');
        const jitterEl = document.getElementById('bleJitter');
        if (fsEl) {
            fsEl.textContent = `${actualFs.toFixed(1)} Hz`;
            fsEl.style.color = actualFs < 6 ? '#ff4444' : actualFs < 20 ? '#ff9900' : '';
            fsEl.title = actualFs < 6
                ? `频率过低！需要至少 6 Hz 才能检测心率`
                : actualFs < 20
                ? `频率偏低，建议设置为 50 Hz`
                : `采样率正常`;
        }
        if (lossEl)   lossEl.textContent   = `${(lossRate * 100).toFixed(1)} %`;
        if (jitterEl) jitterEl.textContent = `${(s.gapJitterEmaMs || 0).toFixed(1)} ms`;

        const targetEl  = document.getElementById('bleTargetFs');
        const missedEl  = document.getElementById('bleMissedFrames');
        const barEl     = document.getElementById('bleFreqBar');
        const barTextEl = document.getElementById('bleFreqBarText');
        if (targetEl) targetEl.textContent = `${this.bleTargetFs} Hz`;
        if (missedEl) {
            const gpsMissed = this._gpsGapMissed || 0;
            const totalMissed = Math.max(s.missed, gpsMissed);
            missedEl.textContent = `${totalMissed} / ${s.received + totalMissed}`;
        }
        if (barEl && barTextEl) {
            const pct = this.bleTargetFs > 0 ? Math.min(100, (actualFs / this.bleTargetFs) * 100) : 0;
            barEl.style.width = `${pct}%`;
            barEl.style.background = pct >= 90 ? '#34a853' : pct >= 60 ? '#ff9900' : '#ff4444';
            barTextEl.textContent = `${pct.toFixed(1)}%`;
        }
    }

    _updateConfigStatusUI() {
        const el = document.getElementById('bleConfigStatus');
        if (!el) return;
        const processingFs = this.vitalsFsDiagnostic?.selected
            || (this.processor && Number.isFinite(this.processor.fs) ? this.processor.fs : null);
        const targetText = Number.isFinite(this.bleTargetFs) ? `${this.bleTargetFs} Hz` : '--';
        const processingText = Number.isFinite(processingFs) ? `${Number(processingFs).toFixed(1)} Hz` : '--';
        const configuredText = Number.isFinite(processingFs)
            && Math.abs(processingFs - this.bleTargetFs) / Math.max(this.bleTargetFs, 1) > 0.12
            ? `目标 ${targetText} · 算法按实测 ${processingText}`
            : `已验证 ${processingText}`;
        const map = {
            pending:     ['等待连接...', '#aaa'],
            configuring: [`正在配置目标 ${targetText}...`, '#ff9900'],
            configured:  [configuredText, '#34a853'],
            fallback:    [`目标 ${targetText} 未达成 · 算法 ${processingText}`, '#ff9900'],
            failed:      ['配置失败', '#ff4444']
        };
        const [text, color] = map[this.bleConfigStatus] || ['--', '#aaa'];
        el.textContent = text;
        el.style.color = color;

        // 配置失败、已配置或降级保持时显示手动重配按钮
        const btn = document.getElementById('bleReconfigBtn');
        if (btn) {
            btn.style.display = (
                this.bleConfigStatus === 'failed' ||
                this.bleConfigStatus === 'configured' ||
                this.bleConfigStatus === 'fallback'
            ) ? 'inline-block' : 'none';
        }
    }

    _beginVitalsSamplingEpoch(reason = '采样时钟变化') {
        const nowMs = Date.now();
        this.vitalsSamplingEpoch = Number.isFinite(this.vitalsSamplingEpoch)
            ? this.vitalsSamplingEpoch + 1
            : 1;
        [
            this.bleBufferTimestamps,
            this.bleBufferDeviceTimestamps,
            this.bleBufferArrivalTimestamps,
            this.bleBufferNotificationIds,
            this.bleBufferI,
            this.bleBufferQ,
            this.bleBufferIMU_X,
            this.bleBufferIMU_Y,
            this.bleBufferIMU_Z,
            this.bleBufferACC_X,
            this.bleBufferACC_Y,
            this.bleBufferACC_Z,
            this.bleBufferTemperature
        ].forEach(buffer => {
            if (Array.isArray(buffer)) buffer.length = 0;
        });
        this.vitalsFsEstimateInfo = null;
        this.vitalsFsDiagnostic = null;
        this._vitalsFsLogKey = '';
        this.vitalsLastGuardResult = null;
        this.vitalsLastGuardTs = 0;
        this._vitalsMotionBaseline = null;
        // An epoch boundary invalidates spectral history even though the last
        // trustworthy displayed value is intentionally preserved.  Without
        // this transition an old 40 bpm result could keep its frozen `fresh`
        // age while the new window warms up, then veto a correct 75--80 bpm
        // octave on the first post-change estimate.
        this.vitalsHrHistory = [];
        this.vitalsRrHistory = [];
        this.vitalsAcceptedCycles = 0;
        if (this.vitalsDisplayState) {
            for (const kind of ['HR', 'RR']) {
                const display = this.vitalsDisplayState[kind];
                const reliable = this.vitalsLastReliable?.[kind];
                if (!display) continue;
                const ageMs = Number.isFinite(reliable?.timestampMs)
                    ? Math.max(0, nowMs - reliable.timestampMs)
                    : null;
                const hasReliable = Number.isFinite(reliable?.value);
                display.currentValue = null;
                display.ageMs = ageMs;
                display.isFresh = false;
                display.status = hasReliable
                    ? (Number.isFinite(ageMs) && ageMs <= this.vitalsHoldMaxMs ? 'held' : 'expired')
                    : 'warming';
            }
            this.vitalsDisplayState.timestampMs = nowMs;
            this.vitalsDisplayState.samplingEpoch = this.vitalsSamplingEpoch;
        }
        // Keep the last trustworthy HR/RR in the tracker. It may be displayed
        // as held while the new epoch warms up, but one FFT/CWT window can
        // never contain samples captured at two different rates.
        if (typeof this.addBLELog === 'function') {
            this.addBLELog(`🧭 新建采样时钟 epoch #${this.vitalsSamplingEpoch}：${reason}；重新积累纯净窗口`);
        }
    }

    _applyBleSamplingRate(fs, { updateTarget = true } = {}) {
        const normalizedFs = Math.max(1, Math.round(fs || 0));
        const previousFs = this.processor && Number.isFinite(this.processor.fs)
            ? this.processor.fs
            : NaN;
        if (Number.isFinite(previousFs)
            && Math.abs(normalizedFs - previousFs) / Math.max(previousFs, 1) > 0.08
            && (this.bleBufferI?.length || 0) > 0) {
            this._beginVitalsSamplingEpoch(
                `算法采样率 ${previousFs.toFixed(1)} → ${normalizedFs.toFixed(1)} Hz`
            );
        }
        if (this.processor) this.processor.fs = normalizedFs;
        if (this.activityMonitor) this.activityMonitor.fs = normalizedFs;
        if (this.sleepMonitor) this.sleepMonitor.fs = normalizedFs;
        if (updateTarget) this.bleTargetFs = normalizedFs;
        return normalizedFs;
    }

    _estimateCurrentBleFs() {
        const s = this.bleStats;
        if (s && s.startRxTs && s.received > 0) {
            const elapsedSec = s.startRxMonoTs > 0
                ? Math.max(0.001, (performance.now() - s.startRxMonoTs) / 1000)
                : Math.max(0.001, (Date.now() - s.startRxTs) / 1000);
            const actualFs = s.received / elapsedSec;
            if (Number.isFinite(actualFs) && actualFs > 0) return actualFs;
        }
        return null;
    }

    _fallbackToCurrentBleRate(reason = '') {
        const estimatedFs = this._estimateCurrentBleFs();
        const fallbackFs = this._applyBleSamplingRate(estimatedFs || 2);
        this.bleConfigStatus = 'fallback';
        this._updateConfigStatusUI();
        const suffix = reason ? `，${reason}` : '';
        this.addBLELog(`⚠️ 50Hz 未达成，保持当前约 ${fallbackFs} Hz 继续连接${suffix}`);
        return fallbackFs;
    }

    _getBleProtocolLabel() {
        try {
            if (!window.BLE || typeof BLE.getConnectionProfile !== 'function') return this.bleProtocol || '未连接';
            const profile = BLE.getConnectionProfile();
            if (!profile || !profile.protocol) return this.bleProtocol || '未连接';
            const parts = [profile.protocol];
            if (profile.serviceUuid) parts.push(`服务 ${profile.serviceUuid}`);
            return parts.join(' · ');
        } catch (_) {
            return this.bleProtocol || '未连接';
        }
    }

    _updateBleProtocolUI() {
        const el = document.getElementById('bleProtocol');
        if (!el) return;
        const text = this.bleProtocol || '未连接';
        el.textContent = text;
        el.style.color =
            text.startsWith('FFF0') ? '#34a853' :
            text.startsWith('NUS') ? '#1a73e8' :
            text.startsWith('通用') || text.startsWith('只读') ? '#ff9900' :
            '#aaa';
    }

    /**
     * 自动配置 Tcycle 为 20ms (50Hz)，带重试机制。
     * 发送后监测实际数据到达间隔，如果仍为 ~2Hz 则重试，最多 3 次。
     */
    async _autoConfigTcycleWithRetry(maxRetries = 3) {
        const delayMs = 1500; // 等待 BLE 服务发现完成
        const checkIntervalMs = 2500; // 每次发送后等待 2.5 秒检测效果
        let lastMeasuredFs = 0;

        await new Promise(r => setTimeout(r, delayMs));

        for (let attempt = 1; attempt <= maxRetries; attempt++) {
            if (!this.bleConnected) {
                this.addBLELog('配置中断：蓝牙已断开');
                this.bleConfigStatus = 'failed';
                this._updateConfigStatusUI();
                return;
            }

            // 检查写入特征是否就绪
            if (!window.BLE || !window.BLE.writeCharacteristic) {
                this.addBLELog(`第 ${attempt}/${maxRetries} 次尝试：写入特征未就绪，等待中...`);
                if (attempt === maxRetries) {
                    this._fallbackToCurrentBleRate('写入特征未就绪');
                    return;
                }
                await new Promise(r => setTimeout(r, 1000));
                continue;
            }

            try {
                this.addBLELog(`第 ${attempt}/${maxRetries} 次尝试配置 50Hz...`);
                await this.sendTcycleCommand(20, false);

                // 等待一段时间后检测实际采样率
                const countBefore = this.bleStats.received;
                const tsBefore = Date.now();
                await new Promise(r => setTimeout(r, checkIntervalMs));
                const countAfter = this.bleStats.received;
                const elapsed = (Date.now() - tsBefore) / 1000;
                const actualFs = elapsed > 0 ? (countAfter - countBefore) / elapsed : 0;
                lastMeasuredFs = actualFs;

                const rateRatio = actualFs / Math.max(this.bleTargetFs, 1);
                if (rateRatio >= 0.8 && rateRatio <= 1.2) {
                    // Accept the commanded rate only inside a two-sided
                    // tolerance. The old one-sided check also accepted 100 Hz
                    // as a successful 50 Hz stream and then forced fs back to
                    // 50, producing an exact half-rate HR/RR result.
                    this._applyBleSamplingRate(this.bleTargetFs);
                    this.bleConfigStatus = 'configured';
                    this._updateConfigStatusUI();
                    this.addBLELog(`配置成功！实际采样率 ~${actualFs.toFixed(1)} Hz`);
                    return;
                }

                if (rateRatio > 1.2) {
                    const adoptedFs = this._applyBleSamplingRate(actualFs, { updateTarget: false });
                    this.bleConfigStatus = 'configured';
                    this._updateConfigStatusUI();
                    this.addBLELog(
                        `⚠️ 设备实际输出 ~${actualFs.toFixed(1)} Hz，高于目标 ${this.bleTargetFs} Hz；`
                        + `生命体征算法采用实测 ${adoptedFs} Hz，禁止按 ${this.bleTargetFs} Hz 缩半换算`
                    );
                    return;
                }

                if (attempt < maxRetries) {
                    this.addBLELog(`实际采样率仅 ~${actualFs.toFixed(1)} Hz，将重试...`);
                }
            } catch (e) {
                if (attempt === maxRetries) {
                    this._fallbackToCurrentBleRate(`配置异常: ${e.message}`);
                    return;
                }
                this.addBLELog(`第 ${attempt} 次配置异常: ${e.message}`);
            }
        }

        const fallbackFs = this._fallbackToCurrentBleRate(
            lastMeasuredFs > 0 ? `实测约 ${lastMeasuredFs.toFixed(1)} Hz` : '未检测到 50Hz'
        );
        this.addBLELog(`ℹ️ 蓝牙保持连接，当前按 ${fallbackFs} Hz 继续工作；如需更高频率可手动重试 50Hz`);
    }

    /**
     * 通过 FFF2 发送 AT 指令设置设备采样周期
     * 协议：$AT+Tcycle=<ms>*\r\n
     * 示例：sendTcycleCommand(20) → 50 Hz；sendTcycleCommand(500) → 2 Hz（设备默认）
     */
    async sendTcycleCommand(periodMs, verifyAfterSend = true) {
        if (!this.bleConnected) {
            this.addBLELog('❌ 未连接蓝牙，无法发送指令'); return;
        }
        if (!Number.isFinite(periodMs) || periodMs < 10 || periodMs > 10000) {
            this.addBLELog('❌ 无效周期值（范围 10–10000 ms）'); return;
        }
        try {
            const cmd = `$AT+Tcycle=${periodMs}*
`;
            await window.BLE.send(cmd);
            const newFs = Math.round(1000 / periodMs);
            // A successful GATT write only proves that the command was sent;
            // it does not prove that firmware applied the requested period.
            // Keep the command target separate from the trusted processing fs
            // until the resulting frame stream has been measured.
            this.bleTargetFs = newFs;
            this.bleConfigStatus = 'configuring';
            this._beginVitalsSamplingEpoch(`Tcycle目标切换为 ${newFs} Hz，等待实测确认`);
            this.resetBleStats();
            this._updateConfigStatusUI();
            this.addBLELog(`📤 通过 ${this._getBleProtocolLabel()} 发送: ${cmd.trim()} → 目标 ${newFs} Hz，等待实测验证`);

            // Automatic configuration performs its own retry measurement.
            // Manual Tcycle buttons use this verification path.
            if (verifyAfterSend) {
                const countBefore = this.bleStats.received;
                const startedAt = Date.now();
                await new Promise(resolve => setTimeout(resolve, 2500));
                const elapsedSeconds = Math.max(0.001, (Date.now() - startedAt) / 1000);
                const measuredFs = (this.bleStats.received - countBefore) / elapsedSeconds;
                const ratio = measuredFs / Math.max(newFs, 1);
                if (ratio >= 0.8 && ratio <= 1.2) {
                    this._applyBleSamplingRate(newFs);
                    this.bleConfigStatus = 'configured';
                    this.addBLELog(`✅ Tcycle已验证：实测 ${measuredFs.toFixed(1)} Hz，算法采用 ${newFs} Hz`);
                } else if (measuredFs >= 1) {
                    const adoptedFs = this._applyBleSamplingRate(measuredFs, { updateTarget: false });
                    this.bleConfigStatus = ratio > 1.2 ? 'configured' : 'fallback';
                    this.addBLELog(
                        `⚠️ Tcycle目标 ${newFs} Hz 与实测 ${measuredFs.toFixed(1)} Hz 不一致，`
                        + `算法采用实测 ${adoptedFs} Hz`
                    );
                } else {
                    this.bleConfigStatus = 'failed';
                    this.addBLELog('❌ Tcycle发送后未获得足够数据，生命体征采样率未验证');
                }
                this._updateConfigStatusUI();
            }
        } catch (err) {
            this._fallbackToCurrentBleRate(`发送指令失败: ${err.message}`);
        }
    }

    resetBleStats() {
        this.bleStats = {
            startRxTs: 0, lastRxTs: 0,
            startRxMonoTs: 0, lastRxMonoTs: 0,
            received: 0, expected: 0, missed: 0,
            lastGapMs: 0, gapEmaMs: 0, gapJitterEmaMs: 0,
            lastSeq: null, seqBased: false
        };
        this._lastGpsTs = null;
        this._gpsGapMissed = 0;
    }


    // 这些函数已被蓝牙专用函数取代，保留作为兼容性
    updateLiveCharts() {
        // 现在由 updateBluetoothLiveCharts() 处理蓝牙数据
        // 文件数据由 updateCharts() 处理
        console.log('updateLiveCharts: 已弃用，请使用 updateBluetoothLiveCharts');
    }

    updateLiveVitalFromBuffer() {
        // 现在由 updateBluetoothVitalSigns() 处理蓝牙数据
        // 文件数据由正常的文件处理流程处理
        console.log('updateLiveVitalFromBuffer: 已弃用，请使用 updateBluetoothVitalSigns');
    }

    /**
     * 处理文件选择
     */
    handleFileSelect(event) {
        const files = Array.from(event.target.files);
        const validFiles = files.filter(file =>
            file.name.toLowerCase().endsWith('.txt') ||
            file.name.toLowerCase().endsWith('.json')
        );

        if (validFiles.length === 0) {
            this.showMessage('请选择.txt或.json格式的数据文件', 'warning');
            return;
        }

        this.selectedFiles = validFiles;
        this.displayFileList();
        this.showMessage(`已选择 ${validFiles.length} 个文件`, 'success');
    }

    /**
     * 显示文件列表
     */
    displayFileList() {
        const fileList = document.getElementById('fileList');
        const fileItems = document.getElementById('fileItems');
        
        fileItems.innerHTML = '';
        
        this.selectedFiles.forEach((file, index) => {
            const li = document.createElement('li');
            li.innerHTML = `
                <span>${file.name}</span>
                <span>${this.formatFileSize(file.size)}</span>
            `;
            fileItems.appendChild(li);
        });
        
        fileList.style.display = 'block';
    }

    /**
     * 清空文件列表
     */
    clearFiles() {
        this.selectedFiles = [];
        document.getElementById('fileList').style.display = 'none';
        document.getElementById('fileInput').value = '';
        this.hideResults();
    }

    /**
     * 处理文件
     */
    async processFiles() {
        if (this.selectedFiles.length === 0) {
            this.showMessage('请先选择文件', 'warning');
            return;
        }

        this.showLoading(true);
        this.showStatus(true);
        this.processedResults = [];

        const totalFiles = this.selectedFiles.length;
        let processedCount = 0;

        for (const file of this.selectedFiles) {
            try {
                this.updateProgress(processedCount / totalFiles * 100, 
                    `正在处理: ${file.name}`);
                
                this.addStatusLog(`开始处理文件: ${file.name}`);
                
                // 读取文件内容
                const fileContent = await this.readFileContent(file);

                // 处理数据
                let result;
                if (file.name.toLowerCase().endsWith('.json')) {
                    result = this.processJsonFile(file.name, fileContent);
                } else {
                    const offline = this.offlineProcessor
                        || new RadarDataProcessor(this.offlineSamplingRate || 50);
                    this.offlineProcessor = offline;
                    result = offline.processSingleFile(file.name, fileContent);
                }
                this.processedResults.push(result);
                
                if (result.status === 'success') {
                    if (result.dataType === 'json') {
                        this.addStatusLog(`✓ ${file.name} 处理成功 - 动物: ${result.animal.name}(${result.animal.species}), 心率: ${result.heartRate} bpm, 呼吸: ${result.respiratoryRate} bpm`);
                    } else {
                        this.addStatusLog(`✓ ${file.name} 处理成功 - 心率: ${result.heartRate} bpm, 呼吸: ${result.respiratoryRate} bpm`);
                    }
                } else {
                    this.addStatusLog(`✗ ${file.name} 处理失败: ${result.error}`);
                }
                
                processedCount++;
                
            } catch (error) {
                this.addStatusLog(`✗ ${file.name} 处理出错: ${error.message}`);
                this.processedResults.push({
                    fileName: file.name,
                    error: error.message,
                    status: 'error'
                });
                processedCount++;
            }
        }

        this.updateProgress(100, '处理完成');
        this.showLoading(false);
        
        // 显示结果
        this.displayResults();
        this.showMessage(`处理完成！成功处理 ${this.processedResults.filter(r => r.status === 'success').length} 个文件`, 'success');
    }

    /**
     * 读取文件内容
     */
    readFileContent(file) {
        return new Promise((resolve, reject) => {
            const reader = new FileReader();
            reader.onload = (e) => resolve(e.target.result);
            reader.onerror = (e) => reject(new Error('文件读取失败'));
            reader.readAsText(file, 'utf-8');
        });
    }

    /**
     * 处理JSON格式的传感器数据文件
     */
    processJsonFile(fileName, jsonContent) {
        try {
            const data = JSON.parse(jsonContent);

            // 验证数据结构
            if (!data.event_id || !data.animal || !data.signals) {
                return {
                    fileName: fileName,
                    status: 'error',
                    error: 'JSON格式不正确，缺少必要字段'
                };
            }

            // 提取动物信息
            const animal = data.animal;
            const device = data.device || {};
            const vitals = data.signals.vitals || { samples: [] };
            const accel = data.signals.accel || { samples: [] };
            const temperature = data.signals.temperature || { samples: [] };

            // 计算统计信息
            const hrValues = vitals.samples.map(s => s.hr).filter(hr => hr && hr > 0);
            const rrValues = vitals.samples.map(s => s.rr).filter(rr => rr && rr > 0);
            const tempValues = temperature.samples.map(s => s.value).filter(temp => temp && temp > 0);

            const avgHeartRate = hrValues.length > 0 ? hrValues.reduce((a, b) => a + b, 0) / hrValues.length : 0;
            const avgRespRate = rrValues.length > 0 ? rrValues.reduce((a, b) => a + b, 0) / rrValues.length : 0;
            const avgTemp = tempValues.length > 0 ? tempValues.reduce((a, b) => a + b, 0) / tempValues.length : 0;

            return {
                fileName: fileName,
                status: 'success',
                dataType: 'json',
                animal: animal,
                device: device,
                heartRate: Math.round(avgHeartRate * 10) / 10,
                respiratoryRate: Math.round(avgRespRate * 10) / 10,
                temperature: Math.round(avgTemp * 10) / 10,
                dataPoints: Math.max(vitals.samples.length, accel.samples.length, temperature.samples.length),
                hrData: hrValues,
                rrData: rrValues,
                tempData: tempValues,
                rawData: data
            };

        } catch (error) {
            return {
                fileName: fileName,
                status: 'error',
                error: `JSON解析失败: ${error.message}`
            };
        }
    }

    /**
     * 显示处理结果
     */
    displayResults() {
        const successResults = this.processedResults.filter(r => r.status === 'success');

        if (successResults.length === 0) {
            this.showMessage('没有成功处理的文件', 'warning');
            return;
        }

        // 检查是否有JSON数据
        const hasJsonData = successResults.some(r => r.dataType === 'json');

        // 更新统计信息
        this.updateStatistics(successResults);

        // 更新图表
        this.updateCharts(successResults);

        // 更新结果表格
        this.updateResultsTable();

        // 显示JSON数据详细信息
        if (hasJsonData) {
            this.displayJsonDataDetails(successResults);
            document.getElementById('jsonDataSection').style.display = 'block';
            document.getElementById('healthAnalysisSection').style.display = 'block';
        } else {
            document.getElementById('jsonDataSection').style.display = 'none';
            document.getElementById('healthAnalysisSection').style.display = 'none';
        }

        // JSON时隐藏部分图表，仅保留心率/呼吸时间序列
        this.setChartVisibilityForJson(hasJsonData);

        // 显示结果区域
        document.getElementById('resultsSection').style.display = 'block';
        document.getElementById('resultsSection').classList.add('fade-in');
    }

    /**
     * 显示JSON数据的详细信息
     */
    displayJsonDataDetails(results) {
        const jsonResults = results.filter(r => r.dataType === 'json');
        if (jsonResults.length === 0) return;

        // 使用最新的JSON数据（如果有多个，取第一个）
        const latestResult = jsonResults[0];
        const animal = latestResult.animal;
        const device = latestResult.device;
        const rawData = latestResult.rawData;

        // 更新动物信息
        const animalEmoji = animal.species === 'dog' ? '🐕' : animal.species === 'cat' ? '🐱' : '🐾';
        document.getElementById('animalEmoji').textContent = animalEmoji;
        document.getElementById('animalName').textContent = animal.name || '未命名宠物';
        document.getElementById('animalBasicInfo').textContent =
            `${animal.breed || '未知品种'} · ${animal.age_months ? Math.floor(animal.age_months / 12) + '岁' + (animal.age_months % 12) + '个月' : '年龄未知'} · ${animal.sex === 'male' ? '公' : animal.sex === 'female' ? '母' : '性别未知'}`;
        document.getElementById('animalWeight').textContent = animal.weight_kg ? `${animal.weight_kg} kg` : '-- kg';
        document.getElementById('animalId').textContent = animal.animal_id || '--';

        // 更新设备信息
        document.getElementById('deviceId').textContent = device.device_id || '--';
        document.getElementById('deviceFirmware').textContent = device.firmware || '--';

        const samplingInfo = device.sampling_hz ?
            `心率:${device.sampling_hz.vitals || '--'}/秒, 加速度:${device.sampling_hz.accel || '--'}Hz, 温度:${device.sampling_hz.temp || '--'}/秒` : '--';
        document.getElementById('deviceSampling').textContent = samplingInfo;

        // 更新测量信息
        document.getElementById('eventId').textContent = rawData.event_id || '--';

        const eventTime = rawData.ts ? new Date(rawData.ts).toLocaleString('zh-CN') : '--';
        document.getElementById('measurementTime').textContent = eventTime;

        const window = rawData.window;
        if (window && window.start_ts && window.end_ts) {
            const startTime = new Date(window.start_ts);
            const endTime = new Date(window.end_ts);
            const duration = Math.round((endTime - startTime) / 1000);
            document.getElementById('measurementDuration').textContent = `${duration} 秒`;
        } else {
            document.getElementById('measurementDuration').textContent = '--';
        }

        const context = rawData.context || {};
        const location = context.location ?
            `${context.location.lat}, ${context.location.lng}` : '--';
        document.getElementById('measurementLocation').textContent = location;

        document.getElementById('measurementNotes').textContent = context.notes || '--';

        // 设置默认的agent endpoint
        const agentEndpointEl = document.getElementById('agentEndpoint');
        if (agentEndpointEl && !agentEndpointEl.value) {
            agentEndpointEl.value = localStorage.getItem('agentEndpoint') || 'http://localhost:8000';
        }
    }

    /**
     * 执行宠物健康分析
     */
    async performHealthAnalysis() {
        const jsonResults = this.processedResults.filter(r => r.dataType === 'json');
        if (jsonResults.length === 0) {
            this.showMessage('没有找到可分析的JSON数据', 'warning');
            return;
        }

        const agentEndpoint = document.getElementById('agentEndpoint').value.trim();
        if (!agentEndpoint) {
            this.showMessage('请设置Agent API地址', 'warning');
            return;
        }

        // 保存endpoint到localStorage
        localStorage.setItem('agentEndpoint', agentEndpoint);

        const result = jsonResults[0]; // 使用第一个JSON结果
        const analysisBtn = document.getElementById('healthAnalysisBtn');
        const reportContainer = document.getElementById('healthAnalysisReport');
        const reportContent = document.getElementById('analysisReportContent');

        // 显示分析界面
        reportContainer.style.display = 'block';
        analysisBtn.disabled = true;
        analysisBtn.textContent = '🔄 分析中...';

        reportContent.innerHTML = `
            <div class="loading-analysis">
                <div class="loading-spinner"></div>
                <p>正在分析宠物健康状况，请稍候...</p>
            </div>
        `;

        try {
            // 构建健康分析查询
            const query = this.buildHealthAnalysisQuery(result);

            // 调用agent API
            const response = await this.fetchWithTimeout(`${agentEndpoint}/agent/plan_and_solve`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({
                    query: query,
                    llm_base_url: 'https://api.openai.com/v1',
                    llm_api_key:  process.env.OPENAI_API_KEY, // 需要用户配置
                    llm_model: 'deepseek-chat',
                    allowed_tools: ['rag.search'],
                    temperature: 0.7,
                    max_tokens: 2000
                })
            }, 300000);

            if (!response.ok) {
                throw new Error(`Agent API请求失败: ${response.status}`);
            }

            const analysisResult = await response.json();

            if (!analysisResult.ok) {
                throw new Error(analysisResult.error?.message || '分析失败');
            }

            // 显示分析结果
            this.displayHealthAnalysisResult(analysisResult, result);

        } catch (error) {
            console.error('健康分析失败:', error);
            reportContent.innerHTML = `
                <div class="analysis-error">
                    <h4>❌ 分析失败</h4>
                    <p>错误信息: ${error.message}</p>
                    <p>请检查Agent API地址和配置是否正确。</p>
                </div>
            `;
        } finally {
            analysisBtn.disabled = false;
            analysisBtn.textContent = '🩺 开始健康分析';
        }
    }

    /**
     * 构建健康分析查询
     */
    buildHealthAnalysisQuery(result) {
        const animal = result.animal;
        const vitals = result.rawData.signals.vitals || { samples: [] };
        const context = result.rawData.context || {};

        const avgHR = result.heartRate;
        const avgRR = result.respiratoryRate;
        const temp = result.temperature;

        let query = `请分析这只${animal.species === 'dog' ? '狗狗' : '猫咪'}的健康状况：

宠物信息：
- 姓名: ${animal.name || '未命名'}
- 品种: ${animal.breed || '未知'}
- 年龄: ${animal.age_months ? Math.floor(animal.age_months / 12) + '岁' + (animal.age_months % 12) + '个月' : '未知'}
- 体重: ${animal.weight_kg || '未知'}kg
- 性别: ${animal.sex === 'male' ? '公' : animal.sex === 'female' ? '母' : '未知'}

生理指标：
- 平均心率: ${avgHR} bpm
- 平均呼吸频率: ${avgRR} bpm
- 体温: ${temp}°C

测量情况：
- 位置: ${context.location ? `${context.location.lat}, ${context.location.lng}` : '未知'}
- 备注: ${context.notes || '无'}
- 标签: ${context.tags ? context.tags.join(', ') : '无'}

请基于这些数据分析宠物的健康状况，包括：
1. 心率和呼吸频率是否正常
2. 体温是否正常
3. 整体健康评估
4. 如果有异常，建议采取什么措施
5. 日常护理建议

请提供详细的分析报告。`;

        return query;
    }

    /**
     * 显示健康分析结果
     */
    displayHealthAnalysisResult(analysisResult, originalData) {
        const timestamp = new Date().toLocaleString('zh-CN');
        document.getElementById('analysisTimestamp').textContent = `分析时间: ${timestamp}`;

        const reportContent = document.getElementById('analysisReportContent');

        // 格式化分析结果
        const answer = analysisResult.answer || '暂无分析结果';
        const plan = analysisResult.plan || [];
        const toolResults = analysisResult.tool_results || [];

        reportContent.innerHTML = `
            <div class="analysis-summary">
                <h4>📊 分析总结</h4>
                <div class="analysis-content">${this.formatAnalysisText(answer)}</div>
            </div>

            ${plan.length > 0 ? `
            <div class="analysis-plan" style="margin-top: 20px;">
                <h4>🔍 分析过程</h4>
                <ol>
                    ${plan.map(step => `<li><strong>${step.type === 'tool' ? '工具调用' : '推理'}:</strong> ${step.note || step.tool_name || '未知步骤'}</li>`).join('')}
                </ol>
            </div>
            ` : ''}

            ${toolResults.length > 0 ? `
            <div class="tool-results" style="margin-top: 20px;">
                <h4>📚 参考资料</h4>
                ${toolResults.map((result, index) => `
                    <div class="tool-result-item">
                        <h5>工具 ${index + 1}: ${result.tool_name}</h5>
                        <div class="tool-content">${this.formatToolResult(result)}</div>
                    </div>
                `).join('')}
            </div>
            ` : ''}
        `;

        this.showMessage('健康分析完成！', 'success');
    }

    /**
     * 格式化分析文本
     */
    formatAnalysisText(text) {
        if (!text) return '暂无内容';

        // 简单的文本格式化，转换换行符和列表
        return text
            .replace(/\n/g, '<br/>')
            .replace(/(\d+)\.\s/g, '<br/>$1. ')
            .replace(/^(\d+)\.\s/gm, '<br/>$1. ');
    }

    /**
     * 格式化工具结果
     */
    formatToolResult(result) {
        if (!result || !result.data) return '暂无数据';

        try {
            const data = typeof result.data === 'string' ? JSON.parse(result.data) : result.data;

            if (result.tool_name === 'rag.search' && data.results) {
                return data.results.map(item =>
                    `<div class="rag-item">
                        <strong>相关度: ${item.score ? item.score.toFixed(3) : '未知'}</strong><br/>
                        ${item.content || item.text || '无内容'}
                    </div>`
                ).join('');
            }

            return JSON.stringify(data, null, 2);
        } catch (e) {
            return result.data;
        }
    }

    /**
     * 导出健康报告
     */
    exportHealthReport() {
        const reportContent = document.getElementById('analysisReportContent');
        if (!reportContent) {
            this.showMessage('没有可导出的报告', 'warning');
            return;
        }

        const reportText = reportContent.innerText || reportContent.textContent;
        const timestamp = new Date().toISOString().slice(0, 19).replace(/:/g, '-');

        const blob = new Blob([reportText], { type: 'text/plain;charset=utf-8' });
        const url = URL.createObjectURL(blob);

        const a = document.createElement('a');
        a.href = url;
        a.download = `宠物健康分析报告_${timestamp}.txt`;
        document.body.appendChild(a);
        a.click();
        document.body.removeChild(a);
        URL.revokeObjectURL(url);

        this.showMessage('报告已导出！', 'success');
    }

    /**
     * 更新统计信息
     */
    updateStatistics(results) {
        const heartRates = results.map(r => r.heartRate).filter(hr => hr > 0);
        const respRates = results.map(r => r.respiratoryRate).filter(rr => rr > 0);
        const totalDataPoints = results.reduce((sum, r) => sum + r.dataPoints, 0);

        document.getElementById('avgHeartRate').textContent = 
            heartRates.length > 0 ? `${(heartRates.reduce((a, b) => a + b, 0) / heartRates.length).toFixed(1)} bpm` : '-- bpm';
        
        document.getElementById('avgRespRate').textContent = 
            respRates.length > 0 ? `${(respRates.reduce((a, b) => a + b, 0) / respRates.length).toFixed(1)} bpm` : '-- bpm';
        
        document.getElementById('processedFiles').textContent = results.length;
        document.getElementById('totalDataPoints').textContent = totalDataPoints.toLocaleString();
    }

    /**
     * 初始化蓝牙图表
     */
    initializeBluetoothCharts() {
        if (typeof Chart === 'undefined') {
            console.error('Chart.js 未加载，蓝牙图表初始化已跳过。');
            return;
        }

        const requiredIds = [
            'bleISignalChart',
            'bleQSignalChart',
            'bleConstellationChart',
            'bleRespiratoryChart',
            'bleHeartbeatChart',
            'bleDynamicACCChart'
        ];
        const missing = requiredIds.filter(id => !document.getElementById(id));
        if (missing.length > 0) {
            console.warn('蓝牙图表DOM缺失，跳过初始化:', missing);
            return;
        }

        // 销毁已有图表，防止 "Canvas is already in use" 错误
        Object.keys(this.bleCharts).forEach(key => {
            try {
                if (this.bleCharts[key] && typeof this.bleCharts[key].destroy === 'function') {
                    this.bleCharts[key].destroy();
                }
            } catch (e) { /* ignore */ }
        });
        this.bleCharts = {};

        const chartOptions = {
            responsive: true,
            maintainAspectRatio: false,
            plugins: {
                legend: {
                    display: true,
                    position: 'top'
                }
            },
            scales: {
                x: {
                    display: true,
                    title: { display: true }
                },
                y: {
                    display: true,
                    title: { display: true }
                }
            },
            animation: false // 关闭动画以提高实时性能
        };

        // 初始化蓝牙专用图表
        // I 通道 - 放大显示微小变化
        this.bleCharts.iSignal = new Chart(document.getElementById('bleISignalChart'), {
            type: 'line',
            data: { labels: [], datasets: [] },
            options: {
                ...chartOptions,
                plugins: { ...chartOptions.plugins, title: { display: true, text: '蓝牙 I 通道实时信号 (自适应放大)' } },
                scales: {
                    x: { display: true, title: { display: true, text: '采样点' } },
                    y: {
                        display: true,
                        title: { display: true, text: '幅度 (V)' },
                        min: 1.2,    // 扩大初始范围，更清楚显示波峰变化
                        max: 2.8,    // 适度范围以突出波峰细节
                        beginAtZero: false
                    }
                }
            }
        });

        // Q 通道 - 放大显示微小变化
        this.bleCharts.qSignal = new Chart(document.getElementById('bleQSignalChart'), {
            type: 'line',
            data: { labels: [], datasets: [] },
            options: {
                ...chartOptions,
                plugins: { ...chartOptions.plugins, title: { display: true, text: '蓝牙 Q 通道实时信号 (自适应放大)' } },
                scales: {
                    x: { display: true, title: { display: true, text: '采样点' } },
                    y: {
                        display: true,
                        title: { display: true, text: '幅度 (V)' },
                        min: 1.2,    // 扩大初始范围，更清楚显示波峰变化
                        max: 2.8,    // 适度范围以突出波峰细节
                        beginAtZero: false
                    }
                }
            }
        });

        this.bleCharts.constellation = new Chart(document.getElementById('bleConstellationChart'), {
            type: 'scatter',
            data: { datasets: [] },
            options: { 
                ...chartOptions, 
                plugins: { ...chartOptions.plugins, title: { display: true, text: '蓝牙 I/Q 星座图' } },
                scales: {
                    x: { title: { display: true, text: 'I通道' } },
                    y: { title: { display: true, text: 'Q通道' } }
                }
            }
        });

        this.bleCharts.respiratory = new Chart(document.getElementById('bleRespiratoryChart'), {
            type: 'line',
            data: { labels: [], datasets: [] },
            options: { ...chartOptions, plugins: { ...chartOptions.plugins, title: { display: true, text: '蓝牙呼吸波形' } } }
        });

        this.bleCharts.heartbeat = new Chart(document.getElementById('bleHeartbeatChart'), {
            type: 'line',
            data: { labels: [], datasets: [] },
            options: { ...chartOptions, plugins: { ...chartOptions.plugins, title: { display: true, text: '蓝牙心跳波形' } } }
        });

        // 初始化 IMU(Gx/Gy/Gz) 图表
        const imuCanvas = document.getElementById('bleIMUChart');
        if (imuCanvas) {
            this.bleCharts.imu = new Chart(imuCanvas, {
                type: 'line',
                data: { labels: [], datasets: [] },
                options: { ...chartOptions, plugins: { ...chartOptions.plugins, title: { display: true, text: '蓝牙 Gx/Gy/Gz 三轴变化' } } }
            });
        }

        // 初始化加速度计(Ax/Ay/Az) 图表
        const accCanvas = document.getElementById('bleACCChart');
        if (accCanvas) {
            this.bleCharts.acc = new Chart(accCanvas, {
                type: 'line',
                data: { labels: [], datasets: [] },
                options: {
                    ...chartOptions,
                    plugins: { ...chartOptions.plugins, title: { display: true, text: '蓝牙 Ax/Ay/Az 原始信号 (m/s²，自适应放大)' } },
                    scales: {
                        x: { display: true, title: { display: true, text: '采样点' } },
                        y: { display: true, title: { display: true, text: '加速度 (m/s²)' } }
                    }
                }
            });
        }

        const dynamicAccCanvas = document.getElementById('bleDynamicACCChart');
        if (dynamicAccCanvas) {
            this.bleCharts.accDynamic = new Chart(dynamicAccCanvas, {
                type: 'line',
                data: { labels: [], datasets: [] },
                options: {
                    ...chartOptions,
                    plugins: { ...chartOptions.plugins, title: { display: true, text: '蓝牙 Ax/Ay/Az 动态变化 (m/s²，去基线)' } },
                    scales: {
                        x: { display: true, title: { display: true, text: '采样点' } },
                        y: { display: true, title: { display: true, text: '动态加速度 (m/s²)' } }
                    }
                }
            });
        }

        // 初始化温度图表
        const tempCanvas = document.getElementById('bleTemperatureChart');
        if (tempCanvas) {
            this.bleCharts.temperature = new Chart(tempCanvas, {
                type: 'line',
                data: { labels: [], datasets: [] },
                options: { 
                    ...chartOptions, 
                    plugins: { ...chartOptions.plugins, title: { display: true, text: '蓝牙 温度变化 (°C)' } },
                    scales: {
                        x: { display: true, title: { display: true, text: '时间' } },
                        y: { 
                            display: true, 
                            title: { display: true, text: '温度 (°C)' },
                            min: 15, // 最小温度15°C
                            max: 45  // 最大温度45°C
                        }
                    }
                }
            });
        }
    }

    initializeBleBackfillCharts() {
        if (typeof Chart === 'undefined') return;

        const requiredIds = [
            'bleBackfillIQChart',
            'bleBackfillLagChart',
            'bleBackfillIMUChart',
            'bleBackfillACCChart'
        ];
        const missing = requiredIds.filter(id => !document.getElementById(id));
        if (missing.length > 0) return;

        Object.keys(this.bleBackfillCharts).forEach(key => {
            try {
                if (this.bleBackfillCharts[key] && typeof this.bleBackfillCharts[key].destroy === 'function') {
                    this.bleBackfillCharts[key].destroy();
                }
            } catch (_) {}
        });
        this.bleBackfillCharts = {};

        const chartOptions = {
            responsive: true,
            maintainAspectRatio: false,
            plugins: {
                legend: {
                    display: true,
                    position: 'top'
                }
            },
            scales: {
                x: { display: true, title: { display: true } },
                y: { display: true, title: { display: true } }
            },
            animation: false
        };

        this.bleBackfillCharts.iq = new Chart(document.getElementById('bleBackfillIQChart'), {
            type: 'line',
            data: { labels: [], datasets: [] },
            options: {
                ...chartOptions,
                plugins: { ...chartOptions.plugins, title: { display: true, text: '补传 I/Q 数据' } },
                scales: {
                    x: { display: true, title: { display: true, text: '样本序号' } },
                    y: { display: true, title: { display: true, text: '幅度 / 电压' } }
                }
            }
        });

        this.bleBackfillCharts.lag = new Chart(document.getElementById('bleBackfillLagChart'), {
            type: 'line',
            data: { labels: [], datasets: [] },
            options: {
                ...chartOptions,
                plugins: { ...chartOptions.plugins, title: { display: true, text: '补传追平进度（设备时间滞后）' } },
                scales: {
                    x: { display: true, title: { display: true, text: '样本序号' } },
                    y: { display: true, title: { display: true, text: '滞后 (秒)' }, beginAtZero: true }
                }
            }
        });

        this.bleBackfillCharts.imu = new Chart(document.getElementById('bleBackfillIMUChart'), {
            type: 'line',
            data: { labels: [], datasets: [] },
            options: {
                ...chartOptions,
                plugins: { ...chartOptions.plugins, title: { display: true, text: '补传 Gx/Gy/Gz' } }
            }
        });

        this.bleBackfillCharts.acc = new Chart(document.getElementById('bleBackfillACCChart'), {
            type: 'line',
            data: { labels: [], datasets: [] },
            options: {
                ...chartOptions,
                plugins: { ...chartOptions.plugins, title: { display: true, text: '补传 Ax/Ay/Az' } }
            }
        });
    }

    /**
     * 初始化蓝牙动态ECG画布
     */
    initializeBLEECG() {
        const resCanvas = document.getElementById('bleRespiratoryECGCanvas');
        const hbCanvas = document.getElementById('bleHeartbeatECGCanvas');
        if (!resCanvas || !hbCanvas) return;

        const ctxRes = resCanvas.getContext('2d');
        const ctxHb = hbCanvas.getContext('2d');
        this._bleECG = {
            res: { canvas: resCanvas, ctx: ctxRes, data: [], playing: false, cursor: 0 },
            hb:  { canvas: hbCanvas,  ctx: ctxHb,  data: [], playing: false, cursor: 0 },
            raf: null
        };

        const draw = () => {
            const { res, hb } = this._bleECG;
            [res, hb].forEach(track => {
                const { canvas, ctx } = track;
                const w = canvas.width = canvas.clientWidth || 600;
                const h = canvas.height = canvas.clientHeight || 160;
                ctx.clearRect(0, 0, w, h);
                ctx.strokeStyle = '#0aa'; ctx.lineWidth = 2; ctx.beginPath();
                const len = track.data.length;
                const view = 1000;
                const start = Math.max(0, len - view);
                for (let i = start; i < len; i++) {
                    const x = (i - start) / view * w;
                    const y = h/2 - (track.data[i] || 0) * (h*0.4);
                    if (i === start) ctx.moveTo(x, y); else ctx.lineTo(x, y);
                }
                ctx.stroke();
            });
            if (this._bleECG.res.playing || this._bleECG.hb.playing) {
                this._bleECG.raf = requestAnimationFrame(draw);
            } else {
                cancelAnimationFrame(this._bleECG.raf);
                this._bleECG.raf = null;
            }
        };

        this._bleECG.draw = draw;
    }

    /**
     * 初始化文件数据的动态ECG画布
     */
    initializeFileECG() {
        const resCanvas = document.getElementById('respiratoryECGCanvas');
        const hbCanvas = document.getElementById('heartbeatECGCanvas');
        if (!resCanvas || !hbCanvas) return;

        const ctxRes = resCanvas.getContext('2d');
        const ctxHb = hbCanvas.getContext('2d');

        // 从处理结果中获取数据
        const firstResult = this.processedResults.find(r => r.respiratoryWave && r.heartbeatWave);
        if (!firstResult) return;

        this._fileECG = {
            res: {
                canvas: resCanvas,
                ctx: ctxRes,
                data: Array.from(firstResult.respiratoryWave),
                playing: false,
                cursor: 0
            },
            hb: {
                canvas: hbCanvas,
                ctx: ctxHb,
                data: Array.from(firstResult.heartbeatWave),
                playing: false,
                cursor: 0
            },
            raf: null
        };

        const draw = () => {
            const { res, hb } = this._fileECG;

            // 绘制呼吸波形
            [res, hb].forEach(track => {
                const { canvas, ctx, data, cursor } = track;
                const w = canvas.width = canvas.clientWidth || 600;
                const h = canvas.height = canvas.clientHeight || 160;
                ctx.clearRect(0, 0, w, h);

                // 绘制网格
                ctx.strokeStyle = '#e0e0e0';
                ctx.lineWidth = 1;
                for (let x = 0; x < w; x += 20) {
                    ctx.beginPath();
                    ctx.moveTo(x, 0);
                    ctx.lineTo(x, h);
                    ctx.stroke();
                }
                for (let y = 0; y < h; y += 20) {
                    ctx.beginPath();
                    ctx.moveTo(0, y);
                    ctx.lineTo(w, y);
                    ctx.stroke();
                }

                // 绘制波形
                if (data.length > 0) {
                    ctx.strokeStyle = track === res ? '#28a745' : '#dc3545';
                    ctx.lineWidth = 2;
                    ctx.beginPath();

                    const displayPoints = Math.min(200, data.length);
                    const startIdx = Math.max(0, cursor - displayPoints);

                    for (let i = 0; i < displayPoints && startIdx + i < data.length; i++) {
                        const x = (i / displayPoints) * w;
                        const value = data[startIdx + i];
                        const y = h/2 - (value * h/4); // 缩放并居中

                        if (i === 0) {
                            ctx.moveTo(x, y);
                        } else {
                            ctx.lineTo(x, y);
                        }
                    }
                    ctx.stroke();
                }
            });

            // 更新游标
            if (res.playing || hb.playing) {
                this._fileECG.res.cursor = (this._fileECG.res.cursor + 1) % Math.max(1, this._fileECG.res.data.length);
                this._fileECG.hb.cursor = (this._fileECG.hb.cursor + 1) % Math.max(1, this._fileECG.hb.data.length);
                this._fileECG.raf = requestAnimationFrame(draw);
            } else {
                cancelAnimationFrame(this._fileECG.raf);
                this._fileECG.raf = null;
            }
        };

        this._fileECG.draw = draw;
    }

    /**
     * 初始化图表
     */
    initializeCharts() {
        if (typeof Chart === 'undefined') {
            this._chartsEnabled = false;
            console.error('Chart.js 未加载，图表功能不可用。');
            return;
        }

        const requiredIds = [
            'iSignalChart',
            'qSignalChart',
            'constellationChart',
            'respiratoryChart',
            'heartbeatChart',
            'heartRateChart',
            'respRateChart',
            'heartRateTimeChart',
            'respRateTimeChart'
        ];
        const missing = requiredIds.filter(id => !document.getElementById(id));
        if (missing.length > 0) {
            this._chartsEnabled = false;
            console.warn('图表DOM缺失，跳过初始化:', missing);
            return;
        }
        this._chartsEnabled = true;

        // 销毁已有图表，防止 "Canvas is already in use" 错误
        Object.keys(this.charts).forEach(key => {
            try {
                if (this.charts[key] && typeof this.charts[key].destroy === 'function') {
                    this.charts[key].destroy();
                }
            } catch (e) { /* ignore */ }
        });
        this.charts = {};

        const chartOptions = {
            responsive: true,
            maintainAspectRatio: false,
            plugins: {
                legend: {
                    display: true,
                    position: 'top'
                }
            },
            scales: {
                x: {
                    display: true,
                    title: {
                        display: true
                    }
                },
                y: {
                    display: true,
                    title: {
                        display: true
                    }
                }
            }
        };

        // 初始化所有图表
        // I 通道图表
        this.charts.iSignal = new Chart(document.getElementById('iSignalChart'), {
            type: 'line',
            data: { labels: [], datasets: [] },
            options: {
                ...chartOptions,
                plugins: { ...chartOptions.plugins, title: { display: true, text: 'I 通道信号 (放大显示)' } },
                scales: {
                    x: { display: true, title: { display: true, text: '采样点' } },
                    y: {
                        display: true,
                        title: { display: true, text: '幅度 (V)' },
                        beginAtZero: false,
                        // 动态放大范围以显示更多细节
                        ticks: {
                            callback: function(value, index, values) {
                                return value.toFixed(4); // 显示更多小数位以观察微小变化
                            }
                        }
                    }
                }
            }
        });

        // Q 通道图表
        this.charts.qSignal = new Chart(document.getElementById('qSignalChart'), {
            type: 'line',
            data: { labels: [], datasets: [] },
            options: {
                ...chartOptions,
                plugins: { ...chartOptions.plugins, title: { display: true, text: 'Q 通道信号 (放大显示)' } },
                scales: {
                    x: { display: true, title: { display: true, text: '采样点' } },
                    y: {
                        display: true,
                        title: { display: true, text: '幅度 (V)' },
                        beginAtZero: false,
                        // 动态放大范围以显示更多细节
                        ticks: {
                            callback: function(value, index, values) {
                                return value.toFixed(4); // 显示更多小数位以观察微小变化
                            }
                        }
                    }
                }
            }
        });

        this.charts.constellation = new Chart(document.getElementById('constellationChart'), {
            type: 'scatter',
            data: { datasets: [] },
            options: { 
                ...chartOptions, 
                plugins: { ...chartOptions.plugins, title: { display: true, text: 'I/Q星座图' } },
                scales: {
                    x: { title: { display: true, text: 'I通道' } },
                    y: { title: { display: true, text: 'Q通道' } }
                }
            }
        });

        this.charts.respiratory = new Chart(document.getElementById('respiratoryChart'), {
            type: 'line',
            data: { labels: [], datasets: [] },
            options: { ...chartOptions, plugins: { ...chartOptions.plugins, title: { display: true, text: '呼吸波形' } } }
        });

        this.charts.heartbeat = new Chart(document.getElementById('heartbeatChart'), {
            type: 'line',
            data: { labels: [], datasets: [] },
            options: { ...chartOptions, plugins: { ...chartOptions.plugins, title: { display: true, text: '心跳波形' } } }
        });

        this.charts.heartRate = new Chart(document.getElementById('heartRateChart'), {
            type: 'bar',
            data: { labels: [], datasets: [] },
            options: { 
                ...chartOptions, 
                plugins: { ...chartOptions.plugins, title: { display: true, text: '心率分布' } },
                scales: {
                    x: { title: { display: true, text: '文件' } },
                    y: { title: { display: true, text: '心率 (bpm)' } }
                }
            }
        });

        this.charts.respRate = new Chart(document.getElementById('respRateChart'), {
            type: 'bar',
            data: { labels: [], datasets: [] },
            options: { 
                ...chartOptions, 
                plugins: { ...chartOptions.plugins, title: { display: true, text: '呼吸频率分布' } },
                scales: {
                    x: { title: { display: true, text: '文件' } },
                    y: { title: { display: true, text: '呼吸频率 (bpm)' } }
                }
            }
        });

        // 心率时间序列图表
        this.charts.heartRateTime = new Chart(document.getElementById('heartRateTimeChart'), {
            type: 'line',
            data: { labels: [], datasets: [] },
            options: { 
                ...chartOptions, 
                plugins: { ...chartOptions.plugins, title: { display: true, text: '心率随时间变化' } },
                scales: {
                    x: { title: { display: true, text: '文件序号' } },
                    y: { title: { display: true, text: '心率 (bpm)' } }
                }
            }
        });

        // 呼吸频率时间序列图表
        this.charts.respRateTime = new Chart(document.getElementById('respRateTimeChart'), {
            type: 'line',
            data: { labels: [], datasets: [] },
            options: { 
                ...chartOptions, 
                plugins: { ...chartOptions.plugins, title: { display: true, text: '呼吸频率随时间变化' } },
                scales: {
                    x: { title: { display: true, text: '文件序号' } },
                    y: { title: { display: true, text: '呼吸频率 (bpm)' } }
                }
            }
        });
    }

    /**
     * 更新图表
     */
    updateCharts(results) {
        if (!this._chartsEnabled) return;
        if (results.length === 0) return;

        // 使用第一个成功的结果来显示波形
        const firstResult = results[0];

        // 如果是JSON数据，只更新心率和呼吸率时间序列图
        if (firstResult.dataType === 'json') {
            this.updateJsonCharts(results);
            return;
        }

        // 更新I/Q信号图
        const sampleSize = Math.min(1000, firstResult.iData.length);
        const indices = Array.from({length: sampleSize}, (_, i) => i);

        // 计算I通道数据的统计信息以实现动态放大
        const iDataSlice = firstResult.iData.slice(0, sampleSize);
        const iMin = Math.min(...iDataSlice);
        const iMax = Math.max(...iDataSlice);
        const iRange = iMax - iMin;
        const iPadding = iRange * 0.05; // 5% padding

        // 设置I通道Y轴动态范围（放大显示微小变化）
        const iYAxisMin = iMin - iPadding;
        const iYAxisMax = iMax + iPadding;

        // 更新 I 通道
        this.charts.iSignal.data = {
            labels: indices,
            datasets: [{
                label: 'I通道',
                data: Array.from(iDataSlice),
                borderColor: 'rgb(75, 192, 192)',
                backgroundColor: 'rgba(75, 192, 192, 0.2)',
                tension: 0.1,
                pointRadius: 0
            }]
        };

        // 动态调整I通道Y轴范围以放大显示细节
        if (this.charts.iSignal.options.scales.y) {
            this.charts.iSignal.options.scales.y.min = iYAxisMin;
            this.charts.iSignal.options.scales.y.max = iYAxisMax;
        }

        this.charts.iSignal.update();

        // 计算Q通道数据的统计信息以实现动态放大
        const qDataSlice = firstResult.qData.slice(0, sampleSize);
        const qMin = Math.min(...qDataSlice);
        const qMax = Math.max(...qDataSlice);
        const qRange = qMax - qMin;
        const qPadding = qRange * 0.05; // 5% padding

        // 设置Q通道Y轴动态范围（放大显示微小变化）
        const qYAxisMin = qMin - qPadding;
        const qYAxisMax = qMax + qPadding;

        // 更新 Q 通道
        this.charts.qSignal.data = {
            labels: indices,
            datasets: [{
                label: 'Q通道',
                data: Array.from(qDataSlice),
                borderColor: 'rgb(255, 99, 132)',
                backgroundColor: 'rgba(255, 99, 132, 0.2)',
                tension: 0.1,
                pointRadius: 0
            }]
        };

        // 动态调整Q通道Y轴范围以放大显示细节
        if (this.charts.qSignal.options.scales.y) {
            this.charts.qSignal.options.scales.y.min = qYAxisMin;
            this.charts.qSignal.options.scales.y.max = qYAxisMax;
        }

        this.charts.qSignal.update();

        // 更新星座图
        const constellationSampleSize = Math.min(500, firstResult.iData.length);
        const step = Math.floor(firstResult.iData.length / constellationSampleSize);
        const constellationData = [];
        
        for (let i = 0; i < firstResult.iData.length; i += step) {
            constellationData.push({
                x: firstResult.iData[i],
                y: firstResult.qData[i]
            });
        }

        this.charts.constellation.data = {
            datasets: [
                {
                    label: 'I/Q数据点',
                    data: constellationData,
                    backgroundColor: 'rgba(54, 162, 235, 0.6)',
                    pointRadius: 2
                },
                {
                    label: '圆心',
                    data: [{
                        x: firstResult.circleCenter[0],
                        y: firstResult.circleCenter[1]
                    }],
                    backgroundColor: 'red',
                    pointRadius: 8
                }
            ]
        };
        this.charts.constellation.update();

        // 更新呼吸波形（仅当有波形数据时）
        if (firstResult.respiratoryWave) {
            this.charts.respiratory.data = {
                labels: indices,
                datasets: [{
                    label: `呼吸波形 (${firstResult.respiratoryRate} bpm)`,
                    data: Array.from(firstResult.respiratoryWave.slice(0, sampleSize)),
                    borderColor: 'rgb(75, 192, 192)',
                    backgroundColor: 'rgba(75, 192, 192, 0.2)',
                    tension: 0.1
                }]
            };
            this.charts.respiratory.update();
        }

        // 更新心跳波形（仅当有波形数据时）
        if (firstResult.heartbeatWave) {
            this.charts.heartbeat.data = {
                labels: indices,
                datasets: [{
                    label: `心跳波形 (${firstResult.heartRate} bpm)`,
                    data: Array.from(firstResult.heartbeatWave.slice(0, sampleSize)),
                    borderColor: 'rgb(255, 99, 132)',
                    backgroundColor: 'rgba(255, 99, 132, 0.2)',
                    tension: 0.1
                }]
            };
            this.charts.heartbeat.update();
        }

        // 更新心率分布图
        const fileNames = results.map(r => r.fileName.substring(0, 10) + '...');
        const heartRates = results.map(r => r.heartRate);

        this.charts.heartRate.data = {
            labels: fileNames,
            datasets: [{
                label: '心率 (bpm)',
                data: heartRates,
                backgroundColor: 'rgba(255, 99, 132, 0.6)',
                borderColor: 'rgba(255, 99, 132, 1)',
                borderWidth: 1
            }]
        };
        this.charts.heartRate.update();

        // 更新呼吸频率分布图
        const respRates = results.map(r => r.respiratoryRate);

        this.charts.respRate.data = {
            labels: fileNames,
            datasets: [{
                label: '呼吸频率 (bpm)',
                data: respRates,
                backgroundColor: 'rgba(75, 192, 192, 0.6)',
                borderColor: 'rgba(75, 192, 192, 1)',
                borderWidth: 1
            }]
        };
        this.charts.respRate.update();

        // 更新心率时间序列图 - 使用真实的时间序列数据
        this.updateTimeSeriesCharts(results);
    }

    /**
     * 更新时间序列图表
     */
    updateTimeSeriesCharts(results) {
        if (results.length === 0) return;

        // 合并所有文件的时间序列数据
        let allHeartRateData = [];
        let allRespRateData = [];
        let allTimeLabels = [];
        let currentTime = 0;

        results.forEach((result, fileIndex) => {
            if (result.heartRateTimeSeries && result.respiratoryRateTimeSeries && result.timeAxis) {
                // 为每个文件的时间序列数据添加偏移
                const fileTimeOffset = currentTime;
                
                result.timeAxis.forEach((time, i) => {
                    const absoluteTime = fileTimeOffset + time;
                    allTimeLabels.push(`${Math.floor(absoluteTime / 60)}:${String(Math.floor(absoluteTime % 60)).padStart(2, '0')}`);
                    allHeartRateData.push(result.heartRateTimeSeries[i]);
                    allRespRateData.push(result.respiratoryRateTimeSeries[i]);
                });
                
                // 更新当前时间偏移（假设每个文件大约持续时间）
                const resultFs = Number.isFinite(result.samplingRate)
                    ? result.samplingRate
                    : (this.offlineProcessor?.fs || this.offlineSamplingRate || 50);
                const fileDuration = result.dataPoints / resultFs;
                currentTime += fileDuration;
            }
        });

        // 如果没有时间序列数据，使用文件级别的数据
        if (allHeartRateData.length === 0) {
            allTimeLabels = results.map((_, index) => `文件${index + 1}`);
            allHeartRateData = results.map(r => r.heartRate);
            allRespRateData = results.map(r => r.respiratoryRate);
        }

        // 更新心率时间序列图
        this.charts.heartRateTime.data = {
            labels: allTimeLabels,
            datasets: [{
                label: '心率变化 (bpm)',
                data: allHeartRateData,
                borderColor: 'rgb(255, 99, 132)',
                backgroundColor: 'rgba(255, 99, 132, 0.1)',
                tension: 0.3,
                fill: true,
                pointRadius: 2,
                pointHoverRadius: 6,
                borderWidth: 2
            }]
        };
        this.charts.heartRateTime.update();

        // 更新呼吸频率时间序列图
        this.charts.respRateTime.data = {
            labels: allTimeLabels,
            datasets: [{
                label: '呼吸频率变化 (bpm)',
                data: allRespRateData,
                borderColor: 'rgb(75, 192, 192)',
                backgroundColor: 'rgba(75, 192, 192, 0.1)',
                tension: 0.3,
                fill: true,
                pointRadius: 2,
                pointHoverRadius: 6,
                borderWidth: 2
            }]
        };
        this.charts.respRateTime.update();
    }

    /**
     * 更新JSON数据的图表（只显示心率和呼吸率时间序列）
     */
    updateJsonCharts(results) {
        if (!this._chartsEnabled) return;
        const jsonResults = results.filter(r => r.dataType === 'json');
        if (jsonResults.length === 0) return;

        const firstResult = jsonResults[0];

        // 只更新心率和呼吸率时间序列图
        if (firstResult.hrData && firstResult.hrData.length > 0) {
            const hrTimeLabels = Array.from({length: firstResult.hrData.length}, (_, i) => i + 1);
            this.charts.heartRateTime.data = {
                labels: hrTimeLabels,
                datasets: [{
                    label: '心率 (bpm)',
                    data: firstResult.hrData,
                    borderColor: 'rgb(255, 99, 132)',
                    backgroundColor: 'rgba(255, 99, 132, 0.2)',
                    tension: 0.1
                }]
            };
            this.charts.heartRateTime.update();
        }

        if (firstResult.rrData && firstResult.rrData.length > 0) {
            const rrTimeLabels = Array.from({length: firstResult.rrData.length}, (_, i) => i + 1);
            this.charts.respRateTime.data = {
                labels: rrTimeLabels,
                datasets: [{
                    label: '呼吸频率 (bpm)',
                    data: firstResult.rrData,
                    borderColor: 'rgb(75, 192, 192)',
                    backgroundColor: 'rgba(75, 192, 192, 0.2)',
                    tension: 0.1
                }]
            };
            this.charts.respRateTime.update();
        }

        // 清空其他图表（雷达信号相关）
        this.clearRadarCharts();
    }

    /**
     * JSON数据时隐藏雷达相关图表
     */
    setChartVisibilityForJson(hasJsonData) {
        const hideIds = [
            'iqChart',
            'constellationChart',
            'respiratoryChart',
            'heartbeatChart',
            'heartRateChart',
            'respRateChart'
        ];
        const ecgSection = document.querySelector('.ecg-section');

        hideIds.forEach(id => {
            const el = document.getElementById(id);
            if (el && el.parentElement) {
                el.parentElement.style.display = hasJsonData ? 'none' : 'block';
            }
        });

        if (ecgSection) {
            ecgSection.style.display = hasJsonData ? 'none' : 'block';
        }
    }

    /**
     * 清空雷达信号相关的图表（用于JSON数据时）
     */
    clearRadarCharts() {
        // 清空I/Q信号图
        if (this.charts.iq) {
            this.charts.iq.data = { labels: [], datasets: [] };
            this.charts.iq.update();
        }

        // 清空星座图
        if (this.charts.constellation) {
            this.charts.constellation.data = { datasets: [] };
            this.charts.constellation.update();
        }

        // 清空呼吸波形图
        if (this.charts.respiratory) {
            this.charts.respiratory.data = { labels: [], datasets: [] };
            this.charts.respiratory.update();
        }

        // 清空心跳波形图
        if (this.charts.heartbeat) {
            this.charts.heartbeat.data = { labels: [], datasets: [] };
            this.charts.heartbeat.update();
        }

        // 清空心率分布图
        if (this.charts.heartRate) {
            this.charts.heartRate.data = { labels: [], datasets: [] };
            this.charts.heartRate.update();
        }

        // 清空呼吸频率分布图
        if (this.charts.respRate) {
            this.charts.respRate.data = { labels: [], datasets: [] };
            this.charts.respRate.update();
        }
    }

    /**
     * 更新结果表格
     */
    updateResultsTable() {
        const tbody = document.getElementById('resultsTableBody');
        tbody.innerHTML = '';

        this.processedResults.forEach(result => {
            const row = document.createElement('tr');

            if (result.status === 'success') {
                if (result.dataType === 'json') {
                    // JSON数据格式
                    row.innerHTML = `
                        <td>${result.fileName}</td>
                        <td>${result.dataPoints.toLocaleString()}</td>
                        <td>${result.heartRate}</td>
                        <td>${result.respiratoryRate}</td>
                        <td>--</td>
                        <td>--</td>
                        <td>--</td>
                        <td><span class="status-success">JSON数据</span></td>
                    `;
                } else {
                    // TXT数据格式（原始雷达数据）
                    row.innerHTML = `
                        <td>${result.fileName}</td>
                        <td>${result.dataPoints.toLocaleString()}</td>
                        <td>${result.heartRate}</td>
                        <td>${result.respiratoryRate}</td>
                        <td>${result.circleCenter[0].toFixed(4)}</td>
                        <td>${result.circleCenter[1].toFixed(4)}</td>
                        <td>${result.circleRadius.toFixed(4)}</td>
                        <td><span class="status-success">雷达数据</span></td>
                    `;
                }
            } else {
                row.innerHTML = `
                    <td>${result.fileName}</td>
                    <td>--</td>
                    <td>--</td>
                    <td>--</td>
                    <td>--</td>
                    <td>--</td>
                    <td>--</td>
                    <td><span class="status-error">失败: ${result.error}</span></td>
                `;
            }
            
            tbody.appendChild(row);
        });
    }

    /**
     * 导出结果为CSV
     */
    exportResults() {
        if (this.processedResults.length === 0) {
            this.showMessage('没有可导出的结果', 'warning');
            return;
        }

        const headers = ['文件名', '数据点数', '心率(bpm)', '呼吸频率(bpm)', '圆心I', '圆心Q', '圆半径', '状态'];
        const csvContent = [
            headers.join(','),
            ...this.processedResults.map(result => {
                if (result.status === 'success') {
                    return [
                        result.fileName,
                        result.dataPoints,
                        result.heartRate,
                        result.respiratoryRate,
                        result.circleCenter[0].toFixed(4),
                        result.circleCenter[1].toFixed(4),
                        result.circleRadius.toFixed(4),
                        '成功'
                    ].join(',');
                } else {
                    return [
                        result.fileName,
                        '--', '--', '--', '--', '--', '--',
                        `失败: ${result.error}`
                    ].join(',');
                }
            })
        ].join('\n');

        this.downloadFile(csvContent, 'radar_processing_results.csv', 'text/csv');
        this.showMessage('结果已导出为CSV文件', 'success');
    }

    /**
     * 导出图表
     */
    exportCharts() {
        Object.keys(this.charts).forEach(chartName => {
            const canvas = this.charts[chartName].canvas;
            const link = document.createElement('a');
            link.download = `${chartName}_chart.png`;
            link.href = canvas.toDataURL();
            link.click();
        });
        
        this.showMessage('图表已导出为PNG文件', 'success');
    }

    /**
     * 切换设置面板
     */
    toggleSettings() {
        const panel = document.getElementById('settingsPanel');
        panel.classList.toggle('open');
    }

    /**
     * 应用设置。采样率输入只用于离线文件和没有实测数据时的备用值；
     * BLE连接期间不得用页面默认50 Hz覆盖已经验证的实时100 Hz时钟。
     */
    applySettings() {
        const srEl = document.getElementById('samplingRate');
        const sr = srEl ? parseInt(srEl.value, 10) : NaN;
        const samplingRate = Number.isFinite(sr) && sr > 0 ? sr : 50;
        this.offlineSamplingRate = samplingRate;
        if (this.offlineProcessor) this.offlineProcessor.fs = samplingRate;
        const liveRateLocked = this.bleConnected || !!this._simInterval;
        if (!liveRateLocked && this.processor) {
            this.processor.fs = samplingRate;
        } else if (typeof this.addBLELog === 'function') {
            this.addBLELog(
                `🔒 实时采样率由设备实测值锁定；设置面板的 ${samplingRate} Hz 仅用于离线/备用，`
                + `当前实时算法保持 ${this._getVitalsFs().toFixed(1)} Hz`
            );
        }
        
        // 应用心率平滑参数
        const smoothEl = document.getElementById('heartRateSmoothing');
        const smooth = smoothEl ? parseInt(smoothEl.value, 10) : NaN;
        if (Number.isFinite(smooth) && smooth >= 3 && smooth <= 9) {
            this.historyMaxLength = smooth;
        }
        
        const deltaEl = document.getElementById('heartRateDelta');
        const delta = deltaEl ? parseInt(deltaEl.value, 10) : NaN;
        if (Number.isFinite(delta) && delta >= 10 && delta <= 50) {
            this.heartRateDelta = delta;
        }
        if (typeof VitalsTracker !== 'undefined') {
            this.vitalsTracker = new VitalsTracker({
                minConfidence: this.vitalsMinConfidence,
                warmupWindows: 2,
                smoothWindows: this.historyMaxLength,
                jumpConfirmWindows: 3,
                invalidExpireWindows: 5,
                holdMaxMs: this.vitalsHoldMaxMs || 30000,
                hrJumpThreshold: this.heartRateDelta,
                rrJumpThreshold: 8,
                warmupMinSpanMs: 2000,
                jumpConfirmMinSpanMs: 4000,
                pendingJumpMaxMisses: 2,
                requireGuardForLowHrBelowBpm: 65
            });
        }
        this._initVitalsExtractor();
        this._resetVitalsStability();
        
        const range = this._getVitalsOptions();
        this.addBLELog(`⚙️ 已应用设置：离线/备用采样率=${samplingRate}Hz, HR=${(range.hrLow*60).toFixed(0)}-${(range.hrHigh*60).toFixed(0)}, RR=${(range.rrLow*60).toFixed(0)}-${(range.rrHigh*60).toFixed(0)}, 跟踪窗=${this.historyMaxLength}`);
        this.showMessage('心率、呼吸率频段及跟踪参数已应用', 'success');
        this.toggleSettings();
    }

    /**
     * 显示/隐藏加载指示器
     */
    showLoading(show) {
        document.getElementById('loadingOverlay').style.display = show ? 'flex' : 'none';
    }

    /**
     * 显示/隐藏状态区域
     */
    showStatus(show) {
        document.getElementById('statusSection').style.display = show ? 'block' : 'none';
    }

    /**
     * 隐藏结果区域
     */
    hideResults() {
        document.getElementById('resultsSection').style.display = 'none';
    }

    /**
     * 更新进度条
     */
    updateProgress(percentage, text) {
        document.getElementById('progressFill').style.width = `${percentage}%`;
        document.getElementById('progressText').textContent = text;
    }

    /**
     * 添加状态日志
     */
    addStatusLog(message) {
        const log = document.getElementById('statusLog');
        const timestamp = new Date().toLocaleTimeString();
        log.innerHTML += `<div>[${timestamp}] ${message}</div>`;
        log.scrollTop = log.scrollHeight;
    }

    /**
     * 显示消息
     */
    showMessage(message, type = 'info') {
        // 简单的消息显示，可以用更复杂的通知系统替换
        const colors = {
            success: '#28a745',
            warning: '#ffc107',
            error: '#dc3545',
            info: '#17a2b8'
        };

        const messageDiv = document.createElement('div');
        messageDiv.style.cssText = `
            position: fixed;
            top: 20px;
            right: 20px;
            background: ${colors[type]};
            color: white;
            padding: 15px 20px;
            border-radius: 5px;
            z-index: 3000;
            box-shadow: 0 4px 15px rgba(0,0,0,0.2);
        `;
        messageDiv.textContent = message;
        
        document.body.appendChild(messageDiv);
        
        setTimeout(() => {
            messageDiv.remove();
        }, 3000);
    }

    /**
     * 下载文件
     */
    downloadFile(content, filename, contentType) {
        const blob = new Blob([content], { type: contentType });
        const url = URL.createObjectURL(blob);
        const link = document.createElement('a');
        link.href = url;
        link.download = filename;
        link.click();
        URL.revokeObjectURL(url);
    }

    /**
     * 汇总本次蓝牙录制的窗口HR/RR统计与平均值
     */
    _buildBluetoothSessionStats() {
        const history = this._bleWindowHistory || [];
        const startTs = this.bleRecordingStartTime ? this.bleRecordingStartTime.toISOString() : new Date().toISOString();
        const endTs = new Date().toISOString();
        const durationSec = this.bleRecordingStartTime ? Math.round((Date.now() - this.bleRecordingStartTime.getTime())/1000) : 0;

        // Repeated held values remain visible to the operator but must not be
        // counted as new observations in session averages or AI diagnostics.
        const hrList = history
            .filter(item => !item.heartRateStatus || item.heartRateStatus === 'fresh')
            .map(item => item.heartRate)
            .filter(value => Number.isFinite(value) && value > 0);
        const rrList = history
            .filter(item => !item.respiratoryRateStatus || item.respiratoryRateStatus === 'fresh')
            .map(item => item.respiratoryRate)
            .filter(value => Number.isFinite(value) && value > 0);
        const avgHR = hrList.length ? Math.round(hrList.reduce((a,b)=>a+b,0)/hrList.length) : 0;
        const avgRR = rrList.length ? Math.round(rrList.reduce((a,b)=>a+b,0)/rrList.length) : 0;

        return {
            startTime: startTs,
            endTime: endTs,
            durationSeconds: durationSec,
            windowCount: history.length,
            windows: history,
            average: { heartRate: avgHR, respiratoryRate: avgRR }
        };
    }

    /**
     * 格式化文件大小
     */
    formatFileSize(bytes) {
        if (bytes === 0) return '0 Bytes';
        const k = 1024;
        const sizes = ['Bytes', 'KB', 'MB', 'GB'];
        const i = Math.floor(Math.log(bytes) / Math.log(k));
        return parseFloat((bytes / Math.pow(k, i)).toFixed(2)) + ' ' + sizes[i];
    }

    // ======= 蓝牙专用函数 =======

    /**
     * 打印原始数据到日志
     */
    printRawData(line) {
        const log = document.getElementById('bleRawDataLog');
        if (!log) return;

        const ts = new Date().toLocaleTimeString();
        const trimmed = line.trim();

        // V1.02 协议帧：解析并标注每个字段
        let displayLine;
        if (trimmed.startsWith('$') && !trimmed.startsWith('$AT')) {
            const raw = trimmed.endsWith('*') ? trimmed.slice(1, -1) : trimmed.slice(1);
            const p = raw.split(',');
            if (p.length >= 16) {
                displayLine = `[${ts}] MAC=${p[0]} Time=${p[1]} Lon=${p[2]} Lat=${p[3]} Ax=${p[4]} Ay=${p[5]} Az=${p[6]} Gx=${p[7]} Gy=${p[8]} Gz=${p[9]} Roll=${p[10]} Pitch=${p[11]} Yaw=${p[12]} V1=${p[13]} V2=${p[14]} V3=${p[15]}`;
            } else {
                displayLine = `[${ts}] ${trimmed}`;
            }
        } else {
            displayLine = `[${ts}] ${trimmed}`;
        }

        this._bleRawLines.push(displayLine);
        if (this._bleRawLines.length > 50) this._bleRawLines.splice(0, this._bleRawLines.length - 50);

        // 节流渲染
        if (this._bleRawRenderTimer) return;
        this._bleRawRenderTimer = setTimeout(() => {
            this._bleRawRenderTimer = null;
            log.style.whiteSpace = 'pre-wrap';
            log.textContent = `原始数据 (Payload: MAC,Time,Lon,Lat,Ax,Ay,Az,Gx,Gy,Gz,Roll,Pitch,Yaw,V1,V2,V3):\n${this._bleRawLines.join('\n')}\n`;
            log.scrollTop = log.scrollHeight;
        }, 200);
    }

    /**
     * 开始蓝牙连接计时
     */
    startBluetoothTimer() {
        this.stopBluetoothTimer(); // 防止重复计时器
        
        this.bleConnectTimer = setInterval(() => {
            if (this.bleConnectStartTime) {
                const elapsedMs = Date.now() - this.bleConnectStartTime;
                const minutes = Math.floor(elapsedMs / 60000);
                const seconds = Math.floor((elapsedMs % 60000) / 1000);
                document.getElementById('bleConnectTime').textContent = `${minutes} 分 ${seconds} 秒`;
            }
        }, 1000);
    }

    /**
     * 接收看门狗已停用：蓝牙连接仅允许手动断开
     */
    startRxWatchdog() {
        return;
    }

    stopRxWatchdog() {
        if (this.rxWatchdogTimer) {
            clearInterval(this.rxWatchdogTimer);
            this.rxWatchdogTimer = null;
        }
    }

    /**
     * 停止蓝牙连接计时
     */
    stopBluetoothTimer() {
        if (this.bleConnectTimer) {
            clearInterval(this.bleConnectTimer);
            this.bleConnectTimer = null;
        }
    }

    /**
     * 重置蓝牙数据
     */
    resetBluetoothData() {
        const backfillEnabled = !!this.bleBackfillState?.enabled;
        const backfillFromTsMs = this.bleBackfillState?.fromDeviceTsMs ?? null;
        this.bleBufferI = [];
        this.bleBufferQ = [];
        this.bleBufferIMU_X = [];
        this.bleBufferIMU_Y = [];
        this.bleBufferIMU_Z = [];
        this.bleBufferACC_X = [];
        this.bleBufferACC_Y = [];
        this.bleBufferACC_Z = [];
        this.bleBufferTemperature = [];
        this.bleBufferTimestamps = [];
        this.bleBufferDeviceTimestamps = [];
        this.bleBufferArrivalTimestamps = [];
        this.bleBufferNotificationIds = [];
        this.vitalsSamplingEpoch = Number.isFinite(this.vitalsSamplingEpoch)
            ? this.vitalsSamplingEpoch + 1
            : 1;
        this.vitalsFsEstimateInfo = null;
        this.vitalsFsDiagnostic = null;
        this._vitalsFsLogKey = '';
        this.bleDataCount = 0;
        
        // 重置录制相关数据
        this.bleRecordingFlag = 0;
        this.bleRecordingData = [];
        this.bleRecordingRawData = [];
        this.bleRecordingStartTime = null;
        this._bleWindowHistory = [];
        
        // 重置有效观测历史；禁止预填70/18等虚构生理值。
        this.heartRateHistory = [];
        this.respiratoryHistory = [];
        this.historyIndex = 0;
        this.lastStableHeartRate = null;
        this.lastStableRespRate = null;
        this.currentHeartRate = null;
        this.currentRespiratoryRate = null;
        this.vitalsLastReliable = {
            HR: { value: NaN, timestampMs: NaN, confidence: 0 },
            RR: { value: NaN, timestampMs: NaN, confidence: 0 }
        };
        this.vitalsDisplayState = null;
        this._vitalsMotionBaseline = null;

        // 重置自适应Y轴状态
        this.adaptiveSampleCount = 0;
        this.adaptiveLastMinI = Infinity;
        this.adaptiveLastMaxI = -Infinity;
        this.adaptiveLastMinQ = Infinity;
        this.adaptiveLastMaxQ = -Infinity;
        this.adaptiveStabilized = false;

        // 重置图表Y轴到初始范围
        if (this.bleCharts.iSignal) {
            this.bleCharts.iSignal.options.scales.y.min = 0;
            this.bleCharts.iSignal.options.scales.y.max = 4.0;
        }
        if (this.bleCharts.qSignal) {
            this.bleCharts.qSignal.options.scales.y.min = 0;
            this.bleCharts.qSignal.options.scales.y.max = 4.0;
        }

        // 重置丢包统计
        this.bleStats = {
            startRxTs: 0,
            lastRxTs: 0,
            startRxMonoTs: 0,
            lastRxMonoTs: 0,
            received: 0,
            expected: 0,
            missed: 0,
            lastGapMs: 0,
            gapEmaMs: 0,
            gapJitterEmaMs: 0,
            lastSeq: null,
            seqBased: false
        };

        const fsEl = document.getElementById('bleActualFs');
        const lossEl = document.getElementById('blePacketLoss');
        const jitterEl = document.getElementById('bleJitter');
        if (fsEl) fsEl.textContent = '-- Hz';
        if (lossEl) lossEl.textContent = '-- %';
        if (jitterEl) jitterEl.textContent = '-- ms';
        const missedEl = document.getElementById('bleMissedFrames');
        const barEl = document.getElementById('bleFreqBar');
        const barTextEl = document.getElementById('bleFreqBarText');
        if (missedEl) missedEl.textContent = '0 / 0';
        if (barEl) barEl.style.width = '0%';
        if (barTextEl) barTextEl.textContent = '0%';
        
        // 清空显示
        document.getElementById('bleDataCount').textContent = '0';
        document.getElementById('bleTotalDataPoints').textContent = '0';
        document.getElementById('bleCurrentHR').textContent = '-- bpm';
        document.getElementById('bleCurrentResp').textContent = '-- bpm';
        const vitalsHrEl = document.getElementById('vitalsHRNew');
        const vitalsRrEl = document.getElementById('vitalsRRNew');
        const vitalsRuntimeEl = document.getElementById('vitalsRuntime');
        if (vitalsHrEl) vitalsHrEl.textContent = '-- bpm';
        if (vitalsRrEl) vitalsRrEl.textContent = '-- bpm';
        if (vitalsRuntimeEl) vitalsRuntimeEl.textContent = '等待数据';
        const vitalsMotionStatusEl = document.getElementById('vitalsMotionStatus');
        if (vitalsMotionStatusEl) {
            vitalsMotionStatusEl.textContent = '等待首次可靠测量';
            vitalsMotionStatusEl.style.background = '#3a3848';
            vitalsMotionStatusEl.style.color = '#bdb9ca';
        }
        this.lastVitalsResult = null;
        this.vitalsFirstResultLogged = false;
        this._resetVitalsStability();
        const iqEl = document.getElementById('bleCurrentIQ');
        if (iqEl) iqEl.textContent = '--';
        const gyrEl = document.getElementById('bleCurrentGyr');
        if (gyrEl) gyrEl.textContent = '--';
        const accEl = document.getElementById('bleCurrentAcc');
        if (accEl) accEl.textContent = '--';
        document.getElementById('bleAvgHeartRate').textContent = '-- bpm';
        document.getElementById('bleAvgRespRate').textContent = '-- bpm';
        const tempEl = document.getElementById('bleCurrentTemp');
        const avgTempEl = document.getElementById('bleAvgTemp');
        if (tempEl) tempEl.textContent = '-- °C';
        if (avgTempEl) avgTempEl.textContent = '-- °C';
        
        // 清空原始数据日志/缓存
        this._bleRawLines = [];
        const rawLog = document.getElementById('bleRawDataLog');
        if (rawLog) {
            rawLog.style.whiteSpace = 'pre-wrap';
            rawLog.textContent = '原始数据:\n';
        }

        // 清空 BLE 事件日志/缓存
        this._bleLogLines = [];
        const bleLog = document.getElementById('bleLog');
        if (bleLog) {
            bleLog.style.whiteSpace = 'pre-line';
            bleLog.textContent = '';
        }

        this.bleBackfillBuffers = this._createEmptyBleBackfillBuffers();
        this.bleBackfillState = this._createBleBackfillState();
        this.bleBackfillState.enabled = backfillEnabled;
        this.bleBackfillState.fromDeviceTsMs = backfillFromTsMs;
        this._bleBackfillLogLines = [];
        const backfillLog = document.getElementById('bleBackfillLog');
        if (backfillLog) {
            backfillLog.style.whiteSpace = 'pre-line';
            backfillLog.textContent = '';
        }
        this.updateBleBackfillUI();

        Object.values(this.bleBackfillCharts || {}).forEach(chart => {
            if (chart && chart.data) {
                chart.data.labels = [];
                chart.data.datasets = [];
                if (typeof chart.update === 'function') chart.update('none');
            }
        });
        
        // 更新按钮状态
        this.updateBLEButtons();
    }

    /**
     * 切换蓝牙数据录制状态 (参考main.py的_set_button_att方法)
     */
    toggleBluetoothRecording() {
        // 切换录制状态，类似于 main.py 中的 flag_record = (1 + flag_record) % 2
        this.bleRecordingFlag = (1 + this.bleRecordingFlag) % 2;
        
        if (this.bleRecordingFlag === 1) {
            // 开始录制
            this.bleRecordingData = [];
            this.bleRecordingRawData = [];
            this._bleWindowHistory = [];
            this.bleRecordingStartTime = new Date();
            this._saveBlePetProfile();

            const profile = this._getBlePetProfile();
            this.bleRecordingPetProfile = profile;
            const startVitals = this._getBleVitalsSnapshot();
            const startImu = this._getLatestImuSnapshot();
            this.bleRecordingStartVitals = startVitals;
            this.bleRecordingStartImu = startImu;

            // 生成录制文件名 (参考main.py的命名规则)
            const timestamp = this.bleRecordingStartTime.toISOString()
                .slice(0, 16).replace('T', '-').replace(/:/g, '-');

            const startTimestamp = new Date().toISOString().replace('T', ' ').slice(0, 19);

            // 在录制数据开头添加元数据信息
            this.bleRecordingData.push(`# 录制开始时间: ${startTimestamp}`);
            this._appendBleRecordingMetadataComments(profile, startVitals, startImu, 'start');
            const isFFF0 = this.bleProtocol && this.bleProtocol.indexOf('FFF0') >= 0;
            if (isFFF0) {
                this.bleRecordingData.push(`# Payload Format: MAC,Time,Lon,Lat,Ax,Ay,Az,Gx,Gy,Gz,Roll,Pitch,Yaw,V1,V2,V3`);
            } else {
                this.bleRecordingData.push(`# Payload Format: timestamp,ADC_I,ADC_Q,Acc_X,Acc_Y,Acc_Z,I_voltage,Q_voltage,Gyr_x,Gyr_y,Gyr_z,temperature`);
            }
            this.bleRecordingData.push(`# 原始数据开始`);

            const startHR = startVitals.heartRate ?? 0;
            const startRR = startVitals.respiratoryRate ?? 0;
            this.addBLELog(`🔴 开始录制数据 - ${timestamp}`);
            this.addBLELog(`🐾 宠物: ${profile.name} / ${profile.breed} / ${profile.medical_condition || '无病情备注'}`);
            this.addBLELog(`💓 开始时心率: ${startHR} bpm, 呼吸率: ${startRR} bpm`);
            this.addBLELog('📝 实时保存到内存，结束时将下载处理后数据、原始数据与会话元数据');

            
        } else {
            // 结束录制并自动下载文件
            const recordingEndTime = new Date();
            const duration = ((recordingEndTime - this.bleRecordingStartTime) / 1000).toFixed(1);

            const profile = this.bleRecordingPetProfile || this._getBlePetProfile();
            const endVitals = this._getBleVitalsSnapshot();
            const endImu = this._getLatestImuSnapshot();
            const endTimestamp = new Date().toISOString().replace('T', ' ').slice(0, 19);

            // 在录制数据末尾添加结束信息
            this.bleRecordingData.push(`# 原始数据结束`);
            this.bleRecordingData.push(`# 录制结束时间: ${endTimestamp}`);
            this._appendBleRecordingMetadataComments(profile, endVitals, endImu, 'end');
            const dataPointCount = this.bleRecordingData.filter(line => !line.startsWith('#')).length;
            this.bleRecordingData.push(`# 录制统计: 总时长 ${duration}秒, 数据点数 ${dataPointCount}`);

            // 生成文件内容 (参考main.py的数据格式)
            let fileContent = '';
            for (const line of this.bleRecordingData) {
                fileContent += line + '\n';
            }
            
            // 生成文件名 (参考main.py的命名格式)
            const timestamp = this.bleRecordingStartTime.toISOString()
                .slice(0, 16).replace('T', '-').replace(/:/g, '-');
            const filename = `bluetooth_record_${timestamp}.txt`;
            
            // 自动下载处理后数据文件
            this.downloadFile(fileContent, filename, 'text/plain');

            // 生成并下载原始数据文件
            const rawFileContent = this.bleRecordingRawData.join('\n');
            const rawFilename = `bluetooth_raw_${timestamp}.txt`;
            this.downloadFile(rawFileContent, rawFilename, 'text/plain');
            this.addBLELog(`📄 已保存原始数据: ${rawFilename} (${this.bleRecordingRawData.length} 行)`);

            const sessionMeta = this._buildBleRecordingSessionMeta({
                profile,
                startTime: this.bleRecordingStartTime,
                endTime: recordingEndTime,
                durationSeconds: parseFloat(duration),
                startVitals: this.bleRecordingStartVitals || this._getBleVitalsSnapshot(),
                endVitals,
                startImu: this.bleRecordingStartImu || null,
                endImu,
                dataPoints: dataPointCount
            });
            const statsJson = JSON.stringify(sessionMeta, null, 2);
            const statsFilename = `bluetooth_record_${timestamp}_session.json`;
            this.downloadFile(statsJson, statsFilename, 'application/json');
            this.addBLELog(`📈 已保存会话元数据: ${statsFilename}`);

            const endHR = endVitals.heartRate ?? 0;
            const endRR = endVitals.respiratoryRate ?? 0;
            this.addBLELog(`🟢 录制结束 - 时长: ${duration}秒`);
            this.addBLELog(`💓 结束时心率: ${endHR} bpm, 呼吸率: ${endRR} bpm`);
            if (endImu) {
                this.addBLELog(`📐 结束时IMU Acc: ${endImu.acc.x.toFixed(3)}, ${endImu.acc.y.toFixed(3)}, ${endImu.acc.z.toFixed(3)} g`);
                this.addBLELog(`📐 结束时IMU Gyr: ${endImu.gyr.x.toFixed(3)}, ${endImu.gyr.y.toFixed(3)}, ${endImu.gyr.z.toFixed(3)} deg/s`);
            }
            this.addBLELog(`💾 已保存处理后数据: ${filename} (${dataPointCount} 数据点 + 元数据)`);
            this.addBLELog(`📂 总共下载3个文件: 处理后数据、原始数据、会话元数据`);

            
            // 清空录制缓存
            this.bleRecordingData = [];
            this.bleRecordingRawData = [];
            this.bleRecordingStartTime = null;
            this.bleRecordingPetProfile = null;
            this.bleRecordingStartVitals = null;
            this.bleRecordingStartImu = null;
        }
        
        // 更新按钮状态
        this.updateBLEButtons();
    }

    /**
     * 更新蓝牙实时图表
     */
    /** 高性能求最小值（避免 spread 导致栈溢出） */
    _arrMin(arr, start) {
        let m = Infinity;
        for (let i = start; i < arr.length; i++) { if (arr[i] < m) m = arr[i]; }
        return m;
    }
    /** 高性能求最大值 */
    _arrMax(arr, start) {
        let m = -Infinity;
        for (let i = start; i < arr.length; i++) { if (arr[i] > m) m = arr[i]; }
        return m;
    }

    updateBluetoothLiveCharts() {
        // 检查所有必需的图表是否已初始化
        if (!this.bleCharts.iSignal || !this.bleCharts.qSignal || !this.bleCharts.constellation) {
            console.warn('❌ 蓝牙图表未初始化：', {
                iSignal: !!this.bleCharts.iSignal,
                qSignal: !!this.bleCharts.qSignal,
                constellation: !!this.bleCharts.constellation,
                imu: !!this.bleCharts.imu,
                temperature: !!this.bleCharts.temperature
            });
            return;
        }

        // 调试：检查数据缓冲区状态
        if (this.bleDataCount === 10) {
            console.log('📊 数据缓冲区状态:', {
                I长度: this.bleBufferI.length,
                Q长度: this.bleBufferQ.length,
                IMU_X长度: this.bleBufferIMU_X.length,
                IMU_Y长度: this.bleBufferIMU_Y.length,
                IMU_Z长度: this.bleBufferIMU_Z.length,
                温度长度: this.bleBufferTemperature.length
            });
        }
        const len = this.bleBufferI.length;
        if (len < 10) return;

        // 自适应Y轴调节逻辑（提高实时性：增加检测频率）
        if (this.adaptiveYAxisEnabled && this.bleDataCount % 10 === 0) { // 每10个数据点计算一次，减少计算开销
            this.adaptiveSampleCount++;

            // 收集最近数据的范围
            const recentDataSize = Math.min(len, this.adaptiveStabilizeWindow);
            const recentStart = len - recentDataSize;
            const currentMinI = this._arrMin(this.bleBufferI, recentStart);
            const currentMaxI = this._arrMax(this.bleBufferI, recentStart);
            const currentMinQ = this._arrMin(this.bleBufferQ, recentStart);
            const currentMaxQ = this._arrMax(this.bleBufferQ, recentStart);

            // 检测信号范围是否发生显著变化（需要重新自适应）
            let rangeChanged = false;
            if (this.adaptiveStabilized) {
                const currentRangeI = currentMaxI - currentMinI;
                const currentRangeQ = currentMaxQ - currentMinQ;
                const stabilizedRangeI = this.adaptiveLastMaxI - this.adaptiveLastMinI;
                const stabilizedRangeQ = this.adaptiveLastMaxQ - this.adaptiveLastMinQ;

                // 检查是否处于微小波动状态（Y轴范围≤0.1）
                const isMicroFluctuationMode = (
                    this.bleCharts.iSignal && this.bleCharts.qSignal &&
                    (this.bleCharts.iSignal.options.scales.y.max - this.bleCharts.iSignal.options.scales.y.min) <= 0.1 ||
                    (this.bleCharts.qSignal.options.scales.y.max - this.bleCharts.qSignal.options.scales.y.min) <= 0.1
                );

                if (isMicroFluctuationMode) {
                    // 微小波动模式下，提高重置阈值，避免频繁重置
                    const rangeChangeThreshold = 1.0; // 从0.2提高到1.0，更宽容
                    const offsetThresholdI = Math.max(stabilizedRangeI * 0.5, 0.1); // 从0.15提高到0.5，从0.05提高到0.1
                    const offsetThresholdQ = Math.max(stabilizedRangeQ * 0.5, 0.1);

                    if (Math.abs(currentRangeI - stabilizedRangeI) / Math.max(stabilizedRangeI, 0.01) > rangeChangeThreshold ||
                        Math.abs(currentRangeQ - stabilizedRangeQ) / Math.max(stabilizedRangeQ, 0.01) > rangeChangeThreshold) {
                        rangeChanged = true;
                        }

                    if (Math.abs(currentMinI - this.adaptiveLastMinI) > offsetThresholdI ||
                        Math.abs(currentMaxI - this.adaptiveLastMaxI) > offsetThresholdI ||
                        Math.abs(currentMinQ - this.adaptiveLastMinQ) > offsetThresholdQ ||
                        Math.abs(currentMaxQ - this.adaptiveLastMaxQ) > offsetThresholdQ) {
                        rangeChanged = true;
                        }
                } else {
                    // 正常模式下的重置逻辑（保持原有敏感度）
                    const rangeChangeThreshold = 0.2;
                    if (Math.abs(currentRangeI - stabilizedRangeI) / Math.max(stabilizedRangeI, 0.01) > rangeChangeThreshold ||
                        Math.abs(currentRangeQ - stabilizedRangeQ) / Math.max(stabilizedRangeQ, 0.01) > rangeChangeThreshold) {
                        rangeChanged = true;
                    }

                    const offsetThresholdI = Math.max(stabilizedRangeI * 0.15, 0.05);
                    const offsetThresholdQ = Math.max(stabilizedRangeQ * 0.15, 0.05);
                    if (Math.abs(currentMinI - this.adaptiveLastMinI) > offsetThresholdI ||
                        Math.abs(currentMaxI - this.adaptiveLastMaxI) > offsetThresholdI ||
                        Math.abs(currentMinQ - this.adaptiveLastMinQ) > offsetThresholdQ ||
                        Math.abs(currentMaxQ - this.adaptiveLastMaxQ) > offsetThresholdQ) {
                        rangeChanged = true;
                    }
                }

                // 检测信号是否完全超出当前显示范围（需要立即响应）
                const currentChartMinI = this.bleCharts.iSignal?.options.scales.y.min || 0;
                const currentChartMaxI = this.bleCharts.iSignal?.options.scales.y.max || 4;
                const currentChartMinQ = this.bleCharts.qSignal?.options.scales.y.min || 0;
                const currentChartMaxQ = this.bleCharts.qSignal?.options.scales.y.max || 4;

                if (currentMinI < currentChartMinI || currentMaxI > currentChartMaxI ||
                    currentMinQ < currentChartMinQ || currentMaxQ > currentChartMaxQ) {
                    rangeChanged = true;
                }
            }

            // 如果检测到范围变化，重置自适应状态
            if (rangeChanged) {
                this.adaptiveSampleCount = 0;
                this.adaptiveLastMinI = Infinity;
                this.adaptiveLastMaxI = -Infinity;
                this.adaptiveLastMinQ = Infinity;
                this.adaptiveLastMaxQ = -Infinity;
                this.adaptiveStabilized = false;

                // 重置图表范围：根据当前状态智能选择范围
                if (this.bleCharts.iSignal && this.bleCharts.qSignal) {
                    // 检查之前是否处于微小波动模式
                    const wasMicroMode = (
                        (this.bleCharts.iSignal.options.scales.y.max - this.bleCharts.iSignal.options.scales.y.min) <= 0.1 ||
                        (this.bleCharts.qSignal.options.scales.y.max - this.bleCharts.qSignal.options.scales.y.min) <= 0.1
                    );

                    if (wasMicroMode) {
                        // 如果之前是微小模式，重置到稍微大一点的范围，但保持相对较小
                        this.bleCharts.iSignal.options.scales.y.min = Math.max(0, currentMinI - 0.1);
                        this.bleCharts.iSignal.options.scales.y.max = currentMaxI + 0.1;
                        this.bleCharts.qSignal.options.scales.y.min = Math.max(0, currentMinQ - 0.1);
                        this.bleCharts.qSignal.options.scales.y.max = currentMaxQ + 0.1;
                        } else {
                        // 正常重置到稍宽的初始范围
                        this.bleCharts.iSignal.options.scales.y.min = 1.0;
                        this.bleCharts.iSignal.options.scales.y.max = 3.0;
                        this.bleCharts.qSignal.options.scales.y.min = 1.0;
                        this.bleCharts.qSignal.options.scales.y.max = 3.0;
                    }
                }
                console.log('🔄 自适应Y轴已重置，重新开始调节');
            }

            // 如果还没稳定，更新范围
            if (!this.adaptiveStabilized) {
                this.adaptiveLastMinI = Math.min(this.adaptiveLastMinI, currentMinI);
                this.adaptiveLastMaxI = Math.max(this.adaptiveLastMaxI, currentMaxI);
                this.adaptiveLastMinQ = Math.min(this.adaptiveLastMinQ, currentMinQ);
                this.adaptiveLastMaxQ = Math.max(this.adaptiveLastMaxQ, currentMaxQ);

                // 检查是否达到稳定阈值
                if (this.adaptiveSampleCount >= this.adaptiveStabilizeThreshold) {
                    // 全程自适应：稳定后设置极紧凑范围以显示微小细节
                    const rangeI = this.adaptiveLastMaxI - this.adaptiveLastMinI;
                    const rangeQ = this.adaptiveLastMaxQ - this.adaptiveLastMinQ;

                    // 详细调试信息

                    // 简化波动性评估：使用数据范围的简单比例来代替复杂标准差计算
                    const dataRangeI = this.adaptiveLastMaxI - this.adaptiveLastMinI;
                    const dataRangeQ = this.adaptiveLastMaxQ - this.adaptiveLastMinQ;

                    // 使用数据范围的10%作为波动性估计（简化计算）
                    const stdI = dataRangeI * 0.1;
                    const stdQ = dataRangeQ * 0.1;

                    let newMinI, newMaxI, newMinQ, newMaxQ;

                    // 实时微小波动检测：波动小于0.2V时启用0.1单位Y轴控制
                    const microFluctuationThreshold = 0.2; // 微小波动阈值：总范围0.2V
                    if (rangeI <= microFluctuationThreshold || rangeQ <= microFluctuationThreshold) {
                        // 计算信号中心点
                        const centerI = (this.adaptiveLastMinI + this.adaptiveLastMaxI) / 2;
                        const centerQ = (this.adaptiveLastMinQ + this.adaptiveLastMaxQ) / 2;

                        // 设置0.1单位长度的Y轴范围（±0.05），最大化放大微小波动
                        newMinI = Math.max(0, centerI - 0.05);
                        newMaxI = centerI + 0.05;
                        newMinQ = Math.max(0, centerQ - 0.05);
                        newMaxQ = centerQ + 0.05;

                                } else {
                        // 自适应完成后，设置适度紧凑的范围来更清楚显示波峰
                        // 使用标准差的3倍作为余量，但最大不超过数据范围的20%，最小0.01V
                        const detailPaddingI = Math.max(0.01, Math.min(stdI * 3, rangeI * 0.20));
                        const detailPaddingQ = Math.max(0.01, Math.min(stdQ * 3, rangeQ * 0.20));

                        // 设置极紧凑的范围：数据范围 ± 很小的余量
                        newMinI = Math.max(0, this.adaptiveLastMinI - detailPaddingI);
                        newMaxI = this.adaptiveLastMaxI + detailPaddingI;
                        newMinQ = Math.max(0, this.adaptiveLastMinQ - detailPaddingQ);
                        newMaxQ = this.adaptiveLastMaxQ + detailPaddingQ;

                        }

                    // 更新I通道Y轴
                    if (this.bleCharts.iSignal) {
                        this.bleCharts.iSignal.options.scales.y.min = newMinI;
                        this.bleCharts.iSignal.options.scales.y.max = newMaxI;
                    }

                    // 更新Q通道Y轴
                    if (this.bleCharts.qSignal) {
                        this.bleCharts.qSignal.options.scales.y.min = newMinQ;
                        this.bleCharts.qSignal.options.scales.y.max = newMaxQ;
                    }

                    this.adaptiveStabilized = true;
                }
            }
        }

        // 🔍 调试：降低日志频率以提高性能

        const sampleSize = Math.min(1000, len);
        const start = len - sampleSize;

        // 缓存 labels 数组，避免每帧 Array.from 分配
        if (!this._chartLabels || this._chartLabels.length !== sampleSize) {
            this._chartLabels = Array.from({length: sampleSize}, (_, i) => i);
        }
        const indices = this._chartLabels;

        // I 通道（确保 dataset 已存在再做 mutation）
        if (this.bleCharts.iSignal) {
            const iDataSlice = this.bleBufferI.slice(start);
            const iDS = this.bleCharts.iSignal.data;
            iDS.labels = indices;
            if (iDS.datasets.length === 0) {
                iDS.datasets.push({ label: 'I通道', data: iDataSlice, borderColor: 'rgb(75, 192, 192)', backgroundColor: 'rgba(75, 192, 192, 0.2)', tension: 0.1, pointRadius: 0 });
            } else {
                iDS.datasets[0].data = iDataSlice;
            }
            this.bleCharts.iSignal.update('none');
        }

        // Q 通道
        if (this.bleCharts.qSignal) {
            const qDataSlice = this.bleBufferQ.slice(start);
            const qDS = this.bleCharts.qSignal.data;
            qDS.labels = indices;
            if (qDS.datasets.length === 0) {
                qDS.datasets.push({ label: 'Q通道', data: qDataSlice, borderColor: 'rgb(255, 99, 132)', backgroundColor: 'rgba(255, 99, 132, 0.2)', tension: 0.1, pointRadius: 0 });
            } else {
                qDS.datasets[0].data = qDataSlice;
            }
            this.bleCharts.qSignal.update('none');
        }

        const constellationSampleSize = Math.min(300, len);
        const step = Math.max(1, Math.floor(len / constellationSampleSize));
        const constData = [];
        for (let i = start; i < len; i += step) constData.push({ x: this.bleBufferI[i], y: this.bleBufferQ[i] });
        if (this.bleCharts.constellation) {
            const cDS = this.bleCharts.constellation.data;
            if (cDS.datasets.length === 0) {
                cDS.datasets.push({ label: 'I/Q数据点', data: constData, backgroundColor: 'rgba(54, 162, 235, 0.6)', pointRadius: 2 });
            } else {
                cDS.datasets[0].data = constData;
            }
            this.bleCharts.constellation.update('none');
        }

        if (this.bleCharts.imu && this.bleBufferIMU_X.length > 0) {
            const imuDS = this.bleCharts.imu.data;
            imuDS.labels = indices;
            if (imuDS.datasets.length < 3) {
                imuDS.datasets = [
                    { label: 'Gx (deg/s)', data: this.bleBufferIMU_X.slice(start), borderColor: 'rgb(255, 99, 132)', backgroundColor: 'rgba(255, 99, 132, 0.08)', tension: 0.1, pointRadius: 0 },
                    { label: 'Gy (deg/s)', data: this.bleBufferIMU_Y.slice(start), borderColor: 'rgb(54, 162, 235)', backgroundColor: 'rgba(54, 162, 235, 0.08)', tension: 0.1, pointRadius: 0 },
                    { label: 'Gz (deg/s)', data: this.bleBufferIMU_Z.slice(start), borderColor: 'rgb(75, 192, 192)', backgroundColor: 'rgba(75, 192, 192, 0.08)', tension: 0.1, pointRadius: 0 }
                ];
            } else {
                imuDS.datasets[0].data = this.bleBufferIMU_X.slice(start);
                imuDS.datasets[1].data = this.bleBufferIMU_Y.slice(start);
                imuDS.datasets[2].data = this.bleBufferIMU_Z.slice(start);
            }
            this.bleCharts.imu.update('none');
        }

        if (this.bleCharts.acc && this.bleBufferACC_X.length > 0) {
            const accDS = this.bleCharts.acc.data;
            accDS.labels = indices;
            if (accDS.datasets.length < 3) {
                accDS.datasets = [
                    { label: 'Ax (m/s²)', data: this.bleBufferACC_X.slice(start), borderColor: 'rgb(255, 159, 64)', backgroundColor: 'rgba(255, 159, 64, 0.08)', tension: 0.1, pointRadius: 0 },
                    { label: 'Ay (m/s²)', data: this.bleBufferACC_Y.slice(start), borderColor: 'rgb(153, 102, 255)', backgroundColor: 'rgba(153, 102, 255, 0.08)', tension: 0.1, pointRadius: 0 },
                    { label: 'Az (m/s²)', data: this.bleBufferACC_Z.slice(start), borderColor: 'rgb(255, 205, 86)', backgroundColor: 'rgba(255, 205, 86, 0.08)', tension: 0.1, pointRadius: 0 }
                ];
            } else {
                accDS.datasets[0].data = this.bleBufferACC_X.slice(start);
                accDS.datasets[1].data = this.bleBufferACC_Y.slice(start);
                accDS.datasets[2].data = this.bleBufferACC_Z.slice(start);
            }
            const accMin = Math.min(
                this._arrMin(this.bleBufferACC_X, start),
                this._arrMin(this.bleBufferACC_Y, start),
                this._arrMin(this.bleBufferACC_Z, start)
            );
            const accMax = Math.max(
                this._arrMax(this.bleBufferACC_X, start),
                this._arrMax(this.bleBufferACC_Y, start),
                this._arrMax(this.bleBufferACC_Z, start)
            );
            const accCenter = (accMin + accMax) / 2;
            const accRange = Math.max(0.3, accMax - accMin);
            const accPadding = Math.max(0.05, accRange * 0.15);
            this.bleCharts.acc.options.scales.y.min = accCenter - (accRange / 2) - accPadding;
            this.bleCharts.acc.options.scales.y.max = accCenter + (accRange / 2) + accPadding;
            this.bleCharts.acc.update('none');
        }

        if (this.bleCharts.accDynamic && this.bleBufferACC_X.length > 0) {
            const accXSlice = this.bleBufferACC_X.slice(start);
            const accYSlice = this.bleBufferACC_Y.slice(start);
            const accZSlice = this.bleBufferACC_Z.slice(start);
            const meanX = accXSlice.reduce((sum, value) => sum + value, 0) / accXSlice.length;
            const meanY = accYSlice.reduce((sum, value) => sum + value, 0) / accYSlice.length;
            const meanZ = accZSlice.reduce((sum, value) => sum + value, 0) / accZSlice.length;
            const dynX = accXSlice.map(value => value - meanX);
            const dynY = accYSlice.map(value => value - meanY);
            const dynZ = accZSlice.map(value => value - meanZ);
            const dynDS = this.bleCharts.accDynamic.data;
            dynDS.labels = indices;
            if (dynDS.datasets.length < 3) {
                dynDS.datasets = [
                    { label: 'Ax 动态 (m/s²)', data: dynX, borderColor: 'rgb(255, 159, 64)', backgroundColor: 'rgba(255, 159, 64, 0.08)', tension: 0.1, pointRadius: 0 },
                    { label: 'Ay 动态 (m/s²)', data: dynY, borderColor: 'rgb(153, 102, 255)', backgroundColor: 'rgba(153, 102, 255, 0.08)', tension: 0.1, pointRadius: 0 },
                    { label: 'Az 动态 (m/s²)', data: dynZ, borderColor: 'rgb(255, 205, 86)', backgroundColor: 'rgba(255, 205, 86, 0.08)', tension: 0.1, pointRadius: 0 }
                ];
            } else {
                dynDS.datasets[0].data = dynX;
                dynDS.datasets[1].data = dynY;
                dynDS.datasets[2].data = dynZ;
            }
            const dynMin = Math.min(...dynX, ...dynY, ...dynZ);
            const dynMax = Math.max(...dynX, ...dynY, ...dynZ);
            const dynCenter = (dynMin + dynMax) / 2;
            const dynRange = Math.max(0.08, dynMax - dynMin);
            const dynPadding = Math.max(0.02, dynRange * 0.2);
            this.bleCharts.accDynamic.options.scales.y.min = dynCenter - (dynRange / 2) - dynPadding;
            this.bleCharts.accDynamic.options.scales.y.max = dynCenter + (dynRange / 2) + dynPadding;
            this.bleCharts.accDynamic.update('none');
        }

        // 更新温度图表
        let validTempData = []; // 声明在外部以便后续使用
        if (this.bleCharts.temperature && this.bleBufferTemperature.length > 0) {
            const tempDataRaw = this.bleBufferTemperature.slice(start);
            // 过滤掉null值，只显示有效温度数据
            validTempData = tempDataRaw.map((temp, idx) => temp !== null ? temp : null);

            // 计算有效温度数据的统计
            const validTemps = validTempData.filter(temp => temp !== null);
            const hasValidTemp = validTemps.length > 0;

            this.bleCharts.temperature.data = {
                labels: indices,
                datasets: [
                    {
                        label: hasValidTemp ? `温度 (°C) - 最新: ${validTemps[validTemps.length - 1]?.toFixed(1)}°C` : '温度 (°C) - 无数据',
                        data: validTempData,
                        borderColor: hasValidTemp ? 'rgb(255, 159, 64)' : 'rgb(200, 200, 200)',
                        backgroundColor: hasValidTemp ? 'rgba(255, 159, 64, 0.2)' : 'rgba(200, 200, 200, 0.1)',
                        tension: 0.3,
                        pointRadius: 0,
                        fill: true,
                        spanGaps: false // 不连接null值之间的间隙
                    }
                ]
            };
            this.bleCharts.temperature.update('none');
        }

        // 更新当前温度显示
        if (validTempData && validTempData.length > 0) {
            const currentTemp = validTempData[validTempData.length - 1];
            if (currentTemp !== null) {
                const tempEl = document.getElementById('bleCurrentTemp');
                const avgTempEl = document.getElementById('bleAvgTemp');
                if (tempEl) {
                    tempEl.textContent = `${currentTemp.toFixed(1)} °C`;
                }
                if (avgTempEl) {
                    avgTempEl.textContent = `${currentTemp.toFixed(1)} °C`;
                }
            }
        }

        // 更新当前陀螺仪显示 (gx, gy, gz)
        if (this.bleBufferIMU_X.length > 0) {
            const gx = this.bleBufferIMU_X[this.bleBufferIMU_X.length - 1];
            const gy = this.bleBufferIMU_Y[this.bleBufferIMU_Y.length - 1];
            const gz = this.bleBufferIMU_Z[this.bleBufferIMU_Z.length - 1];
            const gyrEl = document.getElementById('bleCurrentGyr');
            if (gyrEl) {
                gyrEl.textContent = `gx:${gx.toFixed(1)} gy:${gy.toFixed(1)} gz:${gz.toFixed(1)}`;
            }
        }

        // 更新当前加速度计显示 (ax, ay, az)
        if (this.bleBufferACC_X.length > 0) {
            const ax = this.bleBufferACC_X[this.bleBufferACC_X.length - 1];
            const ay = this.bleBufferACC_Y[this.bleBufferACC_Y.length - 1];
            const az = this.bleBufferACC_Z[this.bleBufferACC_Z.length - 1];
            const accEl = document.getElementById('bleCurrentAcc');
            if (accEl) {
                accEl.textContent = `ax:${ax.toFixed(3)} ay:${ay.toFixed(3)} az:${az.toFixed(3)}`;
            }
        }
    }

    updateBleBackfillCharts() {
        if (!this.bleBackfillCharts.iq || !this.bleBackfillCharts.lag) return;

        const len = this.bleBackfillBuffers.timestamps.length;
        if (len === 0) return;

        this._ensureBleBackfillSectionVisible();

        const sampleSize = Math.min(600, len);
        const start = len - sampleSize;
        const labels = Array.from({ length: sampleSize }, (_, idx) => idx + 1);

        const iqLabels = labels;
        const iqI = this.bleBackfillBuffers.i.slice(start);
        const iqQ = this.bleBackfillBuffers.q.slice(start);
        this.bleBackfillCharts.iq.data.labels = iqLabels;
        this.bleBackfillCharts.iq.data.datasets = [
            {
                label: 'I',
                data: iqI,
                borderColor: 'rgb(75, 192, 192)',
                backgroundColor: 'rgba(75, 192, 192, 0.08)',
                tension: 0.1,
                pointRadius: 0
            },
            {
                label: 'Q',
                data: iqQ,
                borderColor: 'rgb(255, 99, 132)',
                backgroundColor: 'rgba(255, 99, 132, 0.08)',
                tension: 0.1,
                pointRadius: 0
            }
        ];
        this.bleBackfillCharts.iq.update('none');

        this.bleBackfillCharts.lag.data.labels = iqLabels;
        this.bleBackfillCharts.lag.data.datasets = [
            {
                label: '设备时间滞后 (秒)',
                data: this.bleBackfillBuffers.lagMs.slice(start).map(v => v / 1000),
                borderColor: 'rgb(255, 159, 64)',
                backgroundColor: 'rgba(255, 159, 64, 0.10)',
                tension: 0.15,
                pointRadius: 0,
                fill: true
            }
        ];
        this.bleBackfillCharts.lag.update('none');

        this.bleBackfillCharts.imu.data.labels = iqLabels;
        this.bleBackfillCharts.imu.data.datasets = [
            {
                label: 'Gx',
                data: this.bleBackfillBuffers.imuX.slice(start),
                borderColor: 'rgb(255, 99, 132)',
                tension: 0.1,
                pointRadius: 0
            },
            {
                label: 'Gy',
                data: this.bleBackfillBuffers.imuY.slice(start),
                borderColor: 'rgb(54, 162, 235)',
                tension: 0.1,
                pointRadius: 0
            },
            {
                label: 'Gz',
                data: this.bleBackfillBuffers.imuZ.slice(start),
                borderColor: 'rgb(75, 192, 192)',
                tension: 0.1,
                pointRadius: 0
            }
        ];
        this.bleBackfillCharts.imu.update('none');

        this.bleBackfillCharts.acc.data.labels = iqLabels;
        this.bleBackfillCharts.acc.data.datasets = [
            {
                label: 'Ax',
                data: this.bleBackfillBuffers.accX.slice(start),
                borderColor: 'rgb(255, 159, 64)',
                tension: 0.1,
                pointRadius: 0
            },
            {
                label: 'Ay',
                data: this.bleBackfillBuffers.accY.slice(start),
                borderColor: 'rgb(153, 102, 255)',
                tension: 0.1,
                pointRadius: 0
            },
            {
                label: 'Az',
                data: this.bleBackfillBuffers.accZ.slice(start),
                borderColor: 'rgb(255, 205, 86)',
                tension: 0.1,
                pointRadius: 0
            }
        ];
        this.bleBackfillCharts.acc.update('none');
    }

    /**
     * 重置自适应Y轴状态（手动重置为初始范围）
     */
    resetAdaptiveYAxis() {
        console.log('🔄 重置自适应Y轴状态...');

        // 重置状态变量
        this.adaptiveSampleCount = 0;
        this.adaptiveLastMinI = Infinity;
        this.adaptiveLastMaxI = -Infinity;
        this.adaptiveLastMinQ = Infinity;
        this.adaptiveLastMaxQ = -Infinity;
        this.adaptiveStabilized = false;

        // 重置图表Y轴到初始范围
        if (this.bleCharts.iSignal) {
            this.bleCharts.iSignal.options.scales.y.min = 0;
            this.bleCharts.iSignal.options.scales.y.max = 4.0;
            this.bleCharts.iSignal.update();
        }
        if (this.bleCharts.qSignal) {
            this.bleCharts.qSignal.options.scales.y.min = 0;
            this.bleCharts.qSignal.options.scales.y.max = 4.0;
            this.bleCharts.qSignal.update();
        }

        console.log('✅ 自适应Y轴已重置为初始范围 (0-4.0V)');
    }

    /**
     * 强制切换到细节显示模式（极紧凑的Y轴范围）
     */
    forceDetailMode() {
        if (this.bleBufferI.length < 50) {
            console.warn('❌ 数据点不足，无法切换到细节模式');
            return;
        }

        console.log('🔍 强制切换到细节显示模式...');

        // 使用最近50个数据点计算极紧凑的范围
        const detailDataSize = Math.min(this.bleBufferI.length, 50);
        const startIdx = this.bleBufferI.length - detailDataSize;
        const detailI = this.bleBufferI.slice(startIdx);
        const detailQ = this.bleBufferQ.slice(startIdx);

        const minI = Math.min(...detailI);
        const maxI = Math.max(...detailI);
        const minQ = Math.min(...detailQ);
        const maxQ = Math.max(...detailQ);

        const rangeI = maxI - minI;
        const rangeQ = maxQ - minQ;

        // 设置极小的余量：0.02V或数据范围的2%
        const detailPadding = 0.02;
        const rangePaddingI = Math.max(detailPadding, rangeI * 0.02);
        const rangePaddingQ = Math.max(detailPadding, rangeQ * 0.02);

        const detailMinI = Math.max(0, minI - rangePaddingI);
        const detailMaxI = maxI + rangePaddingI;
        const detailMinQ = Math.max(0, minQ - rangePaddingQ);
        const detailMaxQ = maxQ + rangePaddingQ;

        // 更新图表
        if (this.bleCharts.iSignal) {
            this.bleCharts.iSignal.options.scales.y.min = detailMinI;
            this.bleCharts.iSignal.options.scales.y.max = detailMaxI;
            this.bleCharts.iSignal.update();
        }
        if (this.bleCharts.qSignal) {
            this.bleCharts.qSignal.options.scales.y.min = detailMinQ;
            this.bleCharts.qSignal.options.scales.y.max = detailMaxQ;
            this.bleCharts.qSignal.update();
        }

        // 重置自适应状态，防止自动调节覆盖手动设置
        this.adaptiveStabilized = false;

        console.log(`🎯 细节模式已激活: I(${detailMinI.toFixed(4)}-${detailMaxI.toFixed(4)}V), Q(${detailMinQ.toFixed(4)}-${detailMaxQ.toFixed(4)}V)`);
    }

    /**
     * 强制重新初始化所有图表（用于调试蓝牙图表显示问题）
     */
    forceReinitializeCharts() {
        try { this.initializeCharts(); } catch (e) { console.warn('initializeCharts 异常:', e.message); }
        try { this.initializeBluetoothCharts(); } catch (e) { console.warn('initializeBluetoothCharts 异常:', e.message); }
        try { this.initializeBleBackfillCharts(); } catch (e) { console.warn('initializeBleBackfillCharts 异常:', e.message); }

        setTimeout(() => {
            try {
                const allCharts = [
                    ...Object.values(this.charts || {}),
                    ...Object.values(this.bleCharts || {}),
                    ...Object.values(this.bleBackfillCharts || {})
                ];
                allCharts.forEach(chart => {
                    if (chart && typeof chart.resize === 'function') chart.resize();
                    if (chart && typeof chart.update === 'function') chart.update();
                });
            } catch (e) { console.warn('图表resize/update异常:', e.message); }
        }, 100);
    }

    _robustSampleRateFromTimeline(timestamps, maxSamples = 2000) {
        const source = Array.isArray(timestamps) ? timestamps : [];
        const offset = Math.max(0, source.length - maxSamples);
        const points = [];
        let backwards = 0;
        let previousTime = -Infinity;
        for (let index = offset; index < source.length; index++) {
            const timeMs = Number(source[index]);
            if (!Number.isFinite(timeMs)) continue;
            // Ignore isolated backwards timestamps. A true clock reset starts a
            // new sampling epoch elsewhere and must not be averaged into this
            // window.
            if (timeMs < previousTime - 1) {
                backwards += 1;
                continue;
            }
            previousTime = Math.max(previousTime, timeMs);
            points.push({ sampleIndex: index, timeMs });
        }
        if (points.length < 20 || backwards > points.length * 0.05) return null;

        // Estimate slopes over several long, overlapping spans. Crucially the
        // numerator is the cumulative *frame index*, not a per-notification
        // batch size. Variable BLE packing such as 1/1/1/5 frames at 25
        // notifications/s therefore remains 50 frames/s instead of collapsing
        // to the 25 Hz median notification rate.
        const candidates = [];
        const addSlope = (left, right) => {
            const sampleDelta = right.sampleIndex - left.sampleIndex;
            const timeDeltaMs = right.timeMs - left.timeMs;
            if (sampleDelta < 10 || timeDeltaMs < 250) return;
            const rate = sampleDelta * 1000 / timeDeltaMs;
            if (Number.isFinite(rate) && rate >= 1 && rate <= 500) candidates.push(rate);
        };
        addSlope(points[0], points[points.length - 1]);
        const spanPoints = Math.max(12, Math.floor(points.length * 0.45));
        const stride = Math.max(1, Math.floor((points.length - spanPoints) / 5));
        for (let start = 0; start + spanPoints < points.length; start += stride) {
            addSlope(points[start], points[Math.min(points.length - 1, start + spanPoints)]);
        }
        if (candidates.length < 2) return null;
        const globalRate = candidates[0];
        const sorted = candidates.slice().sort((a, b) => a - b);
        const median = values => {
            const middle = Math.floor(values.length / 2);
            return values.length % 2
                ? values[middle]
                : (values[middle - 1] + values[middle]) / 2;
        };
        // The full-span cumulative slope is unbiased by variable frame counts
        // at individual notification boundaries. Shorter spans are used only
        // to reject an unstable clock, not averaged back into the estimate.
        const rate = globalRate;
        const deviations = sorted.map(value => Math.abs(value - rate)).sort((a, b) => a - b);
        const mad = median(deviations);
        const spanMs = points[points.length - 1].timeMs - points[0].timeMs;
        if (!(rate > 0) || spanMs < 800 || mad / rate > 0.30) return null;
        return {
            rate,
            mad,
            spanMs,
            points: points.length,
            backwards
        };
    }

    _estimateNotificationPacking() {
        const ids = this.bleBufferNotificationIds || [];
        const offset = Math.max(0, ids.length - 2000);
        const counts = new Map();
        for (let index = offset; index < ids.length; index++) {
            const id = Number(ids[index]);
            if (!Number.isFinite(id)) continue;
            counts.set(id, (counts.get(id) || 0) + 1);
        }
        if (counts.size < 5) return null;
        const frameCounts = Array.from(counts.values());
        return {
            notifications: counts.size,
            frames: frameCounts.reduce((sum, value) => sum + value, 0),
            meanFramesPerNotification: frameCounts.reduce((sum, value) => sum + value, 0) / counts.size,
            minFramesPerNotification: Math.min(...frameCounts),
            maxFramesPerNotification: Math.max(...frameCounts)
        };
    }

    _estimateVitalsFsFromTimestamps() {
        const arrival = this._robustSampleRateFromTimeline(this.bleBufferArrivalTimestamps);
        const device = this._robustSampleRateFromTimeline(this.bleBufferDeviceTimestamps);
        // Compatibility for imported tests and legacy callers created before
        // the separate clocks existed.
        const legacy = !arrival && !device
            ? this._robustSampleRateFromTimeline(this.bleBufferTimestamps)
            : null;
        const packing = this._estimateNotificationPacking();
        let selected = arrival || device || legacy;
        let source = arrival
            ? 'arrival-frame-slope'
            : device
            ? 'device-frame-slope'
            : legacy
            ? 'legacy-frame-slope'
            : 'unavailable';

        if (arrival && device) {
            const relativeDifference = Math.abs(arrival.rate - device.rate)
                / Math.max(arrival.rate, device.rate, 1);
            if (relativeDifference <= 0.12) {
                selected = {
                    ...arrival,
                    rate: (arrival.rate + device.rate) / 2,
                    mad: Math.max(arrival.mad, device.mad)
                };
                source = 'device-arrival-consensus';
            } else {
                // Arrival time describes the actual sample stream consumed by
                // the browser. Keep the disagreement visible; _getVitalsFs()
                // will also cross-check the independent connection frame count.
                selected = arrival;
                source = 'arrival-device-conflict';
            }
        }
        if (!selected) {
            this.vitalsFsEstimateInfo = {
                rate: NaN,
                source,
                arrivalRate: arrival?.rate ?? NaN,
                deviceRate: device?.rate ?? NaN,
                packing
            };
            return NaN;
        }
        this.vitalsFsEstimateInfo = {
            ...selected,
            source,
            arrivalRate: arrival?.rate ?? NaN,
            deviceRate: device?.rate ?? NaN,
            packing,
            medianBatchSize: packing?.meanFramesPerNotification || 1
        };
        return selected.rate;
    }

    _getVitalsFs() {
        const configured = (this.processor && Number.isFinite(this.processor.fs))
            ? this.processor.fs
            : 50;
        if (!this.bleConnected && !this._simInterval) return configured;

        const timelineMeasured = this._estimateVitalsFsFromTimestamps();
        const connectionMeasured = this.bleStats?.received >= 30
            ? this._estimateCurrentBleFs()
            : NaN;
        let measured = timelineMeasured;
        let source = this.vitalsFsEstimateInfo?.source || 'timestamps';
        let rateConflict = false;
        let antiHalfRateCorrection = false;
        let staleConnectionRateRejected = false;
        const timelineInfo = this.vitalsFsEstimateInfo || {};
        const timelineIsRecentReliable = Number.isFinite(timelineMeasured)
            && Number.isFinite(timelineInfo.spanMs)
            && timelineInfo.spanMs >= 800
            && Number.isFinite(timelineInfo.points)
            && timelineInfo.points >= 20
            && /arrival|device/.test(String(timelineInfo.source || ''));

        // Always cross-check the recent timeline slope with an independent
        // total-frame/elapsed-time estimate. The former implementation only
        // consulted the connection count when the timestamp result was NaN;
        // therefore a finite but wrong 25 Hz batch median silently overrode a
        // real 50 Hz stream and converted 80 bpm to 40 bpm.
        if (Number.isFinite(timelineMeasured) && Number.isFinite(connectionMeasured)) {
            const high = Math.max(timelineMeasured, connectionMeasured);
            const low = Math.min(timelineMeasured, connectionMeasured);
            const relativeDifference = (high - low) / Math.max(high, 1);
            const ratio = high / Math.max(low, 1e-9);
            if (relativeDifference <= 0.12) {
                measured = (timelineMeasured + connectionMeasured) / 2;
                source = `${source}+connection-consensus`;
            } else if (ratio >= 1.75 && ratio <= 2.25) {
                rateConflict = true;
                if (timelineIsRecentReliable) {
                    // The connection estimate averages the whole connection;
                    // after a real 100 -> 50 Hz step it can remain close to
                    // 100 for many seconds.  A recent multi-second frame slope
                    // is then the authoritative clock.  Choosing the high value
                    // unconditionally would turn 80 bpm into about 160 bpm.
                    measured = timelineMeasured;
                    if (timelineMeasured >= connectionMeasured) {
                        source = 'recent-timeline-anti-half-rate';
                        antiHalfRateCorrection = true;
                    } else {
                        source = 'recent-timeline-over-stale-connection';
                        staleConnectionRateRejected = true;
                    }
                } else {
                    // Legacy/short timelines can still collapse to the BLE
                    // notification rate.  In that narrowly defined case the
                    // high frame-count estimate prevents the observed 80 -> 40
                    // scaling until a trustworthy recent slope is available.
                    measured = high;
                    source = 'anti-half-rate-consensus';
                    antiHalfRateCorrection = true;
                }
            } else {
                // Prefer the recent per-sample timeline for other conflicts,
                // but keep the disagreement explicit in diagnostics.
                measured = timelineMeasured;
                rateConflict = true;
                source = `${source}+connection-conflict`;
            }
        } else if (!Number.isFinite(timelineMeasured) && Number.isFinite(connectionMeasured)) {
            measured = connectionMeasured;
            source = 'connection-frame-count';
        }
        if (!Number.isFinite(measured) || measured < 1 || measured > 500) {
            this.vitalsFsDiagnostic = {
                configured,
                target: this.bleTargetFs,
                measured: NaN,
                timelineMeasured,
                connectionMeasured,
                selected: configured,
                source: 'configured-unverified',
                verified: false
            };
            return configured;
        }

        const relativeError = Math.abs(measured - configured) / Math.max(configured, 1);
        // Use the commanded rate inside a small tolerance (less jitter), but do
        // not scale BPM with an unverified 50 Hz target when the board is really
        // delivering 10/20/40 Hz.
        const selected = relativeError <= 0.08 ? configured : measured;
        this.vitalsFsDiagnostic = {
            configured,
            target: this.bleTargetFs,
            measured,
            timelineMeasured,
            connectionMeasured,
            arrivalMeasured: this.vitalsFsEstimateInfo?.arrivalRate ?? NaN,
            deviceMeasured: this.vitalsFsEstimateInfo?.deviceRate ?? NaN,
            selected,
            source,
            relativeError,
            rateConflict,
            antiHalfRateCorrection,
            staleConnectionRateRejected,
            verified: true,
            medianBatchSize: this.vitalsFsEstimateInfo?.medianBatchSize || 1,
            notificationPacking: this.vitalsFsEstimateInfo?.packing || null
        };
        const logKey = [
            source,
            configured.toFixed(1),
            measured.toFixed(1),
            this.vitalsFsDiagnostic.medianBatchSize
        ].join('|');
        if (logKey !== this._vitalsFsLogKey && typeof this.addBLELog === 'function') {
            this._vitalsFsLogKey = logKey;
            if ((this.vitalsFsDiagnostic.notificationPacking?.meanFramesPerNotification || 1) > 1.05) {
                this.addBLELog(
                    `⏱️ 识别到每次BLE通知约 ${this.vitalsFsDiagnostic.medianBatchSize.toFixed(1)} 帧；`
                    + `按帧率 ${measured.toFixed(1)} Hz 计算，不按通知率计算`
                );
            }
            if (antiHalfRateCorrection) {
                this.addBLELog(
                    `🛑 检测到采样率约2:1冲突：时间轴 ${timelineMeasured.toFixed(1)} Hz，`
                    + `累计帧率 ${connectionMeasured.toFixed(1)} Hz；采用 ${selected.toFixed(1)} Hz，`
                    + '禁止心率被缩小一半'
                );
            } else if (staleConnectionRateRejected) {
                this.addBLELog(
                    `🧭 近期时间轴 ${timelineMeasured.toFixed(1)} Hz 已稳定，`
                    + `拒绝使用陈旧连接累计率 ${connectionMeasured.toFixed(1)} Hz，`
                    + '避免心率被放大一倍'
                );
            } else if (rateConflict) {
                this.addBLELog(
                    `⚠️ 采样率来源不一致：时间轴 ${timelineMeasured.toFixed(1)} Hz，`
                    + `累计帧率 ${connectionMeasured.toFixed(1)} Hz，当前采用 ${selected.toFixed(1)} Hz`
                );
            }
            if (relativeError > 0.15) {
                this.addBLELog(
                    `⚠️ 生理算法采样率纠正：配置 ${configured.toFixed(1)} Hz，`
                    + `实测 ${measured.toFixed(1)} Hz，采用 ${selected.toFixed(1)} Hz`
                );
            }
        }
        if (this.bleConnected && typeof this._updateConfigStatusUI === 'function') {
            this._updateConfigStatusUI();
        }
        // A trustworthy measured rate change starts a new pure-rate window.
        // Apply it only after diagnostics have been built; _applyBleSamplingRate
        // clears mixed samples but deliberately keeps the last reliable display.
        if (timelineIsRecentReliable
            && Math.abs(selected - configured) / Math.max(configured, 1) > 0.08) {
            const diagnostic = this.vitalsFsDiagnostic;
            this._applyBleSamplingRate(selected, { updateTarget: false });
            this.vitalsFsDiagnostic = diagnostic;
        }
        return selected;
    }

    _formatVitalsFsDiagnostic(fs) {
        const diagnostic = this.vitalsFsDiagnostic;
        if (!diagnostic || !Number.isFinite(diagnostic.measured)) {
            return `fs=${fs.toFixed(1)}Hz(配置值未验证)`;
        }
        const packing = diagnostic.notificationPacking;
        const batch = packing
            ? `, 帧/通知=${packing.meanFramesPerNotification.toFixed(2)}[${packing.minFramesPerNotification}-${packing.maxFramesPerNotification}]`
            : '';
        const arrival = Number.isFinite(diagnostic.arrivalMeasured)
            ? `, 到达=${diagnostic.arrivalMeasured.toFixed(1)}`
            : '';
        const device = Number.isFinite(diagnostic.deviceMeasured)
            ? `, 设备钟=${diagnostic.deviceMeasured.toFixed(1)}`
            : '';
        const connection = Number.isFinite(diagnostic.connectionMeasured)
            ? `, 累计=${diagnostic.connectionMeasured.toFixed(1)}`
            : '';
        const conflict = diagnostic.antiHalfRateCorrection
            ? ', 已阻断2:1减半'
            : diagnostic.staleConnectionRateRejected
            ? ', 已拒绝陈旧累计率'
            : diagnostic.rateConflict
            ? ', 来源冲突'
            : '';
        return `fs=${fs.toFixed(1)}Hz(实测=${diagnostic.measured.toFixed(1)}, 配置=${diagnostic.configured.toFixed(1)}${arrival}${device}${connection}${batch}${conflict})`;
    }

    _getVitalsOptions() {
        const readBpm = (id, fallback) => {
            const value = Number(document.getElementById(id)?.value);
            return Number.isFinite(value) ? value : fallback;
        };
        const hrMin = Math.max(25, readBpm('heartRateMin', 35));
        const hrMax = Math.max(hrMin + 5, Math.min(300, readBpm('heartRateMax', 260)));
        const rrMin = Math.max(3, readBpm('respRateMin', 6));
        const rrMax = Math.max(rrMin + 2, Math.min(90, readBpm('respRateMax', 60)));
        return {
            hrLow: hrMin / 60,
            hrHigh: hrMax / 60,
            rrLow: rrMin / 60,
            rrHigh: rrMax / 60
        };
    }

    _analyzeVitalsMotion(sampleCount, fs = null) {
        const sampleRate = Number.isFinite(fs) && fs > 0
            ? fs
            : (this.processor && Number.isFinite(this.processor.fs) ? this.processor.fs : 50);
        const accAxes = [
            this.bleBufferACC_X || [],
            this.bleBufferACC_Y || [],
            this.bleBufferACC_Z || []
        ];
        const gyroAxes = [
            this.bleBufferIMU_X || [],
            this.bleBufferIMU_Y || [],
            this.bleBufferIMU_Z || []
        ];
        const length = Math.min(
            sampleCount || 0,
            ...accAxes.map(axis => axis.length)
        );
        const config = { ...{
            blockSec: 0.5,
            gravityTauSec: 0.8,
            minValidFraction: 0.80,
            minNoiseSigmaG: 0.003,
            cleanAbsMarginG: 0.015,
            strongAbsMarginG: 0.080,
            artifactPadBeforeSec: 0.25,
            artifactPadAfterSec: 0.50,
            recentSec: 2,
            hrMinCleanSec: 8,
            rrMinCleanSec: 14,
            hrMinCleanFraction: 0.40,
            rrMinCleanFraction: 0.60
        }, ...(this.vitalsMotionConfig || {}) };
        const quantile = (values, ratio) => {
            const sorted = values.filter(Number.isFinite).slice().sort((a, b) => a - b);
            if (!sorted.length) return NaN;
            const position = Math.max(0, Math.min(1, ratio)) * (sorted.length - 1);
            const low = Math.floor(position);
            const high = Math.ceil(position);
            const fraction = position - low;
            return sorted[low] * (1 - fraction) + sorted[high] * fraction;
        };
        const median = values => quantile(values, 0.5);
        const clamp01 = value => Math.max(0, Math.min(1, value));
        const neutral = (reason = 'IMU样本不足') => ({
            state: 'stationary',
            motionLevel: 'unknown',
            // When IMU is unavailable, fall back to radar-only quality checks.
            // Missing motion metadata must never lower an otherwise valid
            // physiological estimate or be confused with strong movement.
            quality: 1,
            metricQuality: { HR: 1, RR: 1 },
            motionRmsG: 0,
            gyroRmsDps: 0,
            cleanFraction: 1,
            cleanSeconds: length / Math.max(sampleRate, 1),
            recentMotionFraction: 0,
            cleanMask: new Array(Math.max(0, length)).fill(true),
            thresholds: { accClean: 0.02, accStrong: 0.10, gyroClean: 8, gyroStrong: 40 },
            imuAvailable: false,
            unit: 'unknown',
            unitScale: 1,
            reason
        });
        if (length < Math.max(20, Math.round(sampleRate))) return neutral();

        const accStart = accAxes[0].length - length;
        const acc = [0, 1, 2].map(axisIndex => accAxes[axisIndex]
            .slice(accAxes[axisIndex].length - length)
            .map(Number));
        let validSamples = 0;
        const rawMagnitude = new Array(length);
        for (let index = 0; index < length; index++) {
            const finite = acc.every(axis => Number.isFinite(axis[index]));
            if (finite) validSamples += 1;
            rawMagnitude[index] = finite
                ? Math.hypot(acc[0][index], acc[1][index], acc[2][index])
                : NaN;
        }
        const validFraction = validSamples / Math.max(1, length);
        const magnitudeMedian = median(rawMagnitude);
        let unitScale = NaN;
        let unit = 'unknown';
        if (magnitudeMedian >= 0.45 && magnitudeMedian <= 2.2) {
            unitScale = 1;
            unit = 'g';
        } else if (magnitudeMedian >= 4.5 && magnitudeMedian <= 20) {
            unitScale = 1 / 9.80665;
            unit = 'm/s2';
        }
        // Missing accelerometer fields are stored as zero for protocol
        // compatibility. Treat an unknown unit as unavailable rather than
        // interpreting zero as extreme motion and blanking both vitals.
        if (validFraction < config.minValidFraction || !Number.isFinite(unitScale)) {
            return neutral(validFraction < config.minValidFraction ? 'IMU有效率不足' : 'IMU单位/重力幅值未知');
        }

        for (const axis of acc) {
            for (let index = 0; index < axis.length; index++) {
                if (!Number.isFinite(axis[index])) axis[index] = index ? axis[index - 1] : 0;
                axis[index] *= unitScale;
            }
        }

        // A per-sample gravity low-pass follows collar orientation changes in
        // about 0.8 s. This avoids the old full-window mean, where one small
        // rotation contaminated the complete 20/30 s analysis window.
        const alpha = Math.exp(-1 / Math.max(1, sampleRate * config.gravityTauSec));
        const gravity = [acc[0][0], acc[1][0], acc[2][0]];
        const dynamic = new Array(length).fill(0);
        for (let index = 1; index < length; index++) {
            for (let axis = 0; axis < 3; axis++) {
                gravity[axis] = alpha * gravity[axis] + (1 - alpha) * acc[axis][index];
            }
            dynamic[index] = Math.hypot(
                acc[0][index] - gravity[0],
                acc[1][index] - gravity[1],
                acc[2][index] - gravity[2]
            );
        }

        const gyroLength = Math.min(length, ...gyroAxes.map(axis => axis.length));
        const gyroMagnitude = new Array(length).fill(0);
        let gyroAvailable = gyroLength >= length;
        if (gyroAvailable) {
            const gyro = gyroAxes.map(axis => axis.slice(axis.length - length).map(Number));
            gyroAvailable = gyro.every(axis => axis.filter(Number.isFinite).length >= length * config.minValidFraction);
            if (gyroAvailable) {
                for (let index = 0; index < length; index++) {
                    gyroMagnitude[index] = Math.hypot(
                        Number.isFinite(gyro[0][index]) ? gyro[0][index] : 0,
                        Number.isFinite(gyro[1][index]) ? gyro[1][index] : 0,
                        Number.isFinite(gyro[2][index]) ? gyro[2][index] : 0
                    );
                }
            }
        }

        const blockSamples = Math.max(5, Math.round(config.blockSec * sampleRate));
        const blocks = [];
        for (let start = 0; start < length; start += blockSamples) {
            const end = Math.min(length, start + blockSamples);
            if (end - start < Math.max(3, Math.round(blockSamples * 0.5))) continue;
            const energyValues = dynamic.slice(start, end).map(value => value * value).sort((a, b) => a - b);
            const trim = Math.floor(energyValues.length * 0.10);
            const trimmed = energyValues.slice(trim, Math.max(trim + 1, energyValues.length - trim));
            const accEnergy = Math.sqrt(trimmed.reduce((sum, value) => sum + value, 0) / Math.max(1, trimmed.length));
            const gyroValues = gyroMagnitude.slice(start, end);
            const gyroRms = Math.sqrt(gyroValues.reduce((sum, value) => sum + value * value, 0) / Math.max(1, gyroValues.length));
            blocks.push({ start, end, accEnergy, gyroRms, quality: 1, strong: false });
        }
        if (blocks.length < 2) return neutral('IMU分段不足');

        const accEnergies = blocks.map(block => block.accEnergy);
        const gyroEnergies = blocks.map(block => block.gyroRms);
        const accCandidate = Math.min(0.03, Math.max(0, quantile(accEnergies, 0.20) || 0));
        const gyroCandidate = Math.max(0, quantile(gyroEnergies, 0.20) || 0);
        const prior = this._vitalsMotionBaseline;
        const accBaseline = prior && Number.isFinite(prior.acc)
            ? 0.90 * prior.acc + 0.10 * accCandidate
            : accCandidate;
        const gyroBaseline = prior && Number.isFinite(prior.gyro)
            ? 0.90 * prior.gyro + 0.10 * gyroCandidate
            : gyroCandidate;
        const accMad = 1.4826 * median(accEnergies.map(value => Math.abs(value - median(accEnergies))));
        const gyroMad = 1.4826 * median(gyroEnergies.map(value => Math.abs(value - median(gyroEnergies))));
        const accSigma = Math.max(config.minNoiseSigmaG, Math.min(0.02, accMad || 0));
        const gyroSigma = Math.max(0.25, Math.min(8, gyroMad || 0));
        const thresholds = {
            accClean: accBaseline + Math.max(config.cleanAbsMarginG, 3 * accSigma),
            accStrong: accBaseline + Math.max(config.strongAbsMarginG, 8 * accSigma),
            gyroClean: gyroBaseline + Math.max(2, 3 * gyroSigma, 1.5 * gyroBaseline),
            gyroStrong: gyroBaseline + Math.max(20, 8 * gyroSigma, 5 * gyroBaseline)
        };
        const descending = (value, clean, strong) => value <= clean
            ? 1
            : value >= strong
            ? 0
            : clamp01(1 - (value - clean) / Math.max(1e-9, strong - clean));
        for (const block of blocks) {
            const accQuality = descending(block.accEnergy, thresholds.accClean, thresholds.accStrong);
            const gyroQuality = gyroAvailable
                ? descending(block.gyroRms, thresholds.gyroClean, thresholds.gyroStrong)
                : 1;
            block.quality = Math.min(accQuality, gyroQuality);
            block.strong = block.quality <= 0.10;
        }

        const cleanMask = new Array(length).fill(true);
        for (const block of blocks) {
            if (block.quality >= 0.55) continue;
            for (let index = block.start; index < block.end; index++) cleanMask[index] = false;
        }
        const padBefore = Math.round(config.artifactPadBeforeSec * sampleRate);
        const padAfter = Math.round(config.artifactPadAfterSec * sampleRate);
        const dirtyIndices = [];
        for (let index = 0; index < cleanMask.length; index++) {
            if (!cleanMask[index]) dirtyIndices.push(index);
        }
        for (const dirtyIndex of dirtyIndices) {
            const from = Math.max(0, dirtyIndex - padBefore);
            const to = Math.min(length - 1, dirtyIndex + padAfter);
            for (let index = from; index <= to; index++) cleanMask[index] = false;
        }

        const cleanCount = cleanMask.reduce((sum, value) => sum + (value ? 1 : 0), 0);
        const cleanFraction = cleanCount / Math.max(1, length);
        const cleanSeconds = cleanCount / Math.max(1, sampleRate);
        const recentSamples = Math.min(length, Math.max(1, Math.round(config.recentSec * sampleRate)));
        const recentClean = cleanMask.slice(length - recentSamples)
            .reduce((sum, value) => sum + (value ? 1 : 0), 0);
        const recentMotionFraction = 1 - recentClean / recentSamples;
        const recentBlockStart = Math.max(0, length - recentSamples);
        const recentBlocks = blocks.filter(block => block.end > recentBlockStart);
        const recentStrongFraction = recentBlocks.filter(block => block.strong).length
            / Math.max(1, recentBlocks.length);
        let state = 'stationary';
        let motionLevel = 'quiet';
        let quality = clamp01(0.20 + 0.80 * cleanFraction);
        if (recentStrongFraction >= 0.25 || recentMotionFraction >= 0.55) {
            state = 'moving';
            motionLevel = 'gross';
            quality = Math.min(0.05, quality * 0.10);
        } else if (recentMotionFraction >= 0.08 || cleanFraction < 0.90) {
            state = 'recovering';
            motionLevel = 'mild';
            quality = Math.max(0.30, quality * 0.90);
        }

        // Only learn a new stationary baseline when the most recent collar
        // samples are not moving. This prevents sustained walking from being
        // normalized into a false "quiet" baseline.
        if (state !== 'moving') {
            this._vitalsMotionBaseline = { acc: accBaseline, gyro: gyroBaseline };
        }
        const motionRmsG = Math.sqrt(dynamic.reduce((sum, value) => sum + value * value, 0) / length);
        const gyroRmsDps = Math.sqrt(gyroMagnitude.reduce((sum, value) => sum + value * value, 0) / length);
        const metricQuality = {
            HR: state === 'moving' ? 0.02 : clamp01(
                quality
                * Math.min(1, cleanSeconds / config.hrMinCleanSec)
                * Math.min(1, cleanFraction / config.hrMinCleanFraction)
            ),
            RR: state === 'moving' ? 0.02 : clamp01(
                quality
                * Math.min(1, cleanSeconds / config.rrMinCleanSec)
                * Math.min(1, cleanFraction / config.rrMinCleanFraction)
            )
        };

        return {
            state,
            motionLevel,
            quality,
            metricQuality,
            motionRmsG,
            gyroRmsDps,
            cleanFraction,
            cleanSeconds,
            recentMotionFraction,
            cleanMask,
            thresholds,
            imuAvailable: true,
            gyroAvailable,
            unit,
            unitScale,
            validFraction,
            blockQuality: blocks.map(block => block.quality),
            reason: state === 'moving'
                ? '近期持续强运动'
                : state === 'recovering'
                ? '局部运动段已屏蔽/恢复中'
                : '运动质量良好'
        };
    }

    _getVitalsMotionQuality(sampleCount, fs = null) {
        return this._analyzeVitalsMotion(sampleCount, fs).quality;
    }

    _initVitalsExtractor() {
        if (!window.VitalsExtractor) {
            const runtimeEl = document.getElementById('vitalsRuntime');
            if (runtimeEl) runtimeEl.textContent = '算法未加载';
            return null;
        }
        const fs = this._getVitalsFs();
        const options = this._getVitalsOptions();
        this.vitalsExtractor = window.VitalsExtractor.create(this.vitalsMethod, fs, options);
        if (this.processor) this.processor.vitalsOptions = { ...options };
        return this.vitalsExtractor;
    }

    _ensureVitalsRealtime() {
        if (this.vitalsTimer) return;
        if (!window.VitalsExtractor) {
            const runtimeEl = document.getElementById('vitalsRuntime');
            if (runtimeEl) runtimeEl.textContent = '算法未加载';
            return;
        }
        const available = Math.min(this.bleBufferI.length, this.bleBufferQ.length);
        if (available <= 0) return;
        this.startVitalsRealtime();
    }

    updateVitalsSettings(restartTimer = true) {
        const winEl = document.getElementById('vitalsWindowSec');
        const periodEl = document.getElementById('vitalsPeriodSec');
        const win = parseFloat(winEl?.value);
        const period = parseFloat(periodEl?.value);
        if (Number.isFinite(win)) this.vitalsWindowSec = Math.max(12, Math.min(120, win));
        if (Number.isFinite(period)) this.vitalsPeriodSec = Math.max(1, Math.min(30, period));
        if (winEl) winEl.value = String(this.vitalsWindowSec);
        if (periodEl) periodEl.value = String(this.vitalsPeriodSec);
        if (restartTimer && this.vitalsTimer) this.startVitalsRealtime();
    }

    changeVitalsMethod(method) {
        this.vitalsMethod = method === 'cwt' ? 'cwt' : 'freqBP';
        this._initVitalsExtractor();
        this.vitalsFirstResultLogged = false;
        this._resetVitalsStability();
        const runtimeEl = document.getElementById('vitalsRuntime');
        if (runtimeEl) runtimeEl.textContent = this.vitalsMethod === 'cwt' ? 'CWT/ICWT 已选择' : '频域带通已选择';
        this._runVitalsCycle();
    }

    startVitalsRealtime() {
        this.updateVitalsSettings(false);
        this.stopVitalsRealtime();
        this._initVitalsExtractor();
        this._runVitalsCycle();
        this.vitalsTimer = setInterval(() => this._runVitalsCycle(), this.vitalsPeriodSec * 1000);
    }

    stopVitalsRealtime() {
        if (this.vitalsTimer) {
            clearInterval(this.vitalsTimer);
            this.vitalsTimer = null;
        }
        this._resetVitalsStability();
    }

    _resetVitalsStability() {
        this.vitalsHrHistory = [];
        this.vitalsRrHistory = [];
        this.vitalsLastStableHR = null;
        this.vitalsLastStableRR = null;
        this.vitalsAcceptedCycles = 0;
        this.vitalsLastGuardResult = null;
        this.vitalsLastGuardTs = 0;
        if (this.vitalsTracker && typeof this.vitalsTracker.reset === 'function') {
            this.vitalsTracker.reset();
        }
    }

    _median(values) {
        const arr = values.filter(v => Number.isFinite(v)).slice().sort((a, b) => a - b);
        if (!arr.length) return NaN;
        const mid = Math.floor(arr.length / 2);
        return arr.length % 2 ? arr[mid] : (arr[mid - 1] + arr[mid]) / 2;
    }

    _pushVitalsHistory(kind, value) {
        const history = kind === 'hr' ? this.vitalsHrHistory : this.vitalsRrHistory;
        history.push(value);
        while (history.length > this.vitalsSmoothN) history.shift();
        return this._median(history);
    }

    _guardSupportsValue(kind, value, guardResult) {
        if (!guardResult || !Number.isFinite(value)) return true;
        const guardValue = kind === 'hr' ? guardResult.HR_bpm : guardResult.RR_bpm;
        const guardConfidence = kind === 'hr' ? guardResult.HR_confidence : guardResult.RR_confidence;
        if (!Number.isFinite(guardValue) || guardConfidence < this.vitalsMinConfidence) return true;
        const maxDiff = kind === 'hr' ? this.vitalsGuardDiffBpm : 4;
        return Math.abs(value - guardValue) <= maxDiff;
    }

    _stabilizeVitalsResult(result, guardResult = null, motionContext = {}) {
        const rawHR = result ? result.HR_bpm : NaN;
        const rawRR = result ? result.RR_bpm : NaN;
        let candidateHR = rawHR;
        let candidateRR = rawRR;
        let hrConfidence = Number.isFinite(result?.HR_confidence) ? result.HR_confidence : 0;
        let rrConfidence = Number.isFinite(result?.RR_confidence) ? result.RR_confidence : 0;
        const messages = [];

        if (this.vitalsTracker && typeof this.vitalsTracker.update === 'function') {
            const ranges = this._getVitalsOptions();
            const sanitize = (source) => {
                if (!source) return null;
                const copy = { ...source };
                if (!(copy.HR_bpm >= ranges.hrLow * 60 && copy.HR_bpm <= ranges.hrHigh * 60)) {
                    copy.HR_bpm = NaN;
                    copy.HR_confidence = 0;
                }
                if (!(copy.RR_bpm >= ranges.rrLow * 60 && copy.RR_bpm <= ranges.rrHigh * 60)) {
                    copy.RR_bpm = NaN;
                    copy.RR_confidence = 0;
                }
                return copy;
            };
            const primary = sanitize(result);
            const guard = sanitize(guardResult);
            const trackerContext = {
                timestampMs: Number.isFinite(motionContext.timestampMs)
                    ? motionContext.timestampMs
                    : Date.now(),
                motionState: motionContext.motionState || motionContext.state || 'stationary',
                motionQuality: motionContext.motionQuality || motionContext.metricQuality
                    || (Number.isFinite(motionContext.quality) ? motionContext.quality : undefined)
            };
            const tracked = this.vitalsTracker.update(primary, guard, trackerContext);
            const primaryHrValid = Number.isFinite(primary?.HR_bpm)
                && (primary.HR_confidence || 0) >= this.vitalsMinConfidence;
            const primaryRrValid = Number.isFinite(primary?.RR_bpm)
                && (primary.RR_confidence || 0) >= this.vitalsMinConfidence;
            candidateHR = primaryHrValid ? primary.HR_bpm : guard?.HR_bpm;
            candidateRR = primaryRrValid ? primary.RR_bpm : guard?.RR_bpm;
            hrConfidence = tracked.HR_confidence;
            rrConfidence = tracked.RR_confidence;
            this.vitalsLastStableHR = tracked.HR_valid ? tracked.HR_bpm : null;
            this.vitalsLastStableRR = tracked.RR_valid ? tracked.RR_bpm : null;
            this.vitalsHrHistory = tracked.HR_valid
                ? [...this.vitalsHrHistory, tracked.HR_bpm].slice(-this.historyMaxLength)
                : [];
            this.vitalsRrHistory = tracked.RR_valid
                ? [...this.vitalsRrHistory, tracked.RR_bpm].slice(-this.historyMaxLength)
                : [];
            this.vitalsAcceptedCycles = tracked.HR_valid || tracked.RR_valid
                ? Math.min(this.vitalsWarmupMinCycles, this.vitalsAcceptedCycles + 1)
                : 0;
            return {
                rawHR,
                rawRR,
                candidateHR,
                candidateRR,
                stableHR: tracked.HR_bpm,
                stableRR: tracked.RR_bpm,
                hrValid: tracked.HR_valid,
                rrValid: tracked.RR_valid,
                hrConfidence,
                rrConfidence,
                guardHR: guard?.HR_bpm,
                guardRR: guard?.RR_bpm,
                held: tracked.held || { HR: false, RR: false },
                fresh: tracked.fresh || { HR: false, RR: false },
                ageMs: tracked.ageMs || { HR: null, RR: null },
                status: tracked.status || { HR: tracked.HR_valid ? 'fresh' : 'warming', RR: tracked.RR_valid ? 'fresh' : 'warming' },
                messages: tracked.messages || []
            };
        }

        if (guardResult && Number.isFinite(guardResult.HR_bpm) && guardResult.HR_confidence >= this.vitalsMinConfidence) {
            const shortBad = !Number.isFinite(candidateHR) || hrConfidence < this.vitalsMinConfidence;
            const shortDisagrees = Number.isFinite(candidateHR) && Math.abs(candidateHR - guardResult.HR_bpm) > this.vitalsGuardDiffBpm;
            if (shortBad || shortDisagrees) {
                candidateHR = guardResult.HR_bpm;
                hrConfidence = guardResult.HR_confidence;
                messages.push(`HR采用长窗校验: ${candidateHR.toFixed(1)}`);
            }
        }

        if (guardResult && Number.isFinite(guardResult.RR_bpm) && guardResult.RR_confidence >= this.vitalsMinConfidence) {
            const shortBad = !Number.isFinite(candidateRR) || rrConfidence < this.vitalsMinConfidence;
            const shortDisagrees = Number.isFinite(candidateRR) && Math.abs(candidateRR - guardResult.RR_bpm) > 4;
            if (shortBad || shortDisagrees) {
                candidateRR = guardResult.RR_bpm;
                rrConfidence = guardResult.RR_confidence;
                messages.push(`RR采用长窗校验: ${candidateRR.toFixed(1)}`);
            }
        }

        const hrInRange = Number.isFinite(candidateHR) && candidateHR >= 40 && candidateHR <= 220;
        const rrInRange = Number.isFinite(candidateRR) && candidateRR >= 6 && candidateRR <= 30;
        const hrConfident = hrInRange && hrConfidence >= this.vitalsMinConfidence && this._guardSupportsValue('hr', candidateHR, guardResult);
        const rrConfident = rrInRange && rrConfidence >= this.vitalsMinConfidence && this._guardSupportsValue('rr', candidateRR, guardResult);

        let stableHR = NaN;
        if (hrConfident) {
            const jump = Number.isFinite(this.vitalsLastStableHR)
                ? Math.abs(candidateHR - this.vitalsLastStableHR)
                : 0;
            const guardConfirmsDrop = Number.isFinite(this.vitalsLastStableHR)
                && candidateHR < this.vitalsLastStableHR - this.vitalsHrDropGuardBpm
                && guardResult
                && Number.isFinite(guardResult.HR_bpm)
                && guardResult.HR_confidence >= this.vitalsMinConfidence
                && guardResult.HR_bpm < this.vitalsLastStableHR - this.vitalsHrDropGuardBpm
                && Math.abs(guardResult.HR_bpm - candidateHR) <= this.vitalsGuardDiffBpm;
            if (jump > 25) {
                stableHR = this.vitalsLastStableHR;
                messages.push(`HR跳变保护: ${candidateHR.toFixed(1)}→${stableHR.toFixed(1)}`);
            } else if (guardConfirmsDrop) {
                this.vitalsHrHistory = [candidateHR];
                stableHR = candidateHR;
                this.vitalsLastStableHR = stableHR;
                messages.push(`HR下降已由长窗确认: ${candidateHR.toFixed(1)}`);
            } else if (
                Number.isFinite(this.vitalsLastStableHR)
                && candidateHR < this.vitalsLastStableHR - this.vitalsHrDropGuardBpm
            ) {
                stableHR = this.vitalsLastStableHR;
                messages.push(`HR向下防抖: ${candidateHR.toFixed(1)}→${stableHR.toFixed(1)}`);
            } else {
                const median = this._pushVitalsHistory('hr', candidateHR);
                stableHR = this.vitalsHrHistory.length >= 3 ? median : candidateHR;
                this.vitalsLastStableHR = stableHR;
            }
        } else if (hrInRange && Number.isFinite(this.vitalsLastStableHR)) {
            stableHR = this.vitalsLastStableHR;
            messages.push(`HR低置信度(${hrConfidence.toFixed(2)})，沿用稳定值`);
        } else if (Number.isFinite(this.vitalsLastStableHR)) {
            stableHR = this.vitalsLastStableHR;
            messages.push('HR越界/无峰，沿用稳定值');
        }

        let stableRR = NaN;
        if (rrConfident) {
            const jump = Number.isFinite(this.vitalsLastStableRR)
                ? Math.abs(candidateRR - this.vitalsLastStableRR)
                : 0;
            if (jump > 8) {
                stableRR = this.vitalsLastStableRR;
                messages.push(`RR跳变保护: ${candidateRR.toFixed(1)}→${stableRR.toFixed(1)}`);
            } else {
                const median = this._pushVitalsHistory('rr', candidateRR);
                stableRR = this.vitalsRrHistory.length >= 3 ? median : candidateRR;
                this.vitalsLastStableRR = stableRR;
            }
        } else if (rrInRange && Number.isFinite(this.vitalsLastStableRR)) {
            stableRR = this.vitalsLastStableRR;
            messages.push(`RR低置信度(${rrConfidence.toFixed(2)})，沿用稳定值`);
        } else if (Number.isFinite(this.vitalsLastStableRR)) {
            stableRR = this.vitalsLastStableRR;
            messages.push('RR越界/无峰，沿用稳定值');
        }

        if (hrConfident && rrConfident) {
            this.vitalsAcceptedCycles = Math.min(this.vitalsWarmupMinCycles, this.vitalsAcceptedCycles + 1);
        }

        const warmupReady = this.vitalsAcceptedCycles >= this.vitalsWarmupMinCycles;
        if (!warmupReady) {
            messages.push(`预热中 ${this.vitalsAcceptedCycles}/${this.vitalsWarmupMinCycles}`);
            stableHR = NaN;
            stableRR = NaN;
        }

        return {
            rawHR,
            rawRR,
            candidateHR,
            candidateRR,
            stableHR,
            stableRR,
            hrValid: Number.isFinite(stableHR),
            rrValid: Number.isFinite(stableRR),
            hrConfidence,
            rrConfidence,
            guardHR: guardResult?.HR_bpm,
            guardRR: guardResult?.RR_bpm,
            held: { HR: false, RR: false },
            fresh: { HR: Number.isFinite(stableHR), RR: Number.isFinite(stableRR) },
            ageMs: { HR: Number.isFinite(stableHR) ? 0 : null, RR: Number.isFinite(stableRR) ? 0 : null },
            status: {
                HR: Number.isFinite(stableHR) ? 'fresh' : 'warming',
                RR: Number.isFinite(stableRR) ? 'fresh' : 'warming'
            },
            messages
        };
    }

    _applyVitalsMotionEvidence(result, motion) {
        if (!result || !motion) return result;
        const clamp01 = value => Math.max(0, Math.min(1, Number(value) || 0));
        const hrQuality = clamp01(motion.metricQuality?.HR ?? motion.quality);
        const rrQuality = clamp01(motion.metricQuality?.RR ?? motion.quality);
        const config = this.vitalsMotionConfig || {};
        const hrHasCleanData = !motion.imuAvailable
            || (motion.cleanSeconds >= (config.hrMinCleanSec || 8)
                && motion.cleanFraction >= (config.hrMinCleanFraction || 0.40));
        const rrHasCleanData = !motion.imuAvailable
            || (motion.cleanSeconds >= (config.rrMinCleanSec || 14)
                && motion.cleanFraction >= (config.rrMinCleanFraction || 0.60));

        result.HR_confidence = hrHasCleanData
            ? clamp01((result.HR_confidence || 0) * hrQuality)
            : 0;
        result.RR_confidence = rrHasCleanData
            ? clamp01((result.RR_confidence || 0) * rrQuality)
            : 0;
        result.motion = {
            state: motion.state,
            level: motion.motionLevel,
            quality: motion.quality,
            metricQuality: { HR: hrQuality, RR: rrQuality },
            cleanFraction: motion.cleanFraction,
            cleanSeconds: motion.cleanSeconds,
            recentMotionFraction: motion.recentMotionFraction,
            motionRmsG: motion.motionRmsG,
            gyroRmsDps: motion.gyroRmsDps,
            imuAvailable: motion.imuAvailable,
            unit: motion.unit,
            reason: motion.reason,
            hrHasCleanData,
            rrHasCleanData
        };
        return result;
    }

    _publishVitalsResult(stable, result) {
        const nowMs = Date.now();
        const heartRate = stable.hrValid ? Number(stable.stableHR.toFixed(1)) : null;
        const respiratoryRate = stable.rrValid ? Number(stable.stableRR.toFixed(1)) : null;
        if (!this.vitalsLastReliable) {
            this.vitalsLastReliable = {
                HR: { value: NaN, timestampMs: NaN, confidence: 0 },
                RR: { value: NaN, timestampMs: NaN, confidence: 0 }
            };
        }
        const hrFresh = stable.fresh?.HR ?? (Number.isFinite(heartRate) && !stable.held?.HR);
        const rrFresh = stable.fresh?.RR ?? (Number.isFinite(respiratoryRate) && !stable.held?.RR);
        if (hrFresh && Number.isFinite(heartRate)) {
            this.vitalsLastReliable.HR = {
                value: heartRate,
                timestampMs: nowMs,
                confidence: stable.hrConfidence
            };
        }
        if (rrFresh && Number.isFinite(respiratoryRate)) {
            this.vitalsLastReliable.RR = {
                value: respiratoryRate,
                timestampMs: nowMs,
                confidence: stable.rrConfidence
            };
        }
        // Internally only a value inside the time-limited tracker hold remains
        // current. The UI may still show an older historical value in grey,
        // explicitly labelled with its age, so information never vanishes or
        // gets mistaken for a fresh measurement.
        this.currentHeartRate = heartRate;
        this.currentRespiratoryRate = respiratoryRate;
        if (Number.isFinite(heartRate)) this.lastStableHeartRate = heartRate;
        if (Number.isFinite(respiratoryRate)) this.lastStableRespRate = respiratoryRate;

        const displayHeartRate = Number.isFinite(heartRate)
            ? heartRate
            : this.vitalsLastReliable.HR.value;
        const displayRespiratoryRate = Number.isFinite(respiratoryRate)
            ? respiratoryRate
            : this.vitalsLastReliable.RR.value;
        const ageFor = (kind, suppliedAge) => Number.isFinite(suppliedAge)
            ? suppliedAge
            : Number.isFinite(this.vitalsLastReliable[kind].timestampMs)
            ? Math.max(0, nowMs - this.vitalsLastReliable[kind].timestampMs)
            : null;
        const hrAgeMs = ageFor('HR', stable.ageMs?.HR);
        const rrAgeMs = ageFor('RR', stable.ageMs?.RR);
        const hrStatus = Number.isFinite(heartRate)
            ? (stable.status?.HR || (hrFresh ? 'fresh' : 'held'))
            : Number.isFinite(displayHeartRate) ? 'expired' : 'warming';
        const rrStatus = Number.isFinite(respiratoryRate)
            ? (stable.status?.RR || (rrFresh ? 'fresh' : 'held'))
            : Number.isFinite(displayRespiratoryRate) ? 'expired' : 'warming';
        this.vitalsDisplayState = {
            timestampMs: nowMs,
            HR: { value: displayHeartRate, currentValue: heartRate, status: hrStatus, ageMs: hrAgeMs, isFresh: hrStatus === 'fresh' },
            RR: { value: displayRespiratoryRate, currentValue: respiratoryRate, status: rrStatus, ageMs: rrAgeMs, isFresh: rrStatus === 'fresh' },
            motion: result?.motion || null
        };

        const setVitalText = (ids, value) => {
            const text = Number.isFinite(value) ? `${value.toFixed(1)} bpm` : '-- bpm';
            ids.forEach(id => {
                const element = document.getElementById(id);
                if (element) element.textContent = text;
            });
        };
        setVitalText(
            ['bleCurrentHR', 'vitalsHRNew', 'bleCurrentHeartRate', 'currentHeartRate', 'bleAvgHeartRate'],
            displayHeartRate
        );
        setVitalText(
            ['bleCurrentResp', 'vitalsRRNew', 'bleCurrentRespRate', 'currentRespRate', 'bleAvgRespRate'],
            displayRespiratoryRate
        );

        const qualityScore = Number.isFinite(result?.quality?.score) ? result.quality.score : 0;
        const hrElement = document.getElementById('bleCurrentHR');
        const rrElement = document.getElementById('bleCurrentResp');
        const statusLabel = (kind, status, ageMs) => status === 'fresh'
            ? '实时测量'
            : status === 'held'
            ? `运动/低质量保持，${Math.round((ageMs || 0) / 1000)}秒前可靠值`
            : status === 'expired'
            ? `历史值，${Math.round((ageMs || 0) / 1000)}秒前可靠；当前不可测`
            : '等待首次可靠测量';
        const applyElementState = (ids, kind, status, ageMs, confidence) => {
            const colors = { fresh: '#adcada', held: '#f5b942', expired: '#9f9c9d', warming: '#9f9c9d' };
            for (const id of ids) {
                const element = document.getElementById(id);
                if (!element) continue;
                element.dataset.vitalsState = status;
                element.style.color = colors[status] || colors.warming;
                element.title = `${kind}：${statusLabel(kind, status, ageMs)}；置信度 ${(confidence * 100).toFixed(0)}%，信号质量 ${(qualityScore * 100).toFixed(0)}%`;
            }
        };
        applyElementState(['bleCurrentHR', 'vitalsHRNew'], 'HR', hrStatus, hrAgeMs, stable.hrConfidence);
        applyElementState(['bleCurrentResp', 'vitalsRRNew'], 'RR', rrStatus, rrAgeMs, stable.rrConfidence);

        const motionStatusEl = document.getElementById('vitalsMotionStatus');
        if (motionStatusEl) {
            const heldParts = [];
            if (hrStatus === 'held') heldParts.push(`HR保持${Math.round((hrAgeMs || 0) / 1000)}s`);
            if (rrStatus === 'held') heldParts.push(`RR保持${Math.round((rrAgeMs || 0) / 1000)}s`);
            const expiredParts = [];
            if (hrStatus === 'expired') expiredParts.push(`HR历史值${Math.round((hrAgeMs || 0) / 1000)}s`);
            if (rrStatus === 'expired') expiredParts.push(`RR历史值${Math.round((rrAgeMs || 0) / 1000)}s`);
            const motionState = result?.motion?.state;
            const octaveDecision = result?.diagnostics?.octaveCorrection;
            if (expiredParts.length) {
                motionStatusEl.textContent = `⚠️ 当前不可测 · ${expiredParts.join(' · ')}`;
                motionStatusEl.style.background = '#49454a';
                motionStatusEl.style.color = '#d2cdd3';
            } else if (octaveDecision?.ambiguous) {
                const low = Number.isFinite(octaveDecision.originalBpm)
                    ? octaveDecision.originalBpm.toFixed(0)
                    : '--';
                const highCandidate = Number.isFinite(octaveDecision.upperBpm)
                    ? octaveDecision.upperBpm
                    : octaveDecision.alternativeBpm;
                const high = Number.isFinite(highCandidate)
                    ? highCandidate.toFixed(0)
                    : '--';
                motionStatusEl.textContent = `⚠️ HR低/高峰歧义 ${low}/${high} · 保持最近可靠值`;
                motionStatusEl.style.background = '#5a4729';
                motionStatusEl.style.color = '#ffd27a';
            } else if (heldParts.length || motionState === 'moving') {
                motionStatusEl.textContent = `🐾 运动保护 · ${heldParts.join(' · ') || '等待可靠值'}`;
                motionStatusEl.style.background = '#5a4729';
                motionStatusEl.style.color = '#ffd27a';
            } else if (motionState === 'recovering') {
                motionStatusEl.textContent = '↻ 局部运动已剔除 · 正在恢复实时测量';
                motionStatusEl.style.background = '#31495a';
                motionStatusEl.style.color = '#9fd5f5';
            } else if (hrStatus === 'fresh' || rrStatus === 'fresh') {
                motionStatusEl.textContent = '● 实时可靠测量';
                motionStatusEl.style.background = '#294a3b';
                motionStatusEl.style.color = '#8ce0b9';
            } else {
                motionStatusEl.textContent = '等待首次可靠测量';
                motionStatusEl.style.background = '#3a3848';
                motionStatusEl.style.color = '#bdb9ca';
            }
        }

        if (this.bleRecordingFlag === 1 && (Number.isFinite(heartRate) || Number.isFinite(respiratoryRate))) {
            if (!this._bleWindowHistory) this._bleWindowHistory = [];
            this._bleWindowHistory.push({
                t: Date.now(),
                heartRate,
                respiratoryRate,
                heartRateConfidence: stable.hrConfidence,
                respiratoryRateConfidence: stable.rrConfidence,
                heartRateStatus: hrStatus,
                respiratoryRateStatus: rrStatus,
                heartRateAgeMs: hrAgeMs,
                respiratoryRateAgeMs: rrAgeMs,
                motionState: result?.motion?.state || 'unknown',
                signalQuality: qualityScore,
                samplingRate: result?.fs
            });
            if (this._bleWindowHistory.length > 600) {
                this._bleWindowHistory.splice(0, this._bleWindowHistory.length - 600);
            }
        }

        if (this.petEmotionMonitorEnabled && this.petEmotionMonitor
            && hrFresh && rrFresh
            && Number.isFinite(heartRate) && Number.isFinite(respiratoryRate)) {
            const lastActivity = this.activityMonitor?.activityHistory?.length
                ? this.activityMonitor.activityHistory[this.activityMonitor.activityHistory.length - 1]
                : null;
            this.petEmotionMonitor.addVital(
                heartRate,
                respiratoryRate,
                lastActivity ? lastActivity.enmo : 0,
                Date.now()
            );
        }
    }

    _buildVitalsHintMeta(kind) {
        const display = this.vitalsDisplayState?.[kind] || null;
        const lastReliable = this.vitalsLastReliable?.[kind] || null;
        const history = kind === 'HR' ? this.vitalsHrHistory : this.vitalsRrHistory;
        const liveAgeMs = Number.isFinite(lastReliable?.timestampMs)
            ? Math.max(0, Date.now() - lastReliable.timestampMs)
            : (display?.ageMs ?? null);
        const displayStatus = display?.status || 'warming';
        const status = Number.isFinite(liveAgeMs) && liveAgeMs > this.vitalsHoldMaxMs
            && displayStatus !== 'warming'
            ? 'expired'
            : displayStatus;
        return {
            status,
            ageMs: liveAgeMs,
            confidence: Number.isFinite(lastReliable?.confidence)
                ? lastReliable.confidence
                : 0,
            freshWindowCount: Array.isArray(history) ? history.length : 0,
            ambiguous: kind === 'HR'
                ? this.lastVitalsResult?.diagnostics?.octaveCorrection?.ambiguous === true
                : false
        };
    }

    _runVitalsGuardWindow(available, fs) {
        const now = performance.now();
        if (this.vitalsLastGuardResult && now - this.vitalsLastGuardTs < this.vitalsGuardMaxAgeMs) {
            return this.vitalsLastGuardResult;
        }
        const guardSamples = Math.round(this.vitalsGuardWindowSec * fs);
        if (available < guardSamples || !this.vitalsExtractor) return this.vitalsLastGuardResult;

        const start = Math.max(0, available - guardSamples);
        const iData = this.bleBufferI.slice(start, available);
        const qData = this.bleBufferQ.slice(start, available);
        const motion = this._analyzeVitalsMotion(guardSamples, fs);
        try {
            const guardResult = this.vitalsExtractor.run(iData, qData, fs, {
                // The long window is an independent guard, not a second copy of
                // the short-window tracker. Feeding it the current HR hint would
                // reproduce and reinforce a wrong 40 bpm baseline.
                rrHintFreq: NaN,
                hrHintFreq: NaN,
                rrHintMeta: null,
                hrHintMeta: null,
                motionQuality: motion.quality,
                motionCleanMask: motion.cleanMask
            });
            this._applyVitalsMotionEvidence(guardResult, motion);
            this.vitalsLastGuardResult = guardResult;
            this.vitalsLastGuardTs = now;
            return guardResult;
        } catch (e) {
            console.warn('新版算法长窗校验失败:', e);
            return this.vitalsLastGuardResult;
        }
    }

    _runVitalsCycle() {
        const runtimeEl = document.getElementById('vitalsRuntime');
        const hrEl = document.getElementById('vitalsHRNew');
        const rrEl = document.getElementById('vitalsRRNew');
        if (!this.vitalsExtractor) this._initVitalsExtractor();
        if (!this.vitalsExtractor) {
            if (runtimeEl) runtimeEl.textContent = '算法未加载';
            return;
        }

        const fs = this._getVitalsFs();
        const configuredBands = this._getVitalsOptions();
        const minimumFs = configuredBands.hrHigh / 0.45;
        if (fs < minimumFs) {
            const empty = {
                HR_bpm: NaN,
                RR_bpm: NaN,
                HR_confidence: 0,
                RR_confidence: 0,
                reason: '采样率不足'
            };
            const stable = this._stabilizeVitalsResult(empty, null);
            this._publishVitalsResult(stable, { ...empty, fs, quality: { score: 0 } });
            if (runtimeEl) {
                runtimeEl.textContent = `采样率 ${fs.toFixed(1)}Hz 不足：当前HR上限需至少 ${minimumFs.toFixed(1)}Hz，请确认Tcycle指令`;
            }
            return;
        }
        const windowSamples = Math.max(1, Math.round(this.vitalsWindowSec * fs));
        const available = Math.min(this.bleBufferI.length, this.bleBufferQ.length);
        const connectionRate = this._estimateCurrentBleFs();
        const measuredFs = Number.isFinite(connectionRate) ? connectionRate : fs;
        // Do not call a nominal 20 s estimator with 12-13 s of data. Adjacent
        // startup calls otherwise share >92% of their samples and can establish
        // a false low baseline before the first complete analysis window exists.
        const requiredSamples = Math.max(128, windowSamples);
        if (available < requiredSamples) {
            if (runtimeEl) {
                runtimeEl.textContent = `等待数据 ${available}/${requiredSamples}，${this._formatVitalsFsDiagnostic(fs)}，累计帧率=${measuredFs.toFixed(1)}Hz`;
            }
            return;
        }

        const start = Math.max(0, available - windowSamples);
        const iData = this.bleBufferI.slice(start, available);
        const qData = this.bleBufferQ.slice(start, available);
        const label = `vitals-${this.vitalsMethod}`;
        const t0 = performance.now();
        try {
            this.vitalsExtractor.setSampleRate(fs);
            const rrHintFreq = Number.isFinite(this.vitalsLastStableRR) ? this.vitalsLastStableRR / 60 : NaN;
            const hrHintFreq = Number.isFinite(this.vitalsLastStableHR) ? this.vitalsLastStableHR / 60 : NaN;
            const motion = this._analyzeVitalsMotion(iData.length, fs);
            const result = this.vitalsExtractor.run(iData, qData, fs, {
                rrHintFreq,
                hrHintFreq,
                rrHintMeta: this._buildVitalsHintMeta('RR'),
                hrHintMeta: this._buildVitalsHintMeta('HR'),
                motionQuality: motion.quality,
                motionCleanMask: motion.cleanMask
            });
            this._applyVitalsMotionEvidence(result, motion);
            const elapsed = performance.now() - t0;
            const guardResult = this._runVitalsGuardWindow(available, fs);
            const stable = this._stabilizeVitalsResult(result, guardResult, {
                timestampMs: Date.now(),
                motionState: motion.state,
                motionQuality: motion.metricQuality
            });
            result.raw_HR_bpm = stable.rawHR;
            result.raw_RR_bpm = stable.rawRR;
            result.candidate_HR_bpm = stable.candidateHR;
            result.candidate_RR_bpm = stable.candidateRR;
            result.HR_bpm = stable.stableHR;
            result.RR_bpm = stable.stableRR;
            result.stability = {
                hrHistory: this.vitalsHrHistory.slice(),
                rrHistory: this.vitalsRrHistory.slice(),
                acceptedCycles: this.vitalsAcceptedCycles,
                guard: guardResult ? {
                    HR_bpm: guardResult.HR_bpm,
                    RR_bpm: guardResult.RR_bpm,
                    HR_confidence: guardResult.HR_confidence,
                    RR_confidence: guardResult.RR_confidence
                } : null,
                messages: stable.messages
            };
            result.continuity = {
                held: stable.held,
                fresh: stable.fresh,
                ageMs: stable.ageMs,
                status: stable.status
            };
            this.lastVitalsResult = result;
            this._publishVitalsResult(stable, result);
            // The canonical result and its waveforms are produced in the same
            // cycle; render them together so the display can never lag behind
            // and fall back to the removed legacy estimator.
            this.updateBluetoothVitalSigns();
            const displayHR = this.vitalsDisplayState?.HR?.value;
            const displayRR = this.vitalsDisplayState?.RR?.value;
            if (hrEl) hrEl.textContent = Number.isFinite(displayHR) ? `${displayHR.toFixed(1)} bpm` : '-- bpm';
            if (rrEl) rrEl.textContent = Number.isFinite(displayRR) ? `${displayRR.toFixed(1)} bpm` : '-- bpm';
            if (runtimeEl) {
                const name = this.vitalsMethod === 'cwt' ? 'CWT' : '频域';
                const rrTop = result.diagnostics?.rrPeaksTop?.[0];
                const hrTop = result.diagnostics?.hrPeaksRobustTop?.[0];
                const peakState = (stable.hrValid && stable.rrValid)
                    ? '运行中'
                    : `未找到有效峰值: HR=${Number.isFinite(stable.rawHR) ? stable.rawHR.toFixed(1) : '--'}, RR=${Number.isFinite(stable.rawRR) ? stable.rawRR.toFixed(1) : '--'}`;
                const rawText = `短窗HR=${Number.isFinite(stable.rawHR) ? stable.rawHR.toFixed(1) : '--'}, 候选HR=${Number.isFinite(stable.candidateHR) ? stable.candidateHR.toFixed(1) : '--'}, 中位HR=${stable.hrValid ? stable.stableHR.toFixed(1) : '--'}, 短窗RR=${Number.isFinite(stable.rawRR) ? stable.rawRR.toFixed(1) : '--'}, 候选RR=${Number.isFinite(stable.candidateRR) ? stable.candidateRR.toFixed(1) : '--'}, 中位RR=${stable.rrValid ? stable.stableRR.toFixed(1) : '--'}`;
                const peakText = `，RR峰=${rrTop ? rrTop.bpm.toFixed(1) : '--'}, HR峰=${hrTop ? hrTop.bpm.toFixed(1) : '--'}, 抑制RR=${Number.isFinite(result.diagnostics?.rrUsedForHarmonic) ? (result.diagnostics.rrUsedForHarmonic * 60).toFixed(1) : '--'}`;
                const confidenceText = `，置信度HR=${stable.hrConfidence.toFixed(2)}, RR=${stable.rrConfidence.toFixed(2)}`;
                const octaveDecision = result.diagnostics?.octaveCorrection;
                const ambiguityHigh = Number.isFinite(octaveDecision?.upperBpm)
                    ? octaveDecision.upperBpm
                    : octaveDecision?.alternativeBpm;
                const octaveText = octaveDecision?.ambiguous
                    ? `，HR低/高峰歧义=${Number.isFinite(octaveDecision.originalBpm) ? octaveDecision.originalBpm.toFixed(1) : '--'}/${Number.isFinite(ambiguityHigh) ? ambiguityHigh.toFixed(1) : '--'}(不更新基线)`
                    : octaveDecision?.applied
                    ? `，HR倍频纠正=${octaveDecision.originalBpm.toFixed(1)}→${octaveDecision.selectedBpm.toFixed(1)}`
                    : '';
                const guardText2 = guardResult
                    ? `，长窗HR=${Number.isFinite(guardResult.HR_bpm) ? guardResult.HR_bpm.toFixed(1) : '--'}, 长窗RR=${Number.isFinite(guardResult.RR_bpm) ? guardResult.RR_bpm.toFixed(1) : '--'}`
                    : '，长窗=等待数据';
                const motionText = `，运动=${motion.state}, 干净=${(motion.cleanFraction * 100).toFixed(0)}%, HR状态=${stable.status?.HR || '--'}, RR状态=${stable.status?.RR || '--'}`;
                const guardText = stable.messages.length ? `，${stable.messages.join('，')}` : '';
                runtimeEl.textContent = `${peakState}，${name}，${result.samples || iData.length}点，${this._formatVitalsFsDiagnostic(fs)}，epoch=${this.vitalsSamplingEpoch || 0}，${elapsed.toFixed(1)}ms，${rawText}${peakText}${confidenceText}${octaveText}${motionText}${guardText2}${guardText}`;
            }
            if (this.vitalsDiagnosticOpen) this._renderVitalsSpectrum(result, false);
            if (window.console && console.timeStamp) console.timeStamp(label);
            if (!this.vitalsFirstResultLogged && stable.hrValid && stable.rrValid) {
                this.vitalsFirstResultLogged = true;
                const algoName = this.vitalsMethod === 'cwt' ? 'CWT/ICWT' : '频域带通';
                this.addBLELog(`🫀 新算法已启动 (${algoName})：HR=${stable.stableHR.toFixed(1)} bpm, RR=${stable.stableRR.toFixed(1)} bpm，窗口=${iData.length}点，fs=${fs.toFixed(1)}Hz`);
            }
        } catch (e) {
            console.error('新版心率呼吸算法错误:', e);
            if (runtimeEl) runtimeEl.textContent = `算法错误: ${e.message}`;
        }
    }

    runVitalsFullDiagnostic() {
        if (!this.vitalsExtractor) this._initVitalsExtractor();
        if (!this.vitalsExtractor) {
            alert('新版算法未加载，请刷新页面后重试');
            return;
        }
        const available = Math.min(this.bleBufferI.length, this.bleBufferQ.length);
        if (available < 128) {
            alert('数据不足，至少需要 128 个 I/Q 点');
            return;
        }
        const fs = this._getVitalsFs();
        const iData = this.bleBufferI.slice(-available);
        const qData = this.bleBufferQ.slice(-available);
        const t0 = performance.now();
        const rrHintFreq = Number.isFinite(this.vitalsLastStableRR) ? this.vitalsLastStableRR / 60 : NaN;
        const hrHintFreq = Number.isFinite(this.vitalsLastStableHR) ? this.vitalsLastStableHR / 60 : NaN;
            const motion = this._analyzeVitalsMotion(iData.length, fs);
            const result = this.vitalsExtractor.run(iData, qData, fs, {
                rrHintFreq,
                hrHintFreq,
                rrHintMeta: this._buildVitalsHintMeta('RR'),
                hrHintMeta: this._buildVitalsHintMeta('HR'),
                motionQuality: motion.quality,
                motionCleanMask: motion.cleanMask
            });
            this._applyVitalsMotionEvidence(result, motion);
        const elapsed = performance.now() - t0;
        this.lastVitalsResult = result;
        this.vitalsDiagnosticOpen = true;
        this._renderVitalsSpectrum(result, true);
        const runtimeEl = document.getElementById('vitalsRuntime');
        if (runtimeEl) {
            runtimeEl.textContent = `完整诊断: HR=${Number.isFinite(result.HR_bpm) ? result.HR_bpm.toFixed(1) : '--'} bpm, RR=${Number.isFinite(result.RR_bpm) ? result.RR_bpm.toFixed(1) : '--'} bpm, ${elapsed.toFixed(1)}ms`;
        }
    }

    _renderVitalsSpectrum(result, forceShow = false) {
        if (!result || !result.spectrum || !result.spectrum.hr) return;
        const panel = document.getElementById('vitalsDiagnosticPanel');
        const canvas = document.getElementById('vitalsSpectrumChart');
        if (!canvas || typeof Chart === 'undefined') return;
        if (forceShow) this.vitalsDiagnosticOpen = true;
        if (panel && this.vitalsDiagnosticOpen) panel.style.display = 'block';
        if (!this.vitalsDiagnosticOpen) return;

        const hr = result.spectrum.hr;
        const rr = result.spectrum.rr;
        const labels = [];
        const raw = [];
        const robust = [];
        const harmonicMarks = [];
        const peakMarks = [];
        const edgeMarks = [];
        const edgeBins = (result.options && Number.isFinite(result.options.bandEdgeBins)) ? result.options.bandEdgeBins : 1;
        const df = hr.f.length > 1 ? (hr.f[1] - hr.f[0]) : 0;
        const safeLow = (result.options?.hrLow || 0.5) + edgeBins * df;
        const safeHigh = (result.options?.hrHigh || 3.0) - edgeBins * df;
        const peakFreq = result.HR_peak?.freq || result.HR_freq;
        const maxRobust = Math.max(...hr.P_robust.filter(v => Number.isFinite(v)), 0);
        for (let i = 0; i < hr.f.length; i++) {
            if (hr.f[i] < 0 || hr.f[i] > 3.2) continue;
            labels.push(hr.f[i].toFixed(3));
            raw.push(hr.P_raw[i]);
            robust.push(hr.P_robust[i]);
            let mark = null;
            if (Number.isFinite(result.RR_freq) && result.RR_freq > 0) {
                for (let k = 2; k <= 6; k++) {
                    const fH = k * result.RR_freq;
                    if (Math.abs(hr.f[i] - fH) <= (this._getVitalsFs() / (2 * hr.f.length))) {
                        mark = Math.max(hr.P_raw[i], hr.P_robust[i]);
                        break;
                    }
                }
            }
            harmonicMarks.push(mark);
            const isPeak = Number.isFinite(peakFreq) && Math.abs(hr.f[i] - peakFreq) <= Math.max(df, 1e-6);
            peakMarks.push(isPeak ? Math.max(hr.P_raw[i], hr.P_robust[i]) : null);
            const isEdge = Math.abs(hr.f[i] - safeLow) <= Math.max(df / 2, 1e-6)
                || Math.abs(hr.f[i] - safeHigh) <= Math.max(df / 2, 1e-6);
            edgeMarks.push(isEdge ? maxRobust : null);
        }

        const rrTopText = result.diagnostics?.rrPeaksTop?.slice(0, 3)
            .map(p => p.bpm.toFixed(1))
            .join('/') || '--';
        const hrTopText = result.diagnostics?.hrPeaksRobustTop?.slice(0, 3)
            .map(p => p.bpm.toFixed(1))
            .join('/') || '--';
        const rrUsedText = Number.isFinite(result.diagnostics?.rrUsedForHarmonic)
            ? (result.diagnostics.rrUsedForHarmonic * 60).toFixed(1)
            : '--';
        const title = `新版算法频谱 | HR=${Number.isFinite(result.HR_bpm) ? result.HR_bpm.toFixed(1) : '--'} bpm, RR=${Number.isFinite(result.RR_bpm) ? result.RR_bpm.toFixed(1) : '--'} bpm | RR候选=${rrTopText}, HR候选=${hrTopText}, 抑制RR=${rrUsedText}`;
        if (this.vitalsSpectrumChart) {
            const datasets = this.vitalsSpectrumChart.data.datasets;
            if (!datasets[3]) datasets[3] = { label: 'HR峰值(抛物线)', data: [], borderColor: '#22aa88', backgroundColor: '#22aa88', pointRadius: 5, showLine: false };
            if (!datasets[4]) datasets[4] = { label: 'HR带沿安全边界', data: [], borderColor: '#f0a500', backgroundColor: '#f0a500', pointRadius: 4, showLine: false };
            this.vitalsSpectrumChart.data.labels = labels;
            this.vitalsSpectrumChart.data.datasets[0].data = raw;
            this.vitalsSpectrumChart.data.datasets[1].data = robust;
            this.vitalsSpectrumChart.data.datasets[2].data = harmonicMarks;
            this.vitalsSpectrumChart.data.datasets[3].data = peakMarks;
            this.vitalsSpectrumChart.data.datasets[4].data = edgeMarks;
            this.vitalsSpectrumChart.options.plugins.title.text = title;
            this.vitalsSpectrumChart.update('none');
        } else {
            this.vitalsSpectrumChart = new Chart(canvas.getContext('2d'), {
                type: 'line',
                data: {
                    labels,
                    datasets: [
                        { label: 'HR原始谱', data: raw, borderColor: '#d85c6b', pointRadius: 0, tension: 0 },
                        { label: '谐波抑制后谱', data: robust, borderColor: '#5a9bd4', pointRadius: 0, tension: 0 },
                        { label: 'RR谐波位置', data: harmonicMarks, borderColor: '#222', backgroundColor: '#222', pointRadius: 3, showLine: false },
                        { label: 'HR峰值(抛物线)', data: peakMarks, borderColor: '#22aa88', backgroundColor: '#22aa88', pointRadius: 5, showLine: false },
                        { label: 'HR带沿安全边界', data: edgeMarks, borderColor: '#f0a500', backgroundColor: '#f0a500', pointRadius: 4, showLine: false }
                    ]
                },
                options: {
                    responsive: true,
                    animation: false,
                    plugins: {
                        title: { display: true, text: title }
                    },
                    scales: {
                        x: { title: { display: true, text: '频率 (Hz)' } },
                        y: { title: { display: true, text: '幅值' } }
                    }
                }
            });
        }
    }

    /**
     * 更新蓝牙生理参数（参考main.py的心率稳定算法）
     */
    updateBluetoothVitalSigns() {
        // The unified estimator owns both rates and waveforms. This renderer
        // must never run the legacy fixed-bin estimator or overwrite canonical
        // currentHeartRate/currentRespiratoryRate.
        const result = this.lastVitalsResult;
        if (!result?.waveforms) return;
        const phase = new Float64Array(result.waveforms.phase || []);
        const respiratoryWave = new Float64Array(result.waveforms.respiratory || []);
        const heartbeatWave = new Float64Array(result.waveforms.heartbeat || []);
        const dataLength = Math.min(respiratoryWave.length, heartbeatWave.length);
        if (!dataLength) return;

        try {
            // 更新呼吸/心跳波形图表
            const sampleSize = Math.min(1000, dataLength);
            const indices = Array.from({length: sampleSize}, (_, i) => i);

            if (this.bleCharts.respiratory) {
                this.bleCharts.respiratory.data = { labels: indices, datasets: [{ label: '呼吸波形(实时)', data: Array.from(respiratoryWave.slice(-sampleSize)), borderColor: 'rgb(75, 192, 192)', backgroundColor: 'rgba(75, 192, 192, 0.2)', tension: 0.1 }] };
                this.bleCharts.respiratory.update();  // 移除 'none' 让图表真正刷新
            }

            if (this.bleCharts.heartbeat) {
                this.bleCharts.heartbeat.data = { labels: indices, datasets: [{ label: '心跳波形(实时)', data: Array.from(heartbeatWave.slice(-sampleSize)), borderColor: 'rgb(255, 99, 132)', backgroundColor: 'rgba(255, 99, 132, 0.2)', tension: 0.1 }] };
                this.bleCharts.heartbeat.update();  // 移除 'none' 让图表真正刷新
            }

            // 推动ECG动态画布数据
            if (this._bleECG) {
                const resTrack = this._bleECG.res;
                const hbTrack = this._bleECG.hb;
                const pushLen = Math.min(50, respiratoryWave.length);
                const startIdx = Math.max(0, respiratoryWave.length - pushLen);
                // 归一化尾段，避免幅值漂移导致看不见
                const resSeg = Array.from(respiratoryWave.slice(startIdx));
                const hbSeg = Array.from(heartbeatWave.slice(Math.max(0, heartbeatWave.length - pushLen)));
                const norm = (arr) => {
                    if (arr.length === 0) return arr;
                    const mean = arr.reduce((a,b)=>a+b,0)/arr.length;
                    const std = Math.sqrt(arr.reduce((s,v)=>s+(v-mean)*(v-mean),0)/arr.length) || 1;
                    return arr.map(v => (v-mean)/(std*3)); // 压缩到[-~0.3,0.3]范围，便于显示
                };
                const resNorm = norm(resSeg);
                const hbNorm = norm(hbSeg);
                resNorm.forEach(v => resTrack.data.push(v));
                hbNorm.forEach(v => hbTrack.data.push(v));
                // 裁剪，避免无限增长
                if (resTrack.data.length > 5000) resTrack.data.splice(0, resTrack.data.length - 5000);
                if (hbTrack.data.length > 5000) hbTrack.data.splice(0, hbTrack.data.length - 5000);
                // 始终刷新一次画布，即使不在播放状态
                if (this._bleECG.draw) {
                    this._bleECG.draw();
                }
                // 如果在播放状态，继续动画循环
                if ((resTrack.playing || hbTrack.playing) && !this._bleECG.raf) {
                    this._bleECG.raf = requestAnimationFrame(this._bleECG.draw);
                }
            }

            // HR/RR publishing, tracking, recording and upload are handled by
            // _runVitalsCycle -> _publishVitalsResult above.
            return;
        } catch (e) {
            console.error('❌ 更新生理参数错误:', e);
            console.error('错误堆栈:', e.stack);
            console.error('当前状态:', {
                bufferLength: this.bleBufferI.length,
                processorExists: !!this.processor,
                extractVitalSignsMainPy: typeof this.processor?.extractVitalSignsMainPy
            });
            this.addBLELog(`❌ 处理错误: ${e.message}`);
        }
    }
}

// 全局函数供HTML调用
// 使用 var 让它同时挂到 window（方便调试，也避免部分环境下全局可见性差异）
var app = null;
window.app = null;
window.appInitError = null;

function _requireApp() {
    const a = window.app || app;
    if (!a) {
        const err = window.appInitError;
        if (err) {
            console.warn('应用初始化失败:', err);
        } else {
            console.warn('应用尚未初始化或初始化失败，请查看控制台错误日志');
        }
        const reason = err ? (err.message || String(err)) : '';
        alert(
            '应用尚未初始化或初始化失败（请查看控制台 Console 日志），建议刷新页面后重试。' +
            (reason ? `\n原因：${reason}` : '')
        );
        return null;
    }
    return a;
}

// 页面加载完成后初始化应用
document.addEventListener('DOMContentLoaded', () => {
    try {
        app = new RadarWebApp();
        window.app = app;
        window.appInitError = null;
    } catch (e) {
        console.error('❌ RadarWebApp 初始化失败:', e);
        app = null;
        window.app = null;
        window.appInitError = e;
    }
});

// 供HTML按钮调用的全局函数
function processFiles() {
    const a = _requireApp();
    if (a) a.processFiles();
}

function clearFiles() {
    const a = _requireApp();
    if (a) a.clearFiles();
}

function exportResults() {
    const a = _requireApp();
    if (a) a.exportResults();
}

function exportCharts() {
    const a = _requireApp();
    if (a) a.exportCharts();
}

function toggleSettings() {
    const a = _requireApp();
    if (a) a.toggleSettings();
}

function applySettings() {
    const a = _requireApp();
    if (a && typeof a.applySettings === 'function') a.applySettings();
}

// 连接诊断：生成诊断JSON并复制到剪贴板（方便你粘贴给我分析）
async function bleQuickDiagnose() {
    if (!app) return;
    const diag = app.buildBleDiagnostics ? app.buildBleDiagnostics() : { error: 'buildBleDiagnostics not available' };
    const text = JSON.stringify(diag, null, 2);
    try {
        if (navigator.clipboard && navigator.clipboard.writeText) {
            await navigator.clipboard.writeText(text);
            app.addBLELog('🩺 连接诊断已复制到剪贴板，请直接粘贴给我。');
        } else {
            // fallback
            prompt('复制下面的诊断信息发给我：', text);
        }
    } catch (e) {
        prompt('复制下面的诊断信息发给我：', text);
    }
    // 同时也打印到控制台（便于开发者工具查看）
    console.log('[BLE_DIAG]', diag);
}

// ===== Azure 配置/Prompt/RAG UI 逻辑 =====
function showAIConfig() {
    const modal = document.getElementById('aiConfigModal');
    if (!modal) return;
    // 预填本地保存的配置
    document.getElementById('azureEndpoint').value = localStorage.getItem('azureEndpoint') || '';
    document.getElementById('azureApiKey').value = localStorage.getItem('azureApiKey') || '';
    document.getElementById('azureDeployment').value = localStorage.getItem('azureDeployment') || 'gpt-4';
    modal.style.display = 'block';
}

function closeModal(id) {
    const modal = document.getElementById(id);
    if (modal) modal.style.display = 'none';
}

function saveAIConfig() {
    const endpoint = document.getElementById('azureEndpoint').value.trim();
    const apiKey = document.getElementById('azureApiKey').value.trim();
    const deployment = document.getElementById('azureDeployment').value.trim();
    if (!endpoint || !apiKey || !deployment) {
        alert('请完整填写 Endpoint / API Key / Deployment');
        return;
    }
    localStorage.setItem('azureEndpoint', endpoint);
    localStorage.setItem('azureApiKey', apiKey);
    localStorage.setItem('azureDeployment', deployment);
    alert('已保存 Azure OpenAI 配置');
    closeModal('aiConfigModal');
}

async function testAIConnection() {
    try {
        const endpoint = document.getElementById('azureEndpoint').value.trim();
        const apiKey = document.getElementById('azureApiKey').value.trim();
        const deployment = document.getElementById('azureDeployment').value.trim();
        if (!endpoint || !apiKey || !deployment) {
            alert('请先填写所有配置');
            return;
        }
        const analyzer = new AzureGPTAnalyzer();
        analyzer.configure(endpoint, apiKey, deployment);
        // 用极小提示测试
        const response = await analyzer.callAzureOpenAI('Test connection');
        alert('连接成功');
    } catch (e) {
        alert('连接失败: ' + e.message);
    }
}

function showPromptEditor() {
    const modal = document.getElementById('promptEditorModal');
    if (modal) modal.style.display = 'block';
}

function showRAGEditor() {
    const modal = document.getElementById('ragEditorModal');
    if (modal) modal.style.display = 'block';
}

// BLE 控制按钮回调
async function bleConnect() {
    if (!window.BLE) {
        app.showMessage('此浏览器不支持Web Bluetooth', 'error');
        return;
    }
    try {
        await BLE.connect();
    } catch (e) {
        app.showMessage(`连接失败: ${e.message}`, 'error');
    }
}

async function bleDisconnect() {
    if (!window.BLE) return;
    try {
        if (window.app) {
            if (Number.isFinite(app.lastLiveDeviceTsMs)) {
                app.pendingBackfillFromTsMs = app.lastLiveDeviceTsMs;
                app.allowBackfillOnNextConnect = true;
                app.addBLELog('🕘 已记录手动断开；下次连接将只识别断开期间缺失的补传数据');
            } else {
                app.pendingBackfillFromTsMs = null;
                app.allowBackfillOnNextConnect = false;
                app.addBLELog('ℹ️ 当前没有可用设备时间戳，下一次连接将按实时数据处理');
            }
        }
        await BLE.disconnect();
    } catch (e) {
        // ignore
    }
}

// 蓝牙录制控制函数 (参考main.py的按钮响应)
function toggleBluetoothRecording() {
    if (app && app.bleConnected) {
        app.toggleBluetoothRecording();
    } else {
        alert('请先连接蓝牙设备');
    }
}

// 分离的开始/结束录制按钮事件
function bleStartRecording() {
    if (!app || !app.bleConnected) {
        alert('请先连接蓝牙设备');
        return;
    }
    if (app.bleRecordingFlag !== 1) {
        app.toggleBluetoothRecording();
    }
}

function bleStopRecording() {
    if (!app || !app.bleConnected) {
        alert('请先连接蓝牙设备');
        return;
    }
    if (app.bleRecordingFlag === 1) {
        app.toggleBluetoothRecording();
    }
}

// 蓝牙上报控制
function bleStartUpload() {
    if (!app) return;
    app.startBleUpload();
}

function bleStopUpload() {
    if (!app) return;
    app.stopBleUpload();
}

// 蓝牙图表控制函数
function showBluetoothCharts() {
    const section = document.getElementById('bluetoothChartsSection');
    if (section) {
        section.style.display = 'block';
        section.scrollIntoView({ behavior: 'smooth' });
    }
    // 展开后强制刷新图表尺寸（避免之前隐藏导致的空白/不刷新）
    if (window.app && app.bleCharts) {
        setTimeout(() => {
            try {
                Object.values(app.bleCharts).forEach(ch => {
                    if (ch && typeof ch.resize === 'function') ch.resize();
                    if (ch && typeof ch.update === 'function') ch.update('none');
                });
            } catch (_) {}
        }, 50);
    }
}

function hideBluetoothCharts() {
    const section = document.getElementById('bluetoothChartsSection');
    if (section) {
        section.style.display = 'none';
    }
}

// 文件数据ECG播放控制
function toggleECGPlayback() {
    if (!app) return;

    // 初始化ECG播放器（如果还没有初始化）
    if (!app._fileECG) {
        app.initializeFileECG();
    }

    if (!app._fileECG) return;

    const playing = app._fileECG.res.playing || app._fileECG.hb.playing;
    const playBtn = document.getElementById('playBtn');
    const pauseBtn = document.getElementById('pauseBtn');

    if (playing) {
        // 暂停播放
        app._fileECG.res.playing = false;
        app._fileECG.hb.playing = false;
        pauseBtn.style.display = 'none';
        playBtn.style.display = 'inline-block';
    } else {
        // 开始播放
        app._fileECG.res.playing = true;
        app._fileECG.hb.playing = true;
        playBtn.style.display = 'none';
        pauseBtn.style.display = 'inline-block';
        if (!app._fileECG.raf) app._fileECG.draw();
    }
}

// BLE ECG 控制
function toggleBLEECGPlayback() {
    if (!app || !app._bleECG) return;
    const playing = app._bleECG.res.playing || app._bleECG.hb.playing;
    const playBtn = document.getElementById('blePlayBtn');
    const pauseBtn = document.getElementById('blePauseBtn');
    if (playing) {
        app._bleECG.res.playing = false;
        app._bleECG.hb.playing = false;
        pauseBtn.style.display = 'none';
        playBtn.style.display = 'inline-block';
    } else {
        app._bleECG.res.playing = true;
        app._bleECG.hb.playing = true;
        playBtn.style.display = 'none';
        pauseBtn.style.display = 'inline-block';
        if (!app._bleECG.raf) app._bleECG.draw();
    }
}

function resetECG() {
    if (!app || !app._fileECG) return;
    if (app._fileECG) {
        app._fileECG.res.cursor = 0;
        app._fileECG.hb.cursor = 0;
        app._fileECG.res.playing = false;
        app._fileECG.hb.playing = false;

        const playBtn = document.getElementById('playBtn');
        const pauseBtn = document.getElementById('pauseBtn');
        if (playBtn && pauseBtn) {
            pauseBtn.style.display = 'none';
            playBtn.style.display = 'inline-block';
        }
    }
}

function testECG() {
    if (!app) return;

    // 确保有处理结果
    if (app.processedResults.length === 0) {
        app.showMessage('请先上传并处理数据文件', 'warning');
        return;
    }

    // 初始化并测试ECG播放
    app.initializeFileECG();
    if (app._fileECG) {
        // 自动开始播放
        toggleECGPlayback();
        app.showMessage('ECG测试播放已启动', 'success');
    } else {
        app.showMessage('没有可播放的ECG数据', 'warning');
    }
}

function resetBLEECG() {
    if (!app || !app._bleECG) return;
    app._bleECG.res.data = [];
    app._bleECG.hb.data = [];
}

function clearBluetoothData() {
    if (app && confirm('确定要清空蓝牙数据吗？这将重置所有实时数据。')) {
        app.resetBluetoothData();
        app.addBLELog('🔄 已清空蓝牙数据');
    }
}

function clearBleBackfillData() {
    if (!app) return;
    app.bleBackfillBuffers = app._createEmptyBleBackfillBuffers();
    app.bleBackfillState = app._createBleBackfillState();
    app._bleBackfillLogLines = [];
    const logEl = document.getElementById('bleBackfillLog');
    if (logEl) logEl.textContent = '';
    app.updateBleBackfillUI();
    Object.values(app.bleBackfillCharts || {}).forEach(chart => {
        if (chart && chart.data) {
            chart.data.labels = [];
            chart.data.datasets = [];
            if (typeof chart.update === 'function') chart.update('none');
        }
    });
    app.addBLELog('🧹 已清空补传面板数据');
}

function saveBluetoothData() {
    if (!app || !app.bleConnected) {
        alert('请先连接蓝牙设备');
        return;
    }
    
    if (app.bleBufferI.length === 0) {
        alert('没有可保存的数据');
        return;
    }

    // 生成文件内容
    let content = '';
    for (let i = 0; i < app.bleBufferI.length; i++) {
        const ts = app.bleBufferTimestamps[i] || `${Date.now()}-${i}`;
        content += `${ts}\t${app.bleBufferI[i]}\t${app.bleBufferQ[i]}\n`;
    }
    
    // 下载文件
    const timestamp = new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19);
    const filename = `bluetooth_data_${timestamp}.txt`;
    app.downloadFile(content, filename, 'text/plain');
    app.addBLELog(`💾 已保存数据: ${filename} (${app.bleBufferI.length} 数据点)`);
}

// 模拟测试功能
function startSimulationTest() {
    if (!app) return;
    
    app.addBLELog('🧪 开始模拟数据测试...');
    
    // 模拟蓝牙连接
    app.bleConnected = true;
    app.bleConnectStartTime = Date.now();
    app.startBluetoothTimer();
    
    // 显示实时数据区域
    document.getElementById('bleRealTimeData').style.display = 'block';

    // 自动展开图表区域并刷新（确保图表可见且尺寸正确）
    const chartsSection = document.getElementById('bluetoothChartsSection');
    if (chartsSection) {
        chartsSection.style.display = 'block';
        chartsSection.scrollIntoView({ behavior: 'smooth' });
        console.log('✅ 模拟测试：蓝牙图表区域已展开');
    }

    // 确保图表已初始化
    if (!app.bleCharts.iSignal || !app.bleCharts.qSignal) {
        console.log('🔄 模拟测试：重新初始化蓝牙图表...');
        app.initializeBluetoothCharts();
    }

    // 延迟触发布局更新，确保Canvas尺寸正确
    setTimeout(() => {
        if (app.bleCharts) {
            console.log('📊 模拟测试：刷新所有蓝牙图表...');
            Object.values(app.bleCharts).forEach(chart => {
                if (chart && typeof chart.resize === 'function') chart.resize();
                if (chart && typeof chart.update === 'function') chart.update('none');
            });
        }
    }, 200);

    app.updateBLEButtons();
    if (typeof app.startVitalsRealtime === 'function') {
        app.startVitalsRealtime();
    }
    
    // 生成模拟数据 (模拟心率75bpm，呼吸18bpm)
    // Browser background tabs may throttle a nominal 20 ms setInterval to
    // 100--1000 ms. Generate every sample that is due from a monotonic clock
    // on each callback so the self-test remains a true 50 Hz *frame* stream,
    // even when several frames are packed into one simulated notification.
    const simulationFs = 50;
    const simulationStartMonoMs = performance.now();
    const simulationDeviceStartMs = new Date(2026, 0, 1, 12, 0, 0, 0).getTime();
    let simulationNotificationId = 0;
    let dataCount = 0;
    app.stopSimulation();
    if (app.processor) app.processor.fs = simulationFs;
    const compactDeviceTimestamp = sampleIndex => {
        const date = new Date(simulationDeviceStartMs + sampleIndex * 1000 / simulationFs);
        const pad = (value, width = 2) => String(value).padStart(width, '0');
        return `${pad(date.getFullYear() % 100)}${pad(date.getMonth() + 1)}${pad(date.getDate())}`
            + `${pad(date.getHours())}${pad(date.getMinutes())}${pad(date.getSeconds())}${pad(date.getMilliseconds(), 3)}`;
    };
    const emitSimulationSample = (notificationMeta) => {
        const t = dataCount / simulationFs;
        // One deterministic 1-second collar movement verifies that the UI
        // holds the reliable 75/18 baseline, rejects the movement frequency,
        // and returns to fresh output after the clean tail arrives.
        const motionBurst = t >= 18 && t < 19;
        if (dataCount === Math.round(18 * simulationFs)) app.addBLELog('🐾 连续监测自检：开始1秒项圈运动，数值应保持而不是消失');
        if (dataCount === Math.round(19 * simulationFs)) app.addBLELog('↻ 连续监测自检：运动停止，等待快速恢复实时状态');
        // 相位调制正交 I/Q：比把两个正弦直接叠加到 I、Q 更接近 CW
        // 雷达胸壁位移模型，也能验证圆拟合和相位解调的完整链路。
        const radarPhase = 0.70
            + 0.32 * Math.sin(2 * Math.PI * 0.3 * t + 0.20)
            + 0.11 * Math.sin(2 * Math.PI * 1.25 * t + 1.10)
            + (motionBurst ? 1.20 * Math.sin(2 * Math.PI * 2.2 * t + 0.4) : 0)
            + 0.002 * (Math.random() - 0.5);
        const voltageI = 1.65 + 0.12 * Math.cos(radarPhase);
        const voltageQ = 1.55 + 0.105 * Math.sin(radarPhase + 3 * Math.PI / 180);
        
        // 将电压转换为ADC值（-32768~32767）
        // 反向公式：adc = (voltage * 2 / 3.3 - 1) * 32767
        const adcI = Math.round((voltageI * 2 / 3.3 - 1) * 32767);
        const adcQ = Math.round((voltageQ * 2 / 3.3 - 1) * 32767);
        
        // 模拟 V1.02：加速度(g)、陀螺仪(deg/s)，与固件实际输出一致
        const burstWave = motionBurst ? Math.sin(2 * Math.PI * 4.3 * t) : 0;
        const ax_g = voltageI * 0.01 + 0.30 * burstWave;
        const ay_g = voltageQ * 0.005 + 0.22 * (motionBurst ? Math.cos(2 * Math.PI * 3.7 * t) : 0);
        const az_g = 1.0 + 0.01 * (Math.random() - 0.5) + 0.18 * burstWave;
        const gx_dps = 5.0 * Math.sin(2 * Math.PI * 0.5 * t) + 70 * burstWave;
        const gy_dps = 2.5 * Math.cos(2 * Math.PI * 0.2 * t) + 55 * (motionBurst ? Math.cos(2 * Math.PI * 3.7 * t) : 0);
        const gz_dps = 1.0 * Math.sin(2 * Math.PI * 1.0 * t) + 40 * burstWave;
        const roll = 5 * Math.sin(2 * Math.PI * 0.1 * t);
        const pitch = 3 * Math.cos(2 * Math.PI * 0.08 * t);
        const yaw = -45 + 10 * Math.sin(2 * Math.PI * 0.02 * t);
        const gpsTimeBase = compactDeviceTimestamp(dataCount);
        const v2 = voltageI;
        const v3 = voltageQ;
        const simulatedLine = `$SIMTEST,${gpsTimeBase},E114.2010058,N22.3787343,${ax_g.toFixed(5)},${ay_g.toFixed(5)},${az_g.toFixed(5)},${gx_dps.toFixed(5)},${gy_dps.toFixed(5)},${gz_dps.toFixed(5)},${roll.toFixed(2)},${pitch.toFixed(2)},${yaw.toFixed(2)},3.300,${v2.toFixed(3)},${v3.toFixed(3)}*`;
        app.handleBLELine(simulatedLine, notificationMeta);
        
        dataCount++;
    };
    app._simInterval = setInterval(() => {
        if (dataCount >= 2000) {
            app.stopSimulation();
            app.addBLELog('🏁 模拟测试完成');
            return;
        }
        const dueSamples = Math.min(
            2000,
            Math.floor((performance.now() - simulationStartMonoMs) * simulationFs / 1000)
        );
        const framesThisNotification = Math.max(0, dueSamples - dataCount);
        if (!framesThisNotification) return;
        const notificationId = ++simulationNotificationId;
        const arrivalMonoMs = performance.now();
        const arrivalMs = Date.now();
        for (let frameIndex = 0; frameIndex < framesThisNotification && dataCount < 2000; frameIndex++) {
            emitSimulationSample({ notificationId, arrivalMonoMs, arrivalMs, frameIndex });
        }
    }, 50);

    app.addBLELog(`📡 正在生成模拟心率75bpm、呼吸18bpm的数据（${simulationFs}Hz帧率），第18秒含1秒运动连续性自检...`);
}

// 停止模拟
RadarWebApp.prototype.stopSimulation = function() {
    if (this._simInterval) {
        clearInterval(this._simInterval);
        this._simInterval = null;
    }
};

// 触发Azure诊断：基于本次录制窗口统计
async function bleAzureDiagnose() {
    if (!window.AzureGPTAnalyzer) {
        alert('Azure模块未加载');
        return;
    }
    if (!app || !app._bleWindowHistory || app._bleWindowHistory.length === 0) {
        alert('暂无可用的录制窗口统计，请先完成一次录制');
        return;
    }

    try {
        const analyzer = new AzureGPTAnalyzer();
        // 读取页面配置（如果已在右侧设置面板中配置，则可扩展从localStorage读取）
        const endpoint = localStorage.getItem('azureEndpoint') || '';
        const apiKey = localStorage.getItem('azureApiKey') || '';
        const deployment = localStorage.getItem('azureDeployment') || 'gpt-4';
        analyzer.configure(endpoint, apiKey, deployment);

        // 将本次录制窗口统计转换为 processedResults 结构的最小集合
        const session = app._buildBluetoothSessionStats();
        const processedResults = [
            {
                status: 'success',
                heartRate: session.average.heartRate,
                respiratoryRate: session.average.respiratoryRate,
                heartRateTimeSeries: session.windows.map(w => w.heartRate),
                respiratoryRateTimeSeries: session.windows.map(w => w.respiratoryRate),
                timeAxis: session.windows.map((w, i) => i),
                dataPoints: app.bleBufferI.length,
                fileName: 'bluetooth_session'
            }
        ];

        const result = await analyzer.generateDiagnosticReport(processedResults, 'detailed_medical');
        if (result.success) {
            const ts = new Date().toISOString().replace(/[:.]/g, '-').slice(0,19);
            app.downloadFile(result.report, `bluetooth_session_report_${ts}.txt`, 'text/plain');
            app.addBLELog('🤖 已生成并下载AI诊断报告');
        } else {
            alert('生成诊断失败: ' + result.error);
        }
    } catch (e) {
        alert('AI诊断出错: ' + e.message);
    }
}

// 宠物健康分析相关全局函数
function performHealthAnalysis() {
    const a = _requireApp();
    if (a) a.performHealthAnalysis();
}

function exportHealthReport() {
    const a = _requireApp();
    if (a) a.exportHealthReport();
}

// ===== 活动量与步数监测模块控制函数 =====

/**
 * 开始活动监测
 */
function startActivityMonitor() {
    if (!app) {
        alert('应用未初始化');
        return;
    }

    console.log('🎯 开始活动监测...');

    // 先显示仪表板，确保DOM元素存在
    document.getElementById('activityDashboard').style.display = 'block';

    // 更新按钮状态
    document.getElementById('activityStartBtn').style.display = 'none';
    document.getElementById('activityStopBtn').style.display = 'inline-block';
    document.getElementById('activityResetBtn').style.display = 'inline-block';

    // 延迟初始化，确保DOM已渲染
    setTimeout(() => {
        // 初始化ActivityMonitor
        if (!app.activityMonitor) {
            console.log('📊 创建ActivityMonitor实例...');

            // 从输入框读取体重和目标值
            const petWeight = parseFloat(document.getElementById('petWeight').value) || 10.0;
            const stepsGoal = parseInt(document.getElementById('activityStepsGoal').value) || 1000;
            const activityGoal = parseFloat(document.getElementById('activityENMOGoal').value) || 2.0;
            const calorieGoal = parseFloat(document.getElementById('activityCalorieGoal').value) || 100;

            app.activityMonitor = new ActivityMonitor(app.processor.fs, petWeight);
            app.activityMonitor.dailyGoal = stepsGoal;
            app.activityMonitor.activityGoal = activityGoal;
            app.activityMonitor.calorieGoal = calorieGoal;

            updateActivityGoalDisplay();

            // 初始化图表（在DOM渲染后）
            console.log('🎨 初始化图表...');
            app.activityMonitor.initializeCharts();
        }

        app.activityMonitorEnabled = true;
        activityLog('✅ 活动监测已启动');
    }, 100); // 100ms延迟确保DOM已渲染
}

/**
 * 停止活动监测
 */
function stopActivityMonitor() {
    if (!app) return;

    app.activityMonitorEnabled = false;

    // 更新按钮状态
    document.getElementById('activityStartBtn').style.display = 'inline-block';
    document.getElementById('activityStopBtn').style.display = 'none';

    activityLog('⏹️ 活动监测已停止');
}

/**
 * 重置今日数据
 */
function resetActivityData() {
    if (!app || !app.activityMonitor) return;

    if (confirm('确定要重置今日的活动数据吗？')) {
        app.activityMonitor.resetDailyData();
        activityLog('🔄 今日数据已重置');
    }
}

/**
 * 更新活动目标
 */
function updateActivityGoals() {
    if (!app || !app.activityMonitor) return;

    const petWeight = parseFloat(document.getElementById('petWeight').value) || 10.0;
    const stepsGoal = parseInt(document.getElementById('activityStepsGoal').value) || 1000;
    const activityGoal = parseFloat(document.getElementById('activityENMOGoal').value) || 2.0;
    const calorieGoal = parseFloat(document.getElementById('activityCalorieGoal').value) || 100;

    // 更新体重会重新计算RER
    app.activityMonitor.petWeight = petWeight;
    app.activityMonitor.rerDaily = 70 * Math.pow(petWeight, 0.75);
    app.activityMonitor.bmrPerSec = app.activityMonitor.rerDaily / 86400.0;

    app.activityMonitor.dailyGoal = stepsGoal;
    app.activityMonitor.activityGoal = activityGoal;
    app.activityMonitor.calorieGoal = calorieGoal;
    app.activityMonitor.updateCharts();

    updateActivityGoalDisplay();
    activityLog(`🎯 目标已更新: 体重=${petWeight}kg, 步数=${stepsGoal}, 活动量=${activityGoal.toFixed(1)}, 卡路里=${calorieGoal}kcal`);
}

/**
 * 更新目标显示
 */
function updateActivityGoalDisplay() {
    document.getElementById('displayStepsGoal').textContent =
        document.getElementById('activityStepsGoal').value;
    document.getElementById('displayActivityGoal').textContent =
        document.getElementById('activityENMOGoal').value;
    document.getElementById('displayCalorieGoal').textContent =
        document.getElementById('activityCalorieGoal').value;
}

/**
 * 活动日志输出
 */
function activityLog(message) {
    const logDiv = document.getElementById('activityLog');
    if (!logDiv) return;

    const timestamp = new Date().toLocaleTimeString();
    logDiv.innerHTML += `[${timestamp}] ${message}<br>`;
    logDiv.scrollTop = logDiv.scrollHeight;
}

// ===== 睡眠监测控制函数 =====

/**
 * 开始睡眠监测
 */
function startSleepMonitor() {
    if (!app) {
        alert('应用未初始化');
        return;
    }

    console.log('😴 开始睡眠监测...');
    console.log('📊 蓝牙连接状态:', app.bleConnected);
    console.log('📊 活动监测状态:', app.activityMonitorEnabled);

    // 更新按钮状态
    document.getElementById('sleepStartBtn').style.display = 'none';
    document.getElementById('sleepStopBtn').style.display = 'inline-block';

    // 延迟初始化，确保DOM已渲染
    setTimeout(() => {
        // 初始化SleepMonitor
        if (!app.sleepMonitor) {
            console.log('📊 创建SleepMonitor实例...');

            // 从输入框读取体重（如果有的话，否则使用默认值）
            const petWeight = parseFloat(document.getElementById('petWeight')?.value) || 10.0;

            app.sleepMonitor = new SleepMonitor(app.processor.fs, petWeight);

            // 初始化图表（在DOM渲染后）
            console.log('🎨 初始化睡眠监测图表...');
            app.sleepMonitor.initializeCharts();
        }

        app.sleepMonitorEnabled = true;
        console.log('✅ 睡眠监测已启用，等待数据...');
        sleepLog('✅ 睡眠监测已启动');

        // 启动定时更新
        startSleepMonitorUpdates();
    }, 100); // 100ms延迟确保DOM已渲染
}

/**
 * 停止睡眠监测
 */
function stopSleepMonitor() {
    if (!app) return;

    app.sleepMonitorEnabled = false;

    // 更新按钮状态
    document.getElementById('sleepStartBtn').style.display = 'inline-block';
    document.getElementById('sleepStopBtn').style.display = 'none';

    // 停止定时更新
    if (app.sleepMonitorUpdateTimer) {
        clearInterval(app.sleepMonitorUpdateTimer);
        app.sleepMonitorUpdateTimer = null;
    }

    sleepLog('⏹️ 睡眠监测已停止');
}

/**
 * 重置睡眠数据
 */
function resetSleepData() {
    if (!app || !app.sleepMonitor) return;

    if (confirm('确定要重置睡眠数据吗？')) {
        app.sleepMonitor.reset();
        updateSleepDisplay();
        sleepLog('🔄 睡眠数据已重置');
    }
}

/**
 * 生成睡眠报告
 */
function generateSleepReport() {
    if (!app || !app.sleepMonitor) {
        alert('请先启动睡眠监测');
        return;
    }

    const report = app.sleepMonitor.generateSleepReport();

    // 创建报告弹窗
    const reportHTML = `
        <div style="text-align: left; max-height: 500px; overflow-y: auto;">
            <h3>😴 睡眠质量报告</h3>

            <div style="background: #f5f5f5; padding: 15px; border-radius: 8px; margin: 10px 0;">
                <h4>📊 总体评分</h4>
                <div style="font-size: 24px; color: ${report.qualityScore >= 80 ? '#34C759' : report.qualityScore >= 60 ? '#FF9500' : '#FF3B30'};">
                    ${report.qualityScore}分 - ${report.qualityLevel}
                </div>
            </div>

            <div style="background: #f5f5f5; padding: 15px; border-radius: 8px; margin: 10px 0;">
                <h4>⏱️ 睡眠时长</h4>
                <p>总睡眠时间: ${(report.totalSleepTime / 60).toFixed(1)}分钟</p>
                <p>深睡: ${(report.deepSleepTime / 60).toFixed(1)}分钟 (${report.deepSleepRatio.toFixed(1)}%)</p>
                <p>浅睡: ${(report.lightSleepTime / 60).toFixed(1)}分钟</p>
                <p>REM: ${(report.remSleepTime / 60).toFixed(1)}分钟 (${report.remSleepRatio.toFixed(1)}%)</p>
                <p>清醒: ${(report.awakeTime / 60).toFixed(1)}分钟</p>
            </div>

            <div style="background: #f5f5f5; padding: 15px; border-radius: 8px; margin: 10px 0;">
                <h4>💤 睡眠质量</h4>
                <p>睡眠效率: ${report.sleepEfficiency.toFixed(1)}%</p>
                <p>翻身次数: ${report.turnOverCount}次</p>
                <p>翻身频率: ${report.turnOverRate.toFixed(1)}次/小时</p>
            </div>

            <div style="background: #f5f5f5; padding: 15px; border-radius: 8px; margin: 10px 0;">
                <h4>💡 建议</h4>
                ${report.recommendations.map(r => `<p>• ${r}</p>`).join('')}
            </div>
        </div>
    `;

    // 显示报告
    const reportWindow = window.open('', '睡眠报告', 'width=600,height=700');
    reportWindow.document.write(`
        <!DOCTYPE html>
        <html>
        <head>
            <title>睡眠质量报告</title>
            <meta charset="UTF-8">
            <style>
                body { font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; padding: 20px; }
                h3 { color: #333; }
                h4 { color: #666; margin-top: 0; }
            </style>
        </head>
        <body>
            ${reportHTML}
            <button onclick="window.print()" style="margin-top: 20px; padding: 10px 20px; background: #007AFF; color: white; border: none; border-radius: 8px; cursor: pointer;">
                🖨️ 打印报告
            </button>
        </body>
        </html>
    `);
}

/**
 * 启动睡眠监测定时更新
 */
function startSleepMonitorUpdates() {
    // 每2秒更新一次显示
    if (app.sleepMonitorUpdateTimer) {
        clearInterval(app.sleepMonitorUpdateTimer);
    }

    app.sleepMonitorUpdateTimer = setInterval(() => {
        if (app.sleepMonitorEnabled && app.sleepMonitor) {
            updateSleepDisplay();
            app.sleepMonitor.updateCharts();
        }
    }, 2000);
}

/**
 * 更新睡眠显示
 */
function updateSleepDisplay() {
    if (!app || !app.sleepMonitor) {
        console.log('⚠️ 睡眠监测器未初始化');
        return;
    }

    const summary = app.sleepMonitor.getSummary();
    console.log('📊 更新睡眠显示:', {
        currentRMS: summary.currentRMS,
        currentMCR: summary.currentMCR,
        stage: summary.currentStageName,
        dataRate: summary.dataRate
    });

    // 更新当前状态
    const stageElement = document.getElementById('sleepCurrentStage');
    if (stageElement) {
        stageElement.textContent = summary.currentStageName;
    } else {
        console.error('❌ 找不到元素: sleepCurrentStage');
    }

    const bedStatusElement = document.getElementById('sleepInBedStatus');
    if (bedStatusElement) {
        bedStatusElement.textContent = summary.isInBed ? '在床上' : '未在床上';
    }

    // 更新睡眠时长
    const sleepMinutes = (summary.sleepDuration / 60).toFixed(0);
    const durationElement = document.getElementById('sleepDuration');
    if (durationElement) {
        durationElement.textContent = `${sleepMinutes}分钟`;
    }

    if (summary.sleepStartTime) {
        const startTime = new Date(summary.sleepStartTime).toLocaleTimeString();
        const startTimeElement = document.getElementById('sleepStartTime');
        if (startTimeElement) {
            startTimeElement.textContent = `开始于 ${startTime}`;
        }
    } else {
        const startTimeElement = document.getElementById('sleepStartTime');
        if (startTimeElement) {
            startTimeElement.textContent = '未开始';
        }
    }

    // 更新睡眠效率
    const efficiencyElement = document.getElementById('sleepEfficiency');
    if (efficiencyElement) {
        efficiencyElement.textContent = `${summary.sleepEfficiency.toFixed(1)}%`;
    }

    // 更新翻身次数
    const turnOverElement = document.getElementById('sleepTurnOverCount');
    if (turnOverElement) {
        turnOverElement.textContent = `${summary.turnOverCount}次`;
    }

    // 更新睡眠统计
    const deepTimeElement = document.getElementById('sleepDeepTime');
    if (deepTimeElement) {
        deepTimeElement.textContent = `${(summary.deepSleepTime / 60).toFixed(0)}分钟`;
    }

    const lightTimeElement = document.getElementById('sleepLightTime');
    if (lightTimeElement) {
        lightTimeElement.textContent = `${(summary.lightSleepTime / 60).toFixed(0)}分钟`;
    }

    const remTimeElement = document.getElementById('sleepREMTime');
    if (remTimeElement) {
        remTimeElement.textContent = `${(summary.remSleepTime / 60).toFixed(0)}分钟`;
    }

    const awakeTimeElement = document.getElementById('sleepAwakeTime');
    if (awakeTimeElement) {
        awakeTimeElement.textContent = `${(summary.awakeTime / 60).toFixed(0)}分钟`;
    }

    // 更新实时关键指标
    // RMS能量
    const rmsElement = document.getElementById('sleepCurrentRMS');
    if (rmsElement) {
        rmsElement.textContent = `${summary.currentRMS.toFixed(3)}g`;
        console.log('✅ 更新RMS:', summary.currentRMS.toFixed(3));
    } else {
        console.error('❌ 找不到元素: sleepCurrentRMS');
    }

    const rmsStatusElement = document.getElementById('sleepRMSStatus');
    if (rmsStatusElement) {
        let rmsStatus = '静息';
        if (summary.currentRMS > 0.15) rmsStatus = '剧烈活动';
        else if (summary.currentRMS > 0.05) rmsStatus = '活动中';
        else if (summary.currentRMS > 0.02) rmsStatus = '轻微活动';
        rmsStatusElement.textContent = rmsStatus;
    }

    // MCR零交叉率
    const mcrElement = document.getElementById('sleepCurrentMCR');
    if (mcrElement) {
        mcrElement.textContent = `${summary.currentMCR.toFixed(1)}次/秒`;
        console.log('✅ 更新MCR:', summary.currentMCR.toFixed(1));
    } else {
        console.error('❌ 找不到元素: sleepCurrentMCR');
    }

    const mcrStatusElement = document.getElementById('sleepMCRStatus');
    if (mcrStatusElement) {
        let mcrStatus = '低频';
        if (summary.currentMCR > 8) mcrStatus = '高频';
        else if (summary.currentMCR > 5) mcrStatus = '中高频';
        else if (summary.currentMCR > 2) mcrStatus = '中频';
        mcrStatusElement.textContent = mcrStatus;
    }

    // 阶段持续时间
    const stageDurationMin = (summary.stageDuration / 60).toFixed(1);
    const stageDurationElement = document.getElementById('sleepStageDuration');
    if (stageDurationElement) {
        stageDurationElement.textContent = `${stageDurationMin}分钟`;
    } else {
        console.error('❌ 找不到元素: sleepStageDuration');
    }

    const lastStageChangeElement = document.getElementById('sleepLastStageChange');
    if (lastStageChangeElement) {
        if (summary.stageDuration < 60) {
            lastStageChangeElement.textContent = '刚切换';
        } else {
            lastStageChangeElement.textContent = `持续${stageDurationMin}分钟`;
        }
    }

    // 数据更新率
    const dataRateElement = document.getElementById('sleepDataRate');
    if (dataRateElement) {
        dataRateElement.textContent = `${summary.dataRate.toFixed(1)} Hz`;
        console.log('✅ 更新数据率:', summary.dataRate.toFixed(1));
    } else {
        console.error('❌ 找不到元素: sleepDataRate');
    }

    const lastUpdateElement = document.getElementById('sleepLastUpdate');
    if (lastUpdateElement) {
        if (summary.lastUpdateTime) {
            const timeSinceUpdate = (Date.now() - summary.lastUpdateTime) / 1000;
            if (timeSinceUpdate < 2) {
                lastUpdateElement.textContent = '实时更新中';
            } else {
                lastUpdateElement.textContent = `${timeSinceUpdate.toFixed(0)}秒前`;
            }
        } else {
            lastUpdateElement.textContent = '等待数据';
        }
    }
}

/**
 * 睡眠日志输出
 */
function sleepLog(message) {
    const logDiv = document.getElementById('sleepEventLog');
    if (!logDiv) return;

    const timestamp = new Date().toLocaleTimeString();
    logDiv.innerHTML += `[${timestamp}] ${message}<br>`;
    logDiv.scrollTop = logDiv.scrollHeight;
}

// 页面加载时显示活动监测按钮
window.addEventListener('DOMContentLoaded', () => {
    // 当蓝牙连接成功后，显示活动监测开始按钮
    const observer = new MutationObserver(() => {
        const bleDisconnectBtn = document.getElementById('bleDisconnectBtn');
        if (bleDisconnectBtn && bleDisconnectBtn.style.display !== 'none') {
            // 蓝牙已连接，显示活动监测按钮
            const activityStartBtn = document.getElementById('activityStartBtn');
            if (activityStartBtn) {
                activityStartBtn.style.display = 'inline-block';
            }

            // 显示睡眠监测按钮
            const sleepStartBtn = document.getElementById('sleepStartBtn');
            if (sleepStartBtn) {
                sleepStartBtn.style.display = 'inline-block';
            }
        }
    });

    const bleDisconnectBtn = document.getElementById('bleDisconnectBtn');
    if (bleDisconnectBtn) {
        observer.observe(bleDisconnectBtn, { attributes: true, attributeFilter: ['style'] });
    }
});

// ===== 宠物情绪监测控制函数 =====

/**
 * 开始宠物情绪监测（HR / RR / ENMO / HRV 实时图）
 */
function startPetEmotionMonitor() {
    if (!app) { alert('应用未初始化，请先连接蓝牙'); return; }

    // 懒初始化
    if (!app.petEmotionMonitor) {
        if (typeof PetEmotionMonitor === 'undefined') {
            alert('PetEmotionMonitor 模块未加载，请检查 pet-emotion-monitor.js');
            return;
        }
        app.petEmotionMonitor = new PetEmotionMonitor();
    }

    // 同步物种选择
    const speciesEl = document.getElementById('petEmotionSpecies');
    if (speciesEl) app.petEmotionMonitor.species = speciesEl.value;

    // 延迟初始化图表（确保 DOM 已渲染）
    setTimeout(() => {
        app.petEmotionMonitor.start();
        app.petEmotionMonitorEnabled = true;
        console.log('🐾 宠物情绪监测已启动');
    }, 100);
}

/**
 * 停止宠物情绪监测
 */
function stopPetEmotionMonitor() {
    if (!app) return;
    if (app.petEmotionMonitor) app.petEmotionMonitor.stop();
    app.petEmotionMonitorEnabled = false;
    console.log('🐾 宠物情绪监测已停止');
}

// ===== GPS 定位追踪控制函数 =====

function startGPSLocation() {
    if (!app) { alert('应用未初始化，请先连接蓝牙'); return; }

    if (!app.gpsLocationMonitor) {
        if (typeof GPSLocationMonitor === 'undefined') {
            alert('GPSLocationMonitor 模块未加载，请检查 gps-location.js');
            return;
        }
        app.gpsLocationMonitor = new GPSLocationMonitor();
    }

    document.getElementById('gpsDashboard').style.display = 'block';
    document.getElementById('gpsStartBtn').style.display = 'none';
    document.getElementById('gpsStopBtn').style.display = 'inline-block';

    setTimeout(() => {
        app.gpsLocationMonitor.start();
        app.gpsLocationMonitorEnabled = true;
        console.log('📍 GPS 定位追踪已启动');
    }, 100);
}

function stopGPSLocation() {
    if (!app) return;
    if (app.gpsLocationMonitor) app.gpsLocationMonitor.stop();
    app.gpsLocationMonitorEnabled = false;

    document.getElementById('gpsStartBtn').style.display = 'inline-block';
    document.getElementById('gpsStopBtn').style.display = 'none';
    console.log('📍 GPS 定位追踪已停止');
}

function resetGPSLocation() {
    if (!app || !app.gpsLocationMonitor) return;
    if (confirm('确定要重置GPS轨迹数据吗？')) {
        app.gpsLocationMonitor.reset();
        console.log('🔄 GPS 轨迹数据已重置');
    }
}

// 导出睡眠监测相关函数到全局作用域
// 确保这些函数可以被HTML的onclick属性调用
if (typeof window !== 'undefined') {
    window.startGPSLocation = startGPSLocation;
    window.stopGPSLocation = stopGPSLocation;
    window.resetGPSLocation = resetGPSLocation;
    window.startPetEmotionMonitor = startPetEmotionMonitor;
    window.stopPetEmotionMonitor  = stopPetEmotionMonitor;
    window.startSleepMonitor = startSleepMonitor;
    window.stopSleepMonitor = stopSleepMonitor;
    window.resetSleepData = resetSleepData;
    window.generateSleepReport = generateSleepReport;
    window.updateSleepDisplay = updateSleepDisplay;
    window.sleepLog = sleepLog;
    
    console.log('✅ 睡眠监测函数已导出到全局作用域');
    console.log('验证: startSleepMonitor =', typeof window.startSleepMonitor);
    console.log('验证: stopSleepMonitor =', typeof window.stopSleepMonitor);
    console.log('验证: resetSleepData =', typeof window.resetSleepData);
}

// ===== 姿态解算功能 =====

/**
 * 切换姿态解算功能
 */
function toggleAttitude() {
    const app = window.app;
    if (!app) return;

    if (!app.attitudeEnabled) {
        // 启用姿态解算
        app.attitudeEnabled = true;

        // 创建姿态解算器
        if (!app.attitudeSolver) {
            app.attitudeSolver = new AttitudeSolver();
            app.attitudeSolver.setSampleRate(100); // 假设 100Hz 采样率
        }

        // 先显示姿态数据区域（让容器有尺寸），再创建 3D 可视化器
        document.getElementById('attitudeDisplay').style.display = 'block';

        // 延迟创建可视化器，等待浏览器完成布局
        setTimeout(() => {
            if (!app.attitudeVisualizer) {
                app.attitudeVisualizer = new AttitudeVisualizer('attitude3DContainer');
            } else {
                // 已存在则重启动画并适配新尺寸
                app.attitudeVisualizer.onWindowResize();
                app.attitudeVisualizer.start();
            }
        }, 100);

        // 更新按钮文本
        document.getElementById('attitudeEnableBtn').textContent = '停止姿态解算';
        document.getElementById('attitudeEnableBtn').classList.remove('btn-primary');
        document.getElementById('attitudeEnableBtn').classList.add('btn-danger');

        console.log('✅ 姿态解算已启用');
    } else {
        // 停止姿态解算
        app.attitudeEnabled = false;

        // 停止可视化
        if (app.attitudeVisualizer) {
            app.attitudeVisualizer.stop();
        }

        // 隐藏显示区域
        document.getElementById('attitudeDisplay').style.display = 'none';

        // 更新按钮文本
        document.getElementById('attitudeEnableBtn').textContent = '启用姿态解算';
        document.getElementById('attitudeEnableBtn').classList.remove('btn-danger');
        document.getElementById('attitudeEnableBtn').classList.add('btn-primary');

        console.log('⏸️ 姿态解算已停止');
    }
}

/**
 * 切换姿态解算算法
 */
function changeAttitudeAlgorithm() {
    const app = window.app;
    if (!app || !app.attitudeSolver) return;

    const algorithm = document.getElementById('attitudeAlgorithm').value;
    app.attitudeSolver.setAlgorithm(algorithm);

    // 重置姿态
    app.attitudeSolver.reset();

    console.log(`🔄 姿态解算算法已切换为: ${algorithm}`);
}

/**
 * 更新姿态显示
 */
function updateAttitudeDisplay() {
    const app = window.app;
    if (!app || !app.attitudeEnabled || !app.attitudeSolver) return;

    // 获取欧拉角
    const euler = app.attitudeSolver.getEulerAngles();
    document.getElementById('attitudePitch').textContent = euler.pitch.toFixed(1) + '°';
    document.getElementById('attitudeRoll').textContent = euler.roll.toFixed(1) + '°';
    document.getElementById('attitudeYaw').textContent = euler.yaw.toFixed(1) + '°';

    // 获取四元数
    const quat = app.attitudeSolver.getQuaternion();
    document.getElementById('attitudeQW').textContent = quat.w.toFixed(3);
    document.getElementById('attitudeQX').textContent = quat.x.toFixed(3);
    document.getElementById('attitudeQY').textContent = quat.y.toFixed(3);
    document.getElementById('attitudeQZ').textContent = quat.z.toFixed(3);

    // 获取角速度
    const angVel = app.attitudeSolver.getAngularVelocity();
    document.getElementById('attitudeGx').textContent = angVel.gx.toFixed(1);
    document.getElementById('attitudeGy').textContent = angVel.gy.toFixed(1);
    document.getElementById('attitudeGz').textContent = angVel.gz.toFixed(1);

    // 动物姿态判断（使用最新的加速度计数据）
    const lastAx = app.bleBufferACC_X[app.bleBufferACC_X.length - 1] || 0;
    const lastAy = app.bleBufferACC_Y[app.bleBufferACC_Y.length - 1] || 0;
    const lastAz = app.bleBufferACC_Z[app.bleBufferACC_Z.length - 1] || 1;

    // 检查是否有有效的加速度计数据
    if (lastAx === 0 && lastAy === 0 && lastAz === 1) {
        // 这是默认值，说明没有接收到真实的加速度计数据
        if (!app._noAccDataWarningShown) {
            console.warn('⚠️ updateAttitudeDisplay: 没有接收到有效的加速度计数据');
            console.warn('bleBufferACC_X.length:', app.bleBufferACC_X ? app.bleBufferACC_X.length : 0);
            console.warn('请确保蓝牙设备发送的数据包含 Acc: 字段');
            app._noAccDataWarningShown = true;
        }
        return;
    }

    const posture = app.attitudeSolver.classifyPosture(lastAx, lastAy, lastAz);

    const labelEl = document.getElementById('animalPostureLabel');
    const confEl  = document.getElementById('animalPostureConf');
    const accEl   = document.getElementById('animalAccMag');
    const gyrEl   = document.getElementById('animalGyrMag');
    if (labelEl) labelEl.textContent = posture.label;
    if (confEl)  confEl.textContent  = (posture.confidence * 100).toFixed(0) + '%';
    if (accEl)   accEl.textContent   = posture.accMag + ' g';
    if (gyrEl)   gyrEl.textContent   = posture.gyrMag + ' °/s';

    // 获取用户手动标注的状态
    const manualPosture = app.manualPosture || '';

    // 调试：每100次输出一次状态
    if (!app._attitudeDisplayCount) app._attitudeDisplayCount = 0;
    app._attitudeDisplayCount++;
    if (app._attitudeDisplayCount % 100 === 1) {
        console.log(`📊 [第${app._attitudeDisplayCount}次] 姿态显示状态:`, {
            postureRecordingFlag: app.postureRecordingFlag,
            manualPosture: app.manualPosture,
            manualPostureType: typeof app.manualPosture,
            manualPostureLength: app.manualPosture ? app.manualPosture.length : 0,
            postureDataLogLength: app.postureDataLog ? app.postureDataLog.length : 0
        });
    }

    // 如果正在记录姿态数据，保存带标注的数据
    if (app.postureRecordingFlag === 1 && manualPosture) {
        savePostureData(euler, quat, angVel, posture, lastAx, lastAy, lastAz, manualPosture);
        updatePostureDataDisplay();
    } else if (app.postureRecordingFlag === 1 && !manualPosture) {
        // 调试：提示用户需要选择标注状态
        if (!app._postureWarningShown) {
            console.warn('⚠️ 正在记录姿态数据但未选择手动标注状态');
            console.warn('当前 app.manualPosture =', app.manualPosture);
            console.warn('当前 app.manualPosture 类型 =', typeof app.manualPosture);
            console.warn('当前 app.postureRecordingFlag =', app.postureRecordingFlag);
            app._postureWarningShown = true;
        }
    }

    // 更新 3D 可视化
    if (app.attitudeVisualizer) {
        app.attitudeVisualizer.updateQuaternion(quat.w, quat.x, quat.y, quat.z);
    }
}

// 导出姿态解算函数到全局作用域
if (typeof window !== 'undefined') {
    window.toggleAttitude = toggleAttitude;
    window.changeAttitudeAlgorithm = changeAttitudeAlgorithm;
    window.updateAttitudeDisplay = updateAttitudeDisplay;

    console.log('✅ 姿态解算函数已导出到全局作用域');
}

/**
 * 保存姿态数据（用于训练和验证）
 */
function savePostureData(euler, quat, angVel, posture, ax, ay, az, manualLabel) {
    const app = window.app;
    if (!app.postureDataLog) {
        app.postureDataLog = [];
        console.log('🐾 初始化姿态数据日志');
    }

    const timestamp = Date.now();
    const dataPoint = {
        // 时间信息
        timestamp: timestamp,
        time: new Date(timestamp).toISOString(),

        // 标注信息
        manualLabel: manualLabel,  // 用户手动标注的真实状态
        systemLabel: posture.label.replace(/[🏃🚶🐾😴🐕🌿]/g, '').trim(),  // 系统自动判断
        confidence: parseFloat((posture.confidence * 100).toFixed(1)),  // 置信度(%)

        // 欧拉角（最直观的姿态表示）
        pitch: parseFloat(euler.pitch.toFixed(2)),
        roll: parseFloat(euler.roll.toFixed(2)),
        yaw: parseFloat(euler.yaw.toFixed(2)),

        // 四元数（用于姿态重建）
        qw: parseFloat(quat.w.toFixed(4)),
        qx: parseFloat(quat.x.toFixed(4)),
        qy: parseFloat(quat.y.toFixed(4)),
        qz: parseFloat(quat.z.toFixed(4)),

        // 角速度（原始数据）
        gx: parseFloat(angVel.gx.toFixed(2)),
        gy: parseFloat(angVel.gy.toFixed(2)),
        gz: parseFloat(angVel.gz.toFixed(2)),
        gyrMag: parseFloat(posture.gyrMag),  // 角速度幅值（已经是字符串）

        // 加速度（原始数据）
        ax: parseFloat(ax.toFixed(4)),
        ay: parseFloat(ay.toFixed(4)),
        az: parseFloat(az.toFixed(4)),
        accMag: parseFloat(posture.accMag)  // 加速度幅值（已经是字符串）
    };

    app.postureDataLog.push(dataPoint);

    // 每10条记录输出一次日志
    if (app.postureDataLog.length % 10 === 0) {
        console.log(`🐾 已记录 ${app.postureDataLog.length} 条姿态数据`);
    }

    // 前3条记录输出详细信息用于调试
    if (app.postureDataLog.length <= 3) {
        console.log(`✅ 第 ${app.postureDataLog.length} 条数据已保存:`, {
            manualLabel: manualLabel,
            pitch: euler.pitch.toFixed(1),
            roll: euler.roll.toFixed(1),
            yaw: euler.yaw.toFixed(1)
        });
    }
}

/**
 * 更新姿态数据显示
 */
function updatePostureDataDisplay() {
    const app = window.app;
    if (!app.postureDataLog || app.postureDataLog.length === 0) return;

    const countEl = document.getElementById('postureRecordCount');
    const previewEl = document.getElementById('postureDataPreview');

    if (!countEl || !previewEl) return;

    // 更新记录数量
    countEl.textContent = app.postureDataLog.length;

    // 显示最近5条记录
    const recentData = app.postureDataLog.slice(-5).reverse();
    let html = '';
    for (const data of recentData) {
        html += `<div style="margin-bottom: 8px; padding: 6px; background: #1a1a1a; border-left: 3px solid #2196F3; border-radius: 3px;">`;
        html += `<div style="color: #64B5F6; font-weight: bold;">标注: ${data.manualLabel} | 判断: ${data.systemLabel}</div>`;
        html += `<div style="color: #888; font-size: 0.9em; margin-top: 3px;">`;
        html += `pitch=${data.pitch}° roll=${data.roll}° yaw=${data.yaw}° | `;
        html += `acc=${data.accMag}g gyr=${data.gyrMag}°/s | `;
        html += `conf=${data.confidence}%`;
        html += `</div></div>`;
    }
    previewEl.innerHTML = html;
}

/**
 * 导出当前姿态数据
 */
function exportCurrentPostureData() {
    const app = window.app;
    if (!app.postureDataLog || app.postureDataLog.length === 0) {
        alert('没有可导出的姿态数据！请先选择手动标注状态并开始记录。');
        return;
    }

    const timestamp = new Date().toISOString().slice(0, 16).replace('T', '-').replace(/:/g, '-');
    const filename = `posture_data_${timestamp}.json`;

    const exportData = {
        exportTime: new Date().toISOString(),
        totalRecords: app.postureDataLog.length,
        note: '用于姿态判断算法矫正的训练数据',
        data: app.postureDataLog
    };

    const json = JSON.stringify(exportData, null, 2);
    app.downloadFile(json, filename, 'application/json');

    console.log(`🐾 已导出姿态数据: ${filename} (${app.postureDataLog.length} 条记录)`);
}

/**
 * 诊断姿态记录功能
 */
function diagnosePostureRecording() {
    const app = window.app;
    console.log('\n========== 🔍 姿态记录诊断 ==========');

    // 1. 检查基本状态
    console.log('\n1️⃣ 基本状态检查:');
    console.log('  ✓ app 对象:', app ? '存在' : '❌ 不存在');
    console.log('  ✓ attitudeEnabled:', app.attitudeEnabled);
    console.log('  ✓ attitudeSolver:', app.attitudeSolver ? '存在' : '❌ 不存在');

    // 2. 检查手动标注
    console.log('\n2️⃣ 手动标注状态:');
    console.log('  ✓ manualPosture 值:', `"${app.manualPosture}"`);
    console.log('  ✓ manualPosture 类型:', typeof app.manualPosture);
    console.log('  ✓ manualPosture 长度:', app.manualPosture ? app.manualPosture.length : 0);
    console.log('  ✓ manualPosture 是否为空:', !app.manualPosture ? '❌ 是（这会导致无法保存数据！）' : '✅ 否');

    // 3. 检查记录标志
    console.log('\n3️⃣ 记录标志:');
    console.log('  ✓ postureRecordingFlag:', app.postureRecordingFlag);
    console.log('  ✓ postureRecordingFlag === 1:', app.postureRecordingFlag === 1 ? '✅ 是' : '❌ 否');

    // 4. 检查数据日志
    console.log('\n4️⃣ 数据日志:');
    console.log('  ✓ postureDataLog 存在:', app.postureDataLog ? '✅ 是' : '❌ 否');
    console.log('  ✓ postureDataLog 长度:', app.postureDataLog ? app.postureDataLog.length : 'N/A');

    // 5. 检查保存条件
    console.log('\n5️⃣ 保存条件检查:');
    const manualPosture = app.manualPosture || '';
    const condition1 = app.postureRecordingFlag === 1;
    const condition2 = !!manualPosture;
    const canSave = condition1 && condition2;

    console.log('  ✓ 条件1 (postureRecordingFlag === 1):', condition1 ? '✅ 满足' : '❌ 不满足');
    console.log('  ✓ 条件2 (manualPosture 不为空):', condition2 ? '✅ 满足' : '❌ 不满足');
    console.log('  ✓ 最终结果 (可以保存数据):', canSave ? '✅ 是' : '❌ 否');

    // 6. 检查蓝牙数据
    console.log('\n6️⃣ 蓝牙数据检查:');
    console.log('  ✓ bleConnected:', app.bleConnected ? '✅ 已连接' : '❌ 未连接');
    if (app.bleBufferACC_X && app.bleBufferACC_X.length > 0) {
        const lastAx = app.bleBufferACC_X[app.bleBufferACC_X.length - 1];
        const lastAy = app.bleBufferACC_Y[app.bleBufferACC_Y.length - 1];
        const lastAz = app.bleBufferACC_Z[app.bleBufferACC_Z.length - 1];
        console.log('  ✓ 加速度计数据:', `X=${lastAx?.toFixed(3)}, Y=${lastAy?.toFixed(3)}, Z=${lastAz?.toFixed(3)}`);
        console.log('  ✓ 数据有效性:', (lastAx !== 0 || lastAy !== 0 || lastAz !== 1) ? '✅ 有效' : '❌ 无效（默认值）');
    } else {
        console.log('  ✓ 加速度计数据: ❌ 无数据');
    }

    // 7. 给出诊断结论
    console.log('\n========== 📋 诊断结论 ==========');
    if (!canSave) {
        console.log('❌ 无法保存数据！');
        console.log('\n原因分析:');
        if (!condition1) {
            console.log('  • 记录标志未启用，请点击"开始记录姿态"按钮');
        }
        if (!condition2) {
            console.log('  • 手动标注状态为空，请在下拉菜单中选择当前真实姿态');
        }
    } else {
        console.log('✅ 所有条件满足，应该可以正常保存数据');
        if (app.postureDataLog && app.postureDataLog.length === 0) {
            console.log('\n⚠️ 但是数据日志为空，可能的原因:');
            console.log('  • updateAttitudeDisplay() 函数未被调用');
            console.log('  • 蓝牙数据未正常接收');
            console.log('  • 加速度计数据无效');
        }
    }
    console.log('=====================================\n');

    // 弹出简化的诊断结果
    let message = '诊断结果:\n\n';
    message += `姿态解算: ${app.attitudeEnabled ? '✅ 已启用' : '❌ 未启用'}\n`;
    message += `手动标注: ${app.manualPosture ? `✅ ${app.manualPosture}` : '❌ 未选择'}\n`;
    message += `记录状态: ${app.postureRecordingFlag === 1 ? '✅ 记录中' : '❌ 未开始'}\n`;
    message += `数据记录: ${app.postureDataLog ? app.postureDataLog.length : 0} 条\n`;
    message += `\n可以保存: ${canSave ? '✅ 是' : '❌ 否'}\n`;

    if (!canSave) {
        message += '\n请检查:\n';
        if (!condition1) message += '• 点击"开始记录姿态"\n';
        if (!condition2) message += '• 选择手动标注状态\n';
    }

    alert(message);
}

// 导出到全局作用域
if (typeof window !== 'undefined') {
    window.exportCurrentPostureData = exportCurrentPostureData;
    window.togglePostureRecording = togglePostureRecording;
    window.diagnosePostureRecording = diagnosePostureRecording;
}

/**
 * 初始化手动标注功能
 */
function initManualPostureLabeling() {
    const app = window.app;
    const selectEl = document.getElementById('manualPostureInput');
    const displayEl = document.getElementById('manualPostureDisplay');

    console.log('🔧 初始化手动标注功能', {
        selectEl: selectEl ? '存在' : '不存在',
        displayEl: displayEl ? '存在' : '不存在'
    });

    if (!selectEl || !displayEl) {
        console.error('❌ 手动标注元素未找到！');
        return;
    }

    // 初始化标注状态（只在未初始化时设置）
    if (app.manualPosture === undefined) {
        app.manualPosture = '';
        console.log('✅ 初始化 app.manualPosture = ""');
    }
    if (!app.postureDataLog) {
        app.postureDataLog = [];
        console.log('✅ 初始化 app.postureDataLog = []');
    }
    if (app.postureRecordingFlag === undefined) {
        app.postureRecordingFlag = 0; // 独立的姿态记录标志
        console.log('✅ 初始化 app.postureRecordingFlag = 0');
    }
    if (!app.postureRecordingStartTime) {
        app.postureRecordingStartTime = null;
        console.log('✅ 初始化 app.postureRecordingStartTime = null');
    }

    // 监听下拉菜单变化
    selectEl.addEventListener('change', function() {
        const selected = this.value;
        app.manualPosture = selected;

        console.log(`✏️ 手动标注状态已更改:`, {
            selected: selected,
            selectedType: typeof selected,
            selectedLength: selected ? selected.length : 0,
            appManualPosture: app.manualPosture,
            postureRecordingFlag: app.postureRecordingFlag
        });

        // 特殊处理：初始佩戴静止状态 - 触发姿态校准
        if (selected === '初始佩戴静止') {
            if (app.attitudeSolver) {
                app.attitudeSolver.reset();
                console.log('⚙️ 姿态解算器已重置，开始校准...');

                // 显示校准提示
                displayEl.textContent = '⚙️ 校准中...';
                displayEl.style.color = '#2196F3';

                // 3秒后完成校准
                setTimeout(() => {
                    displayEl.textContent = '✅ 校准完成';
                    displayEl.style.color = '#4CAF50';
                    console.log('✅ 姿态校准完成');

                    // 提示用户可以选择真实姿态
                    setTimeout(() => {
                        alert('✅ 3D姿态平面校准完成！\n\n现在请选择动物的真实姿态状态。');
                        selectEl.value = '';
                        app.manualPosture = '';
                        displayEl.textContent = '未标注';
                        displayEl.style.color = '#999';
                    }, 1000);
                }, 3000);
            } else {
                console.error('❌ 姿态解算器未初始化');
                alert('❌ 姿态解算器未启用，请先启用IMU姿态解算功能');
            }
            return;
        }

        if (selected) {
            displayEl.textContent = selected;
            displayEl.style.color = '#FFB74D';

            // 如果正在记录，提示用户
            if (app.postureRecordingFlag === 1) {
                console.log(`✏️ 已切换标注状态为: ${selected}`);
            }
        } else {
            displayEl.textContent = '未标注';
            displayEl.style.color = '#999';
        }
    });

    // 初始化显示区域
    const countEl = document.getElementById('postureRecordCount');
    if (countEl) countEl.textContent = '0';
}

/**
 * 切换姿态数据记录
 */
function togglePostureRecording() {
    const app = window.app;

    // 检查是否启用了姿态解算
    if (!app.attitudeEnabled || !app.attitudeSolver) {
        alert('请先启用IMU姿态解算！');
        return;
    }

    // 检查是否选择了手动标注
    if (!app.manualPosture) {
        alert('请先在下拉菜单中选择当前真实姿态！');
        return;
    }

    // 切换记录状态
    app.postureRecordingFlag = (1 + app.postureRecordingFlag) % 2;

    const btn = document.getElementById('postureRecordStartBtn');

    if (app.postureRecordingFlag === 1) {
        // 开始记录
        app.postureDataLog = [];
        app.postureRecordingStartTime = new Date();
        app._postureWarningShown = false;

        // 清空显示
        const countEl = document.getElementById('postureRecordCount');
        const previewEl = document.getElementById('postureDataPreview');
        if (countEl) countEl.textContent = '0';
        if (previewEl) previewEl.innerHTML = '<div style="color: #888;">正在记录数据...</div>';

        // 更新按钮
        btn.textContent = '⏹️ 停止记录';
        btn.style.background = '#f44336';

        console.log(`🔴 开始记录姿态数据 - 标注: ${app.manualPosture}`);
        console.log(`📊 初始化状态: postureDataLog.length = ${app.postureDataLog.length}`);
        console.log(`📊 记录标志: postureRecordingFlag = ${app.postureRecordingFlag}`);
        console.log(`📊 姿态解算启用: attitudeEnabled = ${app.attitudeEnabled}`);
        console.log(`📊 姿态解算器存在: attitudeSolver = ${app.attitudeSolver ? '是' : '否'}`);


    } else {
        // 停止记录并导出
        const duration = ((new Date() - app.postureRecordingStartTime) / 1000).toFixed(1);

        console.log(`⏹️ 停止记录 - 检查数据状态:`);
        console.log(`  postureDataLog 存在: ${app.postureDataLog ? '是' : '否'}`);
        console.log(`  postureDataLog 长度: ${app.postureDataLog ? app.postureDataLog.length : 'N/A'}`);
        console.log(`  记录时长: ${duration} 秒`);

        if (app.postureDataLog && app.postureDataLog.length > 0) {
            // 自动导出数据
            const timestamp = app.postureRecordingStartTime.toISOString()
                .slice(0, 16).replace('T', '-').replace(/:/g, '-');
            const filename = `posture_data_${timestamp}.json`;

            const exportData = {
                recordingStartTime: app.postureRecordingStartTime.toISOString(),
                recordingEndTime: new Date().toISOString(),
                durationSeconds: parseFloat(duration),
                totalRecords: app.postureDataLog.length,
                note: '用于姿态判断算法矫正的训练数据',
                data: app.postureDataLog
            };

            const json = JSON.stringify(exportData, null, 2);
            app.downloadFile(json, filename, 'application/json');

            console.log(`🟢 停止记录 - 时长: ${duration}秒, 记录: ${app.postureDataLog.length} 条`);
            console.log(`📄 已导出: ${filename}`);
        } else {
            console.warn('⚠️ 没有记录到任何姿态数据');
            console.warn('可能的原因:');
            console.warn('  1. 未选择手动标注状态 (app.manualPosture =', app.manualPosture, ')');
            console.warn('  2. 姿态解算未启用 (app.attitudeEnabled =', app.attitudeEnabled, ')');
            console.warn('  3. 蓝牙数据未正常接收');
            alert('没有记录到任何姿态数据！\n\n请确保：\n1. 已选择手动标注状态\n2. 姿态解算已启用\n3. 蓝牙设备正常连接并发送数据');
        }

        // 更新按钮
        btn.textContent = '🔴 开始记录姿态';
        btn.style.background = '#4CAF50';

        app.postureRecordingStartTime = null;
    }
}

// 在页面加载时初始化
document.addEventListener('DOMContentLoaded', function() {
    initManualPostureLabeling();
});

/**
 * 调整姿态解算灵敏度
 */
function adjustAttitudeSensitivity() {
    const app = window.app;
    if (!app || !app.attitudeSolver) return;

    const beta = parseFloat(document.getElementById('attitudeBeta').value);
    document.getElementById('attitudeBetaValue').textContent = beta.toFixed(1);

    app.attitudeSolver.beta = beta;
    console.log(`🎚️ 姿态解算灵敏度已调整为: ${beta}`);
}

// 导出灵敏度调整函数
if (typeof window !== 'undefined') {
    window.adjustAttitudeSensitivity = adjustAttitudeSensitivity;
}

// ===== 蓝牙自动重连功能 =====

/**
 * 重连到保存的蓝牙设备
 */
async function bleReconnect() {
    if (!window.BLE) {
        app.showMessage('此浏览器不支持Web Bluetooth', 'error');
        return;
    }

    const savedInfo = BLE.getSavedDeviceInfo();
    if (!savedInfo) {
        app.showMessage('没有保存的设备', 'warning');
        return;
    }

    try {
        app.showMessage(`正在重连到: ${savedInfo.name}...`, 'info');
        await BLE.reconnectToSaved();
        app.showMessage(`已重连到: ${savedInfo.name}`, 'success');
    } catch (e) {
        app.showMessage(`重连失败: ${e.message}`, 'error');
        console.error('重连失败:', e);
    }
}

/**
 * 清除保存的蓝牙设备
 */
function bleClearSaved() {
    if (!window.BLE) return;
    
    BLE.clearSavedDevice();
    updateBLESavedDeviceUI();
    app.showMessage('已清除保存的设备', 'success');
}

/**
 * 更新保存设备的UI显示
 */
function updateBLESavedDeviceUI() {
    const savedInfo = window.BLE ? BLE.getSavedDeviceInfo() : null;
    const savedDeviceEl = document.getElementById('bleSavedDevice');
    const reconnectBtn = document.getElementById('bleReconnectBtn');
    const clearSavedBtn = document.getElementById('bleClearSavedBtn');

    if (savedDeviceEl) {
        if (savedInfo) {
            const savedDate = new Date(savedInfo.savedAt);
            savedDeviceEl.innerHTML = `
                <strong>已保存设备:</strong> ${savedInfo.name}<br>
                <small>保存时间: ${savedDate.toLocaleString()}</small>
            `;
            savedDeviceEl.style.display = 'block';
        } else {
            savedDeviceEl.style.display = 'none';
        }
    }

    if (reconnectBtn) {
        reconnectBtn.style.display = savedInfo ? 'inline-block' : 'none';
    }

    if (clearSavedBtn) {
        clearSavedBtn.style.display = savedInfo ? 'inline-block' : 'none';
    }
}

// 页面加载时更新UI
document.addEventListener('DOMContentLoaded', () => {
    updateBLESavedDeviceUI();
});

// 导出函数到全局作用域
if (typeof window !== 'undefined') {
    window.bleReconnect = bleReconnect;
    window.bleClearSaved = bleClearSaved;
    window.updateBLESavedDeviceUI = updateBLESavedDeviceUI;

    console.log('✅ 蓝牙自动重连功能已加载');
}
