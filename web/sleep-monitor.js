/**
 * 睡眠质量监测模块 (Sleep Quality Monitor Module)
 * 基于IMU加速度计数据，实现宠物睡眠状态检测和分析
 *
 * 核心功能:
 * 1. 零交叉率(MCR)计算 - 检测体动频率
 * 2. RMS能量计算 - 检测运动强度
 * 3. 睡眠阶段分类 - 清醒/浅睡/深睡/REM
 * 4. 睡眠事件检测 - 入睡/醒来/翻身
 * 5. 睡眠质量报告生成
 */

class SleepMonitor {
    constructor(samplingRate = 50, petWeight = 10.0) {
        this.fs = samplingRate; // 采样率 (Hz)
        this.petWeight = petWeight; // 宠物体重 (kg)

        // 窗口配置
        this.windowSize = 300;  // 6秒窗口（300样本@50Hz）
        this.slideStep = 50;    // 1秒滑动步长

        // 睡眠检测阈值（可根据宠物调整）
        this.thresholds = {
            inBedRMS: 0.05,        // 在床判定阈值 (g)
            deepSleepRMS: 0.02,    // 深睡阈值 (g)
            lightSleepRMS: 0.03,   // 浅睡阈值 (g)
            awakeMCR: 5,           // 清醒MCR阈值 (次/秒)
            deepSleepMCR: 2,       // 深睡MCR阈值 (次/秒)
            minSleepDuration: 600, // 最小睡眠时长 (秒)
            minStageDuration: 60,  // 最小阶段持续时长 (秒)
            remRMSMin: 0.02,       // REM最小RMS (g)
            remRMSMax: 0.05,       // REM最大RMS (g)
            remMCRMin: 3,          // REM最小MCR (次/秒)
            remMCRMax: 8,          // REM最大MCR (次/秒)
            turnOverRMS: 0.15      // 翻身阈值 (g)
        };

        // 数据缓冲区
        this.axBuffer = [];
        this.ayBuffer = [];
        this.azBuffer = [];
        this.timestampBuffer = [];

        // 睡眠状态
        this.currentStage = 'awake';  // awake, light, deep, rem
        this.isInBed = false;
        this.isSleeping = false;
        this.sleepStartTime = null;
        this.lastStageChangeTime = null;

        // 实时指标
        this.currentRMS = 0;
        this.currentMCR = 0;
        this.lastUpdateTime = null;
        this.dataUpdateCount = 0;
        this.dataUpdateStartTime = Date.now();

        // 睡眠历史记录
        this.sleepHistory = [];  // {timestamp, stage, rms, mcr, duration}
        this.sleepEvents = [];   // {timestamp, event, details}
        this.stageHistory = [];  // {stage, startTime, endTime, duration}

        // 统计数据
        this.totalSleepTime = 0;      // 总睡眠时间 (秒)
        this.deepSleepTime = 0;       // 深睡时间 (秒)
        this.lightSleepTime = 0;      // 浅睡时间 (秒)
        this.remSleepTime = 0;        // REM睡眠时间 (秒)
        this.awakeTime = 0;           // 清醒时间 (秒)
        this.sleepEfficiency = 0;     // 睡眠效率 (%)
        this.turnOverCount = 0;       // 翻身次数

        // 图表对象
        this.charts = {
            sleepStageTimeline: null,
            rmsChart: null,
            sleepCycleChart: null
        };

        console.log('✅ 睡眠监测模块初始化完成');
    }

    /**
     * 计算零交叉率 (Mean Crossing Rate)
     * @param {Array} signal - 信号数组
     * @returns {number} MCR值 (次/秒)
     */
    calculateMCR(signal) {
        if (signal.length < 2) return 0;

        // 计算信号均值
        const mean = signal.reduce((sum, val) => sum + val, 0) / signal.length;

        // 计算零交叉次数
        let crossings = 0;
        for (let i = 1; i < signal.length; i++) {
            if ((signal[i - 1] - mean) * (signal[i] - mean) < 0) {
                crossings++;
            }
        }

        // 转换为每秒交叉次数
        const duration = signal.length / this.fs;
        return crossings / duration;
    }

    /**
     * 计算RMS能量
     * @param {Array} ax - X轴加速度数组
     * @param {Array} ay - Y轴加速度数组
     * @param {Array} az - Z轴加速度数组
     * @returns {number} RMS值 (g)
     */
    calculateRMS(ax, ay, az) {
        if (ax.length === 0) return 0;

        let sumSquares = 0;
        for (let i = 0; i < ax.length; i++) {
            sumSquares += ax[i] * ax[i] + ay[i] * ay[i] + az[i] * az[i];
        }

        return Math.sqrt(sumSquares / ax.length);
    }

    /**
     * 分类睡眠阶段
     * @param {number} rms - RMS能量值
     * @param {number} mcr - MCR值
     * @returns {string} 睡眠阶段: 'awake', 'light', 'deep', 'rem'
     */
    classifySleepStage(rms, mcr) {
        // 判断是否在床上
        if (rms > this.thresholds.inBedRMS) {
            return 'awake';
        }

        // 深睡眠: 低RMS + 低MCR
        if (rms < this.thresholds.deepSleepRMS && mcr < this.thresholds.deepSleepMCR) {
            return 'deep';
        }

        // REM睡眠: 中等RMS + 中高MCR
        if (rms >= this.thresholds.remRMSMin && rms <= this.thresholds.remRMSMax &&
            mcr >= this.thresholds.remMCRMin && mcr <= this.thresholds.remMCRMax) {
            return 'rem';
        }

        // 浅睡眠: 低RMS + 中等MCR
        if (rms < this.thresholds.lightSleepRMS) {
            return 'light';
        }

        // 默认为清醒
        return 'awake';
    }

    /**
     * 添加加速度计数据
     * @param {number} ax - X轴加速度 (g)
     * @param {number} ay - Y轴加速度 (g)
     * @param {number} az - Z轴加速度 (g)
     * @param {number} timestamp - 时间戳 (ms)
     */
    addAccelerometerData(ax, ay, az, timestamp) {
        // 首次接收数据时输出日志
        if (this.axBuffer.length === 0) {
            console.log(`😴 [睡眠监测] 开始接收数据: ax=${ax.toFixed(3)}, ay=${ay.toFixed(3)}, az=${az.toFixed(3)}`);
        }

        this.axBuffer.push(ax);
        this.ayBuffer.push(ay);
        this.azBuffer.push(az);
        this.timestampBuffer.push(timestamp);

        // 当缓冲区达到窗口大小时，处理数据
        if (this.axBuffer.length >= this.windowSize) {
            this.processAccelerometerData();

            // 滑动窗口：移除旧数据
            const removeCount = this.slideStep;
            this.axBuffer.splice(0, removeCount);
            this.ayBuffer.splice(0, removeCount);
            this.azBuffer.splice(0, removeCount);
            this.timestampBuffer.splice(0, removeCount);
        }
    }

    /**
     * 处理加速度计数据窗口
     */
    processAccelerometerData() {
        // 获取当前时间戳
        const currentTime = this.timestampBuffer[this.timestampBuffer.length - 1];

        // 计算RMS能量
        const rms = this.calculateRMS(this.axBuffer, this.ayBuffer, this.azBuffer);

        // 计算加速度幅值用于MCR计算
        const magnitude = [];
        for (let i = 0; i < this.axBuffer.length; i++) {
            magnitude.push(Math.sqrt(
                this.axBuffer[i] ** 2 +
                this.ayBuffer[i] ** 2 +
                this.azBuffer[i] ** 2
            ));
        }

        // 计算MCR
        const mcr = this.calculateMCR(magnitude);

        // 更新实时指标
        this.currentRMS = rms;
        this.currentMCR = mcr;
        this.lastUpdateTime = currentTime;
        this.dataUpdateCount++;

        // 每10次更新输出一次日志
        if (this.dataUpdateCount % 10 === 0) {
            console.log(`😴 [睡眠监测] RMS=${rms.toFixed(3)}g, MCR=${mcr.toFixed(1)}次/秒, 阶段=${this.currentStage}, 数据点=${this.dataUpdateCount}`);
        }

        // 分类睡眠阶段
        const newStage = this.classifySleepStage(rms, mcr);

        // 检测睡眠事件
        this.detectSleepEvents(newStage, rms, currentTime);

        // 更新当前阶段
        if (newStage !== this.currentStage) {
            this.updateSleepStage(newStage, currentTime);
        }

        // 记录历史数据
        const duration = this.windowSize / this.fs;
        this.sleepHistory.push({
            timestamp: currentTime,
            stage: newStage,
            rms: rms,
            mcr: mcr,
            duration: duration
        });

        // 限制历史记录长度（保留最近8小时）
        const maxHistory = 8 * 3600 / (this.slideStep / this.fs);
        if (this.sleepHistory.length > maxHistory) {
            this.sleepHistory.shift();
        }

        // 更新统计数据
        this.updateStatistics(newStage, duration);
    }

    /**
     * 检测睡眠事件
     * @param {string} newStage - 新的睡眠阶段
     * @param {number} rms - RMS值
     * @param {number} timestamp - 时间戳
     */
    detectSleepEvents(newStage, rms, timestamp) {
        // 检测入睡事件
        if (!this.isSleeping && newStage !== 'awake') {
            this.isSleeping = true;
            this.sleepStartTime = timestamp;
            this.sleepEvents.push({
                timestamp: timestamp,
                event: 'sleep_onset',
                details: `进入${this.getStageName(newStage)}`
            });
            console.log('😴 检测到入睡事件');
        }

        // 检测醒来事件
        if (this.isSleeping && newStage === 'awake') {
            const sleepDuration = (timestamp - this.sleepStartTime) / 1000;
            if (sleepDuration >= this.thresholds.minSleepDuration) {
                this.isSleeping = false;
                this.sleepEvents.push({
                    timestamp: timestamp,
                    event: 'wake_up',
                    details: `睡眠时长: ${(sleepDuration / 60).toFixed(1)}分钟`
                });
                console.log('😊 检测到醒来事件');
            }
        }

        // 检测翻身事件
        if (rms > this.thresholds.turnOverRMS) {
            this.turnOverCount++;
            this.sleepEvents.push({
                timestamp: timestamp,
                event: 'turn_over',
                details: `RMS: ${rms.toFixed(3)}g`
            });
        }

        // 检测在床状态
        const wasInBed = this.isInBed;
        this.isInBed = rms <= this.thresholds.inBedRMS;

        if (!wasInBed && this.isInBed) {
            this.sleepEvents.push({
                timestamp: timestamp,
                event: 'in_bed',
                details: '宠物上床'
            });
        } else if (wasInBed && !this.isInBed) {
            this.sleepEvents.push({
                timestamp: timestamp,
                event: 'out_of_bed',
                details: '宠物离床'
            });
        }
    }

    /**
     * 更新睡眠阶段
     * @param {string} newStage - 新的睡眠阶段
     * @param {number} timestamp - 时间戳
     */
    updateSleepStage(newStage, timestamp) {
        // 记录阶段变化
        if (this.lastStageChangeTime !== null) {
            const duration = (timestamp - this.lastStageChangeTime) / 1000;

            // 只记录持续时间足够长的阶段
            if (duration >= this.thresholds.minStageDuration) {
                this.stageHistory.push({
                    stage: this.currentStage,
                    startTime: this.lastStageChangeTime,
                    endTime: timestamp,
                    duration: duration
                });
            }
        }

        this.currentStage = newStage;
        this.lastStageChangeTime = timestamp;
    }

    /**
     * 更新统计数据
     * @param {string} stage - 当前睡眠阶段
     * @param {number} duration - 持续时间 (秒)
     */
    updateStatistics(stage, duration) {
        switch (stage) {
            case 'deep':
                this.deepSleepTime += duration;
                this.totalSleepTime += duration;
                break;
            case 'light':
                this.lightSleepTime += duration;
                this.totalSleepTime += duration;
                break;
            case 'rem':
                this.remSleepTime += duration;
                this.totalSleepTime += duration;
                break;
            case 'awake':
                this.awakeTime += duration;
                break;
        }

        // 计算睡眠效率
        const totalTime = this.totalSleepTime + this.awakeTime;
        if (totalTime > 0) {
            this.sleepEfficiency = (this.totalSleepTime / totalTime) * 100;
        }
    }

    /**
     * 初始化图表
     */
    initializeCharts() {
        console.log('🎨 初始化睡眠监测图表...');

        // 睡眠阶段时间线图
        const stageTimelineCanvas = document.getElementById('sleepStageTimelineChart');
        if (stageTimelineCanvas) {
            const ctx = stageTimelineCanvas.getContext('2d');
            this.charts.sleepStageTimeline = new Chart(ctx, {
                type: 'line',
                data: {
                    labels: [],
                    datasets: [{
                        label: '睡眠阶段',
                        data: [],
                        borderColor: '#5856D6',
                        backgroundColor: 'rgba(88, 86, 214, 0.1)',
                        stepped: true,
                        fill: true
                    }]
                },
                options: {
                    responsive: true,
                    maintainAspectRatio: false,
                    scales: {
                        y: {
                            min: 0,
                            max: 3,
                            ticks: {
                                stepSize: 1,
                                callback: function(value) {
                                    const stages = ['清醒', '浅睡', '深睡', 'REM'];
                                    return stages[value] || '';
                                }
                            }
                        },
                        x: {
                            display: true
                        }
                    },
                    plugins: {
                        legend: {
                            display: false
                        }
                    }
                }
            });
            console.log('✅ 睡眠阶段时间线图初始化成功');
        }

        // RMS趋势图
        const rmsChartCanvas = document.getElementById('sleepRMSChart');
        if (rmsChartCanvas) {
            const ctx = rmsChartCanvas.getContext('2d');
            this.charts.rmsChart = new Chart(ctx, {
                type: 'line',
                data: {
                    labels: [],
                    datasets: [{
                        label: 'RMS能量',
                        data: [],
                        borderColor: '#FF9500',
                        backgroundColor: 'rgba(255, 149, 0, 0.1)',
                        fill: true,
                        tension: 0.4,
                        pointRadius: 2,
                        pointHoverRadius: 4
                    }]
                },
                options: {
                    responsive: true,
                    maintainAspectRatio: false,
                    scales: {
                        y: {
                            beginAtZero: false,  // 不从0开始，自动调整范围
                            title: {
                                display: true,
                                text: 'RMS (g)'
                            },
                            ticks: {
                                callback: function(value) {
                                    return value.toFixed(3);  // 显示3位小数
                                }
                            }
                        },
                        x: {
                            display: true,
                            title: {
                                display: true,
                                text: '时间'
                            }
                        }
                    },
                    plugins: {
                        legend: {
                            display: false
                        },
                        tooltip: {
                            callbacks: {
                                label: function(context) {
                                    return 'RMS: ' + context.parsed.y.toFixed(4) + 'g';
                                }
                            }
                        }
                    }
                }
            });
            console.log('✅ RMS趋势图初始化成功');
        }

        // 睡眠周期饼图
        const sleepCycleCanvas = document.getElementById('sleepCycleChart');
        if (sleepCycleCanvas) {
            const ctx = sleepCycleCanvas.getContext('2d');
            this.charts.sleepCycleChart = new Chart(ctx, {
                type: 'doughnut',
                data: {
                    labels: ['深睡', '浅睡', 'REM', '清醒'],
                    datasets: [{
                        data: [0, 0, 0, 0],
                        backgroundColor: [
                            '#5856D6',  // 深睡 - 紫色
                            '#007AFF',  // 浅睡 - 蓝色
                            '#FF9500',  // REM - 橙色
                            '#8E8E93'   // 清醒 - 灰色
                        ]
                    }]
                },
                options: {
                    responsive: true,
                    maintainAspectRatio: false,
                    plugins: {
                        legend: {
                            position: 'bottom'
                        }
                    }
                }
            });
            console.log('✅ 睡眠周期饼图初始化成功');
        }

        console.log('🎨 睡眠监测图表初始化完成');
    }

    /**
     * 更新图表
     */
    updateCharts() {
        // 更新睡眠阶段时间线
        if (this.charts.sleepStageTimeline && this.sleepHistory.length > 0) {
            const recentHistory = this.sleepHistory.slice(-60); // 最近60个数据点
            const labels = recentHistory.map(h => {
                const date = new Date(h.timestamp);
                return `${date.getHours()}:${String(date.getMinutes()).padStart(2, '0')}`;
            });
            const data = recentHistory.map(h => {
                const stageMap = { 'awake': 0, 'light': 1, 'deep': 2, 'rem': 3 };
                return stageMap[h.stage] || 0;
            });

            this.charts.sleepStageTimeline.data.labels = labels;
            this.charts.sleepStageTimeline.data.datasets[0].data = data;
            this.charts.sleepStageTimeline.update('none');
        }

        // 更新RMS趋势图
        if (this.charts.rmsChart && this.sleepHistory.length > 0) {
            const recentHistory = this.sleepHistory.slice(-60);
            const labels = recentHistory.map(h => {
                const date = new Date(h.timestamp);
                return `${date.getHours()}:${String(date.getMinutes()).padStart(2, '0')}`;
            });
            const data = recentHistory.map(h => h.rms);

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

            this.charts.rmsChart.data.labels = labels;
            this.charts.rmsChart.data.datasets[0].data = data;
            this.charts.rmsChart.update('none');

            // 每10次更新输出一次调试信息
            if (this.dataUpdateCount % 10 === 0) {
                console.log(`📊 [RMS图表] 数据点=${data.length}, 范围=${minRMS.toFixed(4)}-${maxRMS.toFixed(4)}g`);
            }
        }

        // 更新睡眠周期饼图
        if (this.charts.sleepCycleChart) {
            this.charts.sleepCycleChart.data.datasets[0].data = [
                this.deepSleepTime / 60,   // 转换为分钟
                this.lightSleepTime / 60,
                this.remSleepTime / 60,
                this.awakeTime / 60
            ];
            this.charts.sleepCycleChart.update('none');
        }
    }

    /**
     * 获取睡眠阶段中文名称
     * @param {string} stage - 睡眠阶段
     * @returns {string} 中文名称
     */
    getStageName(stage) {
        const stageNames = {
            'awake': '清醒',
            'light': '浅睡',
            'deep': '深睡',
            'rem': 'REM睡眠'
        };
        return stageNames[stage] || '未知';
    }

    /**
     * 获取睡眠摘要
     * @returns {Object} 睡眠摘要数据
     */
    getSummary() {
        // 计算数据更新率
        const elapsedTime = (Date.now() - this.dataUpdateStartTime) / 1000;
        const dataRate = elapsedTime > 0 ? this.dataUpdateCount / elapsedTime : 0;

        // 计算当前阶段持续时间
        const stageDuration = this.lastStageChangeTime ?
            (Date.now() - this.lastStageChangeTime) / 1000 : 0;

        return {
            currentStage: this.currentStage,
            currentStageName: this.getStageName(this.currentStage),
            isInBed: this.isInBed,
            isSleeping: this.isSleeping,
            totalSleepTime: this.totalSleepTime,
            deepSleepTime: this.deepSleepTime,
            lightSleepTime: this.lightSleepTime,
            remSleepTime: this.remSleepTime,
            awakeTime: this.awakeTime,
            sleepEfficiency: this.sleepEfficiency,
            turnOverCount: this.turnOverCount,
            sleepStartTime: this.sleepStartTime,
            sleepDuration: this.sleepStartTime ? (Date.now() - this.sleepStartTime) / 1000 : 0,
            // 实时指标
            currentRMS: this.currentRMS,
            currentMCR: this.currentMCR,
            lastUpdateTime: this.lastUpdateTime,
            dataRate: dataRate,
            stageDuration: stageDuration
        };
    }

    /**
     * 生成睡眠报告
     * @returns {Object} 详细的睡眠报告
     */
    generateSleepReport() {
        const summary = this.getSummary();

        // 计算睡眠质量评分 (0-100)
        let qualityScore = 0;

        // 深睡比例 (理想: 20-25%)
        const deepSleepRatio = summary.totalSleepTime > 0 ?
            (summary.deepSleepTime / summary.totalSleepTime) * 100 : 0;
        if (deepSleepRatio >= 20 && deepSleepRatio <= 25) {
            qualityScore += 30;
        } else {
            qualityScore += Math.max(0, 30 - Math.abs(deepSleepRatio - 22.5) * 2);
        }

        // 睡眠效率 (理想: >85%)
        if (summary.sleepEfficiency >= 85) {
            qualityScore += 30;
        } else {
            qualityScore += (summary.sleepEfficiency / 85) * 30;
        }

        // 翻身次数 (理想: 每小时<3次)
        const sleepHours = summary.totalSleepTime / 3600;
        const turnOverRate = sleepHours > 0 ? summary.turnOverCount / sleepHours : 0;
        if (turnOverRate < 3) {
            qualityScore += 20;
        } else {
            qualityScore += Math.max(0, 20 - (turnOverRate - 3) * 5);
        }

        // REM睡眠比例 (理想: 20-25%)
        const remSleepRatio = summary.totalSleepTime > 0 ?
            (summary.remSleepTime / summary.totalSleepTime) * 100 : 0;
        if (remSleepRatio >= 20 && remSleepRatio <= 25) {
            qualityScore += 20;
        } else {
            qualityScore += Math.max(0, 20 - Math.abs(remSleepRatio - 22.5) * 2);
        }

        // 睡眠质量等级
        let qualityLevel = '差';
        if (qualityScore >= 80) qualityLevel = '优秀';
        else if (qualityScore >= 60) qualityLevel = '良好';
        else if (qualityScore >= 40) qualityLevel = '一般';

        return {
            ...summary,
            qualityScore: Math.round(qualityScore),
            qualityLevel: qualityLevel,
            deepSleepRatio: deepSleepRatio,
            remSleepRatio: remSleepRatio,
            turnOverRate: turnOverRate,
            sleepEvents: this.sleepEvents,
            stageHistory: this.stageHistory,
            recommendations: this.generateRecommendations(qualityScore, deepSleepRatio, remSleepRatio, turnOverRate)
        };
    }

    /**
     * 生成睡眠建议
     * @param {number} qualityScore - 睡眠质量评分
     * @param {number} deepSleepRatio - 深睡比例
     * @param {number} remSleepRatio - REM睡眠比例
     * @param {number} turnOverRate - 翻身频率
     * @returns {Array} 建议列表
     */
    generateRecommendations(qualityScore, deepSleepRatio, remSleepRatio, turnOverRate) {
        const recommendations = [];

        if (qualityScore < 60) {
            recommendations.push('整体睡眠质量需要改善，建议增加运动量和调整作息时间');
        }

        if (deepSleepRatio < 15) {
            recommendations.push('深睡时间不足，建议睡前减少刺激性活动');
        } else if (deepSleepRatio > 30) {
            recommendations.push('深睡时间过长，可能存在过度疲劳，建议适当增加白天活动');
        }

        if (remSleepRatio < 15) {
            recommendations.push('REM睡眠不足，建议保持规律作息');
        }

        if (turnOverRate > 5) {
            recommendations.push('翻身频繁，可能睡眠环境不舒适或存在不适');
        }

        if (recommendations.length === 0) {
            recommendations.push('睡眠质量良好，继续保持！');
        }

        return recommendations;
    }

    /**
     * 重置睡眠数据
     */
    reset() {
        // 清空缓冲区
        this.axBuffer = [];
        this.ayBuffer = [];
        this.azBuffer = [];
        this.timestampBuffer = [];

        // 重置状态
        this.currentStage = 'awake';
        this.isInBed = false;
        this.isSleeping = false;
        this.sleepStartTime = null;
        this.lastStageChangeTime = null;

        // 重置实时指标
        this.currentRMS = 0;
        this.currentMCR = 0;
        this.lastUpdateTime = null;
        this.dataUpdateCount = 0;
        this.dataUpdateStartTime = Date.now();

        // 清空历史记录
        this.sleepHistory = [];
        this.sleepEvents = [];
        this.stageHistory = [];

        // 重置统计数据
        this.totalSleepTime = 0;
        this.deepSleepTime = 0;
        this.lightSleepTime = 0;
        this.remSleepTime = 0;
        this.awakeTime = 0;
        this.sleepEfficiency = 0;
        this.turnOverCount = 0;

        // 更新图表
        if (this.charts.sleepStageTimeline) {
            this.charts.sleepStageTimeline.data.labels = [];
            this.charts.sleepStageTimeline.data.datasets[0].data = [];
            this.charts.sleepStageTimeline.update();
        }

        if (this.charts.rmsChart) {
            this.charts.rmsChart.data.labels = [];
            this.charts.rmsChart.data.datasets[0].data = [];
            this.charts.rmsChart.update();
        }

        if (this.charts.sleepCycleChart) {
            this.charts.sleepCycleChart.data.datasets[0].data = [0, 0, 0, 0];
            this.charts.sleepCycleChart.update();
        }

        console.log('🔄 睡眠监测数据已重置');
    }
}

// 导出SleepMonitor类到全局作用域
if (typeof window !== 'undefined') {
    window.SleepMonitor = SleepMonitor;
    console.log('✅ SleepMonitor类已导出到全局作用域');
}