/**
 * 活动量与步数监测模块 (Activity & Step Counter Module)
 * 基于IMU加速度计数据，实现活动量和步数可视化
 *
 * 功能特性:
 * 1. 实时计算活动量 (ENMO - Euclidean Norm Minus One)
 * 2. 实时步数统计 (基于峰值检测算法)
 * 3. 累计数据展示 (环形图和柱状图)
 * 4. 活动强度分级 (静息/轻度/中度/剧烈)
 * 5. 每小时/每天的活动趋势分析
 */

class ActivityMonitor {
    constructor(samplingRate = 50, petWeight = 10.0) {
        this.fs = samplingRate; // 采样率 (Hz)
        this.petWeight = petWeight; // 宠物体重 (kg)

        // 计算基础代谢率 (RER - Resting Energy Requirement)
        // 公式: 70 * weight_kg ^ 0.75 (kcal/day)
        this.rerDaily = 70 * Math.pow(this.petWeight, 0.75);
        // 转换为每秒的基础消耗 (BMR per second)
        this.bmrPerSec = this.rerDaily / 86400.0;

        // 数据缓存
        this.accBufferX = [];
        this.accBufferY = [];
        this.accBufferZ = [];
        this.timestamps = [];

        // 活动量统计
        this.activityHistory = []; // {timestamp, enmo, mad, intensity, mets, calories}
        this.stepHistory = [];      // {timestamp, steps}

        // 累计数据
        this.totalENMO = 0;         // 总活动量
        this.totalSteps = 0;        // 总步数
        this.totalCalories = 0;     // 总卡路里消耗 (kcal)
        this.dailyGoal = 1000;      // 每日步数目标
        // 每日活动量目标（g单位ENMO，慢走≈0.2/s，小跑≈0.5/s）
        // 参考：中等活跃犬 10分钟慢走 ≈ 12，30分钟小跑 ≈ 90 → 日目标100合理
        this.activityGoal = 100;
        this.calorieGoal = 100;     // 每日卡路里目标 (kcal)

        // 实时状态
        this.currentIntensity = 'resting'; // resting, light, moderate, vigorous
        this.currentMETs = 1.0;     // 当前代谢当量
        this.lastUpdateTime = Date.now();

        // 步数检测状态
        this.globalSampleCount = 0;  // 全局样本计数器（永不重置，用于准确跟踪峰值位置）
        this.lastPeakGlobalIndex = -1;  // 上次检测到峰值的全局索引（在整个缓存中的位置）
        this.minPeakDistance = Math.floor(this.fs / 3); // 最小步间距

        // 图表对象
        this.charts = {};

        // 带通滤波器系数 (0.5Hz - 5Hz, 用于步数检测)
        // 简化的Butterworth滤波器实现
        this.filterB = null;
        this.filterA = null;
        this.initBandpassFilter();

        // 每小时数据统计
        this.hourlyData = new Array(24).fill(null).map(() => ({
            steps: 0,
            activity: 0,
            calories: 0,
            intensity: 'resting'
        }));

        // 性能优化：更新节流
        this.lastChartUpdate = 0;
        this.chartUpdateInterval = 1000; // 1秒更新一次图表
    }

    /**
     * 初始化带通滤波器 (0.5Hz - 5Hz)
     */
    initBandpassFilter() {
        // 简化实现：使用移动平均和差分近似带通滤波
        // 实际应用中可以使用更精确的IIR滤波器
        this.filterWindowSize = Math.floor(this.fs / 2); // 0.5秒窗口
    }

    /**
     * 添加加速度数据
     * @param {number} ax - X轴加速度 (m/s²，内部自动转换为 g)
     * @param {number} ay - Y轴加速度 (m/s²，内部自动转换为 g)
     * @param {number} az - Z轴加速度 (m/s²，内部自动转换为 g)
     * @param {number} timestamp - 时间戳 (ms)
     */
    addAccelerometerData(ax, ay, az, timestamp) {
        // 项圈IMU输出单位为 m/s²，ENMO公式需要 g 单位（减去 1.0g 重力）
        // 统一在入口转换：1g = 9.80665 m/s²
        const G = 9.80665;
        const axG = ax / G;
        const ayG = ay / G;
        const azG = az / G;

        this.accBufferX.push(axG);
        this.accBufferY.push(ayG);
        this.accBufferZ.push(azG);
        this.timestamps.push(timestamp);

        // 增加全局样本计数器
        this.globalSampleCount++;

        // 限制缓存大小 (保留最近10秒数据)
        const maxBufferSize = this.fs * 10;
        if (this.accBufferX.length > maxBufferSize) {
            this.accBufferX.shift();
            this.accBufferY.shift();
            this.accBufferZ.shift();
            this.timestamps.shift();
        }

        // 每秒计算一次活动量和步数
        // 关键修复：只在累积了足够的新样本时才处理（每秒一次，而不是每个样本都处理）
        if (this.globalSampleCount % this.fs === 0 && this.accBufferX.length >= this.fs) {
            this.processActivityMetrics();
        }
    }

    /**
     * 计算活动量指标 (ENMO & MAD)
     * 返回这段数据的总ENMO（不是平均值）
     */
    calculateActivityMetrics(accX, accY, accZ) {
        const n = accX.length;
        if (n === 0) return { enmo: 0, mad: 0, intensity: 'resting' };

        // 1. 计算合加速度 (Signal Vector Magnitude)
        const svm = [];
        for (let i = 0; i < n; i++) {
            const magnitude = Math.sqrt(
                accX[i] * accX[i] +
                accY[i] * accY[i] +
                accZ[i] * accZ[i]
            );
            svm.push(magnitude);
        }

        // 2. ENMO (Euclidean Norm Minus One)
        // ENMO标准定义：求和后除以采样率，得到归一化的活动量
        // 这样ENMO值独立于采样率，便于设置合理的每日目标
        let enmoSum = 0;
        for (let i = 0; i < n; i++) {
            enmoSum += Math.max(0, svm[i] - 1.0);
        }

        // 关键修复：除以采样率进行归一化
        // 这样1秒的数据返回的ENMO值在合理范围内（0.01-0.5）
        // 而不是原来的虚高值（0.5-25）
        const enmo = enmoSum / this.fs;

        // 3. MAD (Mean Amplitude Deviation)
        const svmMean = svm.reduce((a, b) => a + b, 0) / n;
        let madSum = 0;
        for (let i = 0; i < n; i++) {
            madSum += Math.abs(svm[i] - svmMean);
        }
        const mad = madSum / n;

        // 4. 活动强度分级
        let intensity = 'resting';
        if (mad > 0.20) intensity = 'vigorous';
        else if (mad > 0.12) intensity = 'moderate';
        else if (mad > 0.05) intensity = 'light';

        // 5. 计算METs (代谢当量) 和卡路里
        // 根据ENMO强度映射METs
        let mets = 1.0; // 默认静息状态
        if (enmo > 0.50) mets = 6.0;      // 剧烈活动/狂奔
        else if (enmo > 0.20) mets = 4.0; // 中度活动/小跑
        else if (enmo > 0.05) mets = 2.0; // 轻微活动/慢走

        // 计算这段时间的卡路里消耗
        // Calorie = BMR_per_sec * MET * duration
        const duration = n / this.fs; // 秒
        const calories = this.bmrPerSec * mets * duration;

        return {
            enmo: enmo,
            mad: mad,
            intensity: intensity,
            mets: mets,
            calories: calories,
            svm: svm
        };
    }

    /**
     * 步数检测算法 - 使用全局索引跟踪避免重复计数
     * @param {number} startGlobalIndex - 检测窗口在全局缓存中的起始索引
     */
    countStepsInWindow(accX, accY, accZ, startGlobalIndex) {
        const n = accX.length;
        if (n < 15) {
            return 0;
        }

        // 1. 计算合加速度
        const svm = [];
        for (let i = 0; i < n; i++) {
            const magnitude = Math.sqrt(
                accX[i] * accX[i] +
                accY[i] * accY[i] +
                accZ[i] * accZ[i]
            );
            svm.push(magnitude);
        }

        // 2. 简化的带通滤波
        const filtered = this.simpleBandpassFilter(svm);

        // 3. 峰值检测
        const minPeakHeight = 0.10;
        let newSteps = 0;
        let peakValues = [];

        for (let i = 1; i < filtered.length - 1; i++) {
            const globalIndex = startGlobalIndex + i;

            // 检测局部最大值
            if (filtered[i] > filtered[i - 1] &&
                filtered[i] > filtered[i + 1] &&
                filtered[i] > minPeakHeight) {

                // 检查是否与上次峰值距离足够远（使用全局索引）
                if (globalIndex - this.lastPeakGlobalIndex >= this.minPeakDistance) {
                    newSteps++;
                    this.lastPeakGlobalIndex = globalIndex;
                    peakValues.push(filtered[i].toFixed(3));
                }
            }
        }

        // 调试输出
        if (newSteps > 0) {
            const maxFiltered = Math.max(...filtered);
            console.log(`🚶 步数检测: ${newSteps}步, 最大峰值=${maxFiltered.toFixed(3)}, 阈值=${minPeakHeight}, 峰值=${peakValues.join(',')}, 全局索引=${startGlobalIndex}-${startGlobalIndex + n}, 上次峰值=${this.lastPeakGlobalIndex}`);
        }

        return newSteps;
    }

    /**
     * 简化的带通滤波器
     */
    simpleBandpassFilter(signal) {
        const n = signal.length;
        if (n < 5) return signal;

        // 移动平均去除高频噪声
        const smoothed = [];
        const windowSize = 3;
        for (let i = 0; i < n; i++) {
            let sum = 0;
            let count = 0;
            for (let j = Math.max(0, i - windowSize); j <= Math.min(n - 1, i + windowSize); j++) {
                sum += signal[j];
                count++;
            }
            smoothed.push(sum / count);
        }

        // 去除直流分量 (减去均值)
        const mean = smoothed.reduce((a, b) => a + b, 0) / n;
        return smoothed.map(v => v - mean);
    }

    /**
     * 处理活动量指标 (每秒调用一次)
     */
    processActivityMetrics() {
        const now = Date.now();
        const currentLength = this.accBufferX.length;

        // 需要至少1秒的数据才能处理
        if (currentLength < this.fs) {
            return;
        }

        // 只处理最近1秒的新数据来计算活动量
        const newDataX = this.accBufferX.slice(-this.fs);
        const newDataY = this.accBufferY.slice(-this.fs);
        const newDataZ = this.accBufferZ.slice(-this.fs);

        // 计算这1秒数据的活动量
        const metrics = this.calculateActivityMetrics(newDataX, newDataY, newDataZ);

        // 步数检测：使用最近2秒的数据（如果有的话）
        let newSteps = 0;
        const windowSize = Math.min(currentLength, this.fs * 2);
        const startIdx = currentLength - windowSize;
        // 使用真实的全局索引
        const startGlobalIndex = this.globalSampleCount - windowSize;

        newSteps = this.countStepsInWindow(
            this.accBufferX.slice(startIdx),
            this.accBufferY.slice(startIdx),
            this.accBufferZ.slice(startIdx),
            startGlobalIndex
        );

        // 调试日志
        if (this.activityHistory.length % 5 === 0) {
            console.log(`📊 [${this.activityHistory.length}秒] ENMO=${metrics.enmo.toFixed(4)}, MAD=${metrics.mad.toFixed(4)}, 强度=${metrics.intensity}, METs=${metrics.mets.toFixed(1)}, 卡路里=${metrics.calories.toFixed(4)}, 新步数=${newSteps}, 总步数=${this.totalSteps}`);
        }

        // 累加ENMO（只在活动时）
        if (metrics.intensity !== 'resting') {
            this.totalENMO += metrics.enmo;
            if (this.activityHistory.length % 5 === 0) {
                console.log(`📈 累加ENMO: +${metrics.enmo.toFixed(4)}, 总计=${this.totalENMO.toFixed(2)}`);
            }
        }

        // 累加卡路里（包括静息状态的基础代谢）
        this.totalCalories += metrics.calories;
        this.currentMETs = metrics.mets;

        // 累加步数
        if (newSteps > 0) {
            this.totalSteps += newSteps;
            console.log(`👣 检测到步数: +${newSteps}, 总计: ${this.totalSteps}`);
        }

        // 记录历史数据
        this.activityHistory.push({
            timestamp: now,
            enmo: metrics.enmo,
            mad: metrics.mad,
            intensity: metrics.intensity,
            mets: metrics.mets,
            calories: metrics.calories
        });

        if (newSteps > 0) {
            this.stepHistory.push({
                timestamp: now,
                steps: newSteps
            });
        }

        // 限制历史记录长度 (保留最近1小时)
        const maxHistoryTime = 3600 * 1000;
        this.activityHistory = this.activityHistory.filter(
            item => now - item.timestamp < maxHistoryTime
        );
        this.stepHistory = this.stepHistory.filter(
            item => now - item.timestamp < maxHistoryTime
        );

        // 更新每小时统计
        this.updateHourlyStats(now, metrics.enmo, newSteps, metrics.intensity, metrics.calories);

        // 更新当前状态
        this.currentIntensity = metrics.intensity;
        this.lastUpdateTime = now;

        // 更新图表 (节流)
        if (now - this.lastChartUpdate > this.chartUpdateInterval) {
            this.updateCharts();
            this.lastChartUpdate = now;
        }
    }

    /**
     * 更新每小时统计
     */
    updateHourlyStats(timestamp, enmo, steps, intensity, calories) {
        const hour = new Date(timestamp).getHours();
        this.hourlyData[hour].steps += steps;
        this.hourlyData[hour].activity += enmo;
        this.hourlyData[hour].calories = (this.hourlyData[hour].calories || 0) + calories;

        // 更新强度 (取最高强度)
        const intensityLevels = { resting: 0, light: 1, moderate: 2, vigorous: 3 };
        const currentLevel = intensityLevels[this.hourlyData[hour].intensity] || 0;
        const newLevel = intensityLevels[intensity] || 0;
        if (newLevel > currentLevel) {
            this.hourlyData[hour].intensity = intensity;
        }
    }

    /**
     * 初始化图表
     */
    initializeCharts() {
        console.log('🎨 初始化活动监测图表...');

        // 1. 步数目标环形图
        const stepsRingCtx = document.getElementById('activityStepsRingChart');
        if (stepsRingCtx) {
            console.log('✅ 找到步数环形图canvas');
            try {
                this.charts.stepsRing = new Chart(stepsRingCtx, {
                    type: 'doughnut',
                    data: {
                        labels: ['已完成', '未完成'],
                        datasets: [{
                            data: [0, 100],
                            backgroundColor: ['#34C759', '#E5E5EA'],
                            borderWidth: 0
                        }]
                    },
                    options: {
                        responsive: true,
                        maintainAspectRatio: false,
                        cutout: '75%',
                        plugins: {
                            legend: { display: false },
                            tooltip: { enabled: false }
                        }
                    }
                });
                console.log('✅ 步数环形图初始化成功');
            } catch (e) {
                console.error('❌ 步数环形图初始化失败:', e);
            }
        } else {
            console.warn('⚠️ 未找到步数环形图canvas元素');
        }

        // 2. 活动量目标环形图
        const activityRingCtx = document.getElementById('activityENMORingChart');
        if (activityRingCtx) {
            console.log('✅ 找到活动量环形图canvas');
            try {
                this.charts.activityRing = new Chart(activityRingCtx, {
                    type: 'doughnut',
                    data: {
                        labels: ['已完成', '未完成'],
                        datasets: [{
                            data: [0, 100],
                            backgroundColor: ['#FF2D55', '#E5E5EA'],
                            borderWidth: 0
                        }]
                    },
                    options: {
                        responsive: true,
                        maintainAspectRatio: false,
                        cutout: '75%',
                        plugins: {
                            legend: { display: false },
                            tooltip: { enabled: false }
                        }
                    }
                });
                console.log('✅ 活动量环形图初始化成功');
            } catch (e) {
                console.error('❌ 活动量环形图初始化失败:', e);
            }
        } else {
            console.warn('⚠️ 未找到活动量环形图canvas元素');
        }

        // 3. 卡路里目标环形图
        const calorieRingCtx = document.getElementById('activityCalorieRingChart');
        if (calorieRingCtx) {
            console.log('✅ 找到卡路里环形图canvas');
            try {
                this.charts.calorieRing = new Chart(calorieRingCtx, {
                    type: 'doughnut',
                    data: {
                        labels: ['已消耗', '未消耗'],
                        datasets: [{
                            data: [0, 100],
                            backgroundColor: ['#FF9500', '#E5E5EA'],
                            borderWidth: 0
                        }]
                    },
                    options: {
                        responsive: true,
                        maintainAspectRatio: false,
                        cutout: '75%',
                        plugins: {
                            legend: { display: false },
                            tooltip: { enabled: false }
                        }
                    }
                });
                console.log('✅ 卡路里环形图初始化成功');
            } catch (e) {
                console.error('❌ 卡路里环形图初始化失败:', e);
            }
        } else {
            console.warn('⚠️ 未找到卡路里环形图canvas元素');
        }

        // 4. 每小时步数柱状图
        const hourlyStepsCtx = document.getElementById('activityHourlyStepsChart');
        if (hourlyStepsCtx) {
            console.log('✅ 找到每小时步数图canvas');
            try {
                const hours = Array.from({ length: 24 }, (_, i) => `${i}:00`);
                this.charts.hourlySteps = new Chart(hourlyStepsCtx, {
                    type: 'bar',
                    data: {
                        labels: hours,
                        datasets: [{
                            label: '步数',
                            data: new Array(24).fill(0),
                            backgroundColor: '#34C759',
                            borderRadius: 4
                        }]
                    },
                    options: {
                        responsive: true,
                        maintainAspectRatio: false,
                        scales: {
                            y: {
                                beginAtZero: true,
                                title: { display: true, text: '步数' }
                            },
                            x: {
                                title: { display: true, text: '时间' }
                            }
                        },
                        plugins: {
                            legend: { display: false }
                        }
                    }
                });
                console.log('✅ 每小时步数图初始化成功');
            } catch (e) {
                console.error('❌ 每小时步数图初始化失败:', e);
            }
        } else {
            console.warn('⚠️ 未找到每小时步数图canvas元素');
        }

        // 4. 每小时活动量柱状图
        const hourlyActivityCtx = document.getElementById('activityHourlyENMOChart');
        if (hourlyActivityCtx) {
            console.log('✅ 找到每小时活动量图canvas');
            try {
                const hours = Array.from({ length: 24 }, (_, i) => `${i}:00`);
                this.charts.hourlyActivity = new Chart(hourlyActivityCtx, {
                    type: 'bar',
                    data: {
                        labels: hours,
                        datasets: [{
                            label: '活动量',
                            data: new Array(24).fill(0),
                            backgroundColor: '#FF2D55',
                            borderRadius: 4
                        }]
                    },
                    options: {
                        responsive: true,
                        maintainAspectRatio: false,
                        scales: {
                            y: {
                                beginAtZero: true,
                                title: { display: true, text: 'ENMO' }
                            },
                            x: {
                                title: { display: true, text: '时间' }
                            }
                        },
                        plugins: {
                            legend: { display: false }
                        }
                    }
                });
                console.log('✅ 每小时活动量图初始化成功');
            } catch (e) {
                console.error('❌ 每小时活动量图初始化失败:', e);
            }
        } else {
            console.warn('⚠️ 未找到每小时活动量图canvas元素');
        }

        // 5. 每小时卡路里柱状图
        const hourlyCalorieCtx = document.getElementById('activityHourlyCalorieChart');
        if (hourlyCalorieCtx) {
            console.log('✅ 找到每小时卡路里图canvas');
            try {
                const hours = Array.from({ length: 24 }, (_, i) => `${i}:00`);
                this.charts.hourlyCalorie = new Chart(hourlyCalorieCtx, {
                    type: 'bar',
                    data: {
                        labels: hours,
                        datasets: [{
                            label: '卡路里 (kcal)',
                            data: new Array(24).fill(0),
                            backgroundColor: '#FF9500',
                            borderRadius: 4
                        }]
                    },
                    options: {
                        responsive: true,
                        maintainAspectRatio: false,
                        scales: {
                            y: {
                                beginAtZero: true,
                                title: { display: true, text: '卡路里 (kcal)' }
                            },
                            x: {
                                title: { display: true, text: '时间' }
                            }
                        },
                        plugins: {
                            legend: { display: false }
                        }
                    }
                });
                console.log('✅ 每小时卡路里图初始化成功');
            } catch (e) {
                console.error('❌ 每小时卡路里图初始化失败:', e);
            }
        } else {
            console.warn('⚠️ 未找到每小时卡路里图canvas元素');
        }

        // 6. 实时活动强度趋势图
        const intensityTrendCtx = document.getElementById('activityIntensityTrendChart');
        if (intensityTrendCtx) {
            console.log('✅ 找到活动强度趋势图canvas');
            try {
                this.charts.intensityTrend = new Chart(intensityTrendCtx, {
                    type: 'line',
                    data: {
                        labels: [],
                        datasets: [{
                            label: 'MAD强度',
                            data: [],
                            borderColor: '#007AFF',
                            backgroundColor: 'rgba(0, 122, 255, 0.1)',
                            fill: true,
                            tension: 0.4
                        }]
                    },
                    options: {
                        responsive: true,
                        maintainAspectRatio: false,
                        scales: {
                            y: {
                                beginAtZero: true,
                                title: { display: true, text: 'MAD (g)' }
                            },
                            x: {
                                title: { display: true, text: '时间' },
                                display: true
                            }
                        },
                        plugins: {
                            legend: { display: true }
                        }
                    }
                });
                console.log('✅ 活动强度趋势图初始化成功');
            } catch (e) {
                console.error('❌ 活动强度趋势图初始化失败:', e);
            }
        } else {
            console.warn('⚠️ 未找到活动强度趋势图canvas元素');
        }

        console.log('🎨 图表初始化完成，已创建图表数量:', Object.keys(this.charts).length);
    }

    /**
     * 更新所有图表
     */
    updateCharts() {
        if (Object.keys(this.charts).length === 0) {
            console.warn('⚠️ 图表未初始化，跳过更新');
            return;
        }

        this.updateRingCharts();
        this.updateHourlyCharts();
        this.updateIntensityTrend();
        this.updateStatistics();
    }

    /**
     * 更新环形图 (目标完成度)
     */
    updateRingCharts() {
        // 更新步数环
        if (this.charts.stepsRing) {
            const stepsPercent = Math.min(100, (this.totalSteps / this.dailyGoal) * 100);
            this.charts.stepsRing.data.datasets[0].data = [stepsPercent, 100 - stepsPercent];
            this.charts.stepsRing.update('none');
        }

        // 更新活动量环
        if (this.charts.activityRing) {
            const activityPercent = Math.min(100, (this.totalENMO / this.activityGoal) * 100);
            this.charts.activityRing.data.datasets[0].data = [activityPercent, 100 - activityPercent];
            this.charts.activityRing.update('none');
        }

        // 更新卡路里环
        if (this.charts.calorieRing) {
            const caloriePercent = Math.min(100, (this.totalCalories / this.calorieGoal) * 100);
            this.charts.calorieRing.data.datasets[0].data = [caloriePercent, 100 - caloriePercent];
            this.charts.calorieRing.update('none');
        }
    }

    /**
     * 更新每小时图表
     */
    updateHourlyCharts() {
        // 更新步数柱状图
        if (this.charts.hourlySteps) {
            const stepsData = this.hourlyData.map(h => h.steps);
            this.charts.hourlySteps.data.datasets[0].data = stepsData;

            // 根据活动强度设置颜色
            const colors = this.hourlyData.map(h => {
                switch (h.intensity) {
                    case 'vigorous': return '#FF2D55';
                    case 'moderate': return '#FF9500';
                    case 'light': return '#34C759';
                    default: return '#E5E5EA';
                }
            });
            this.charts.hourlySteps.data.datasets[0].backgroundColor = colors;
            this.charts.hourlySteps.update('none');
        }

        // 更新活动量柱状图
        if (this.charts.hourlyActivity) {
            const activityData = this.hourlyData.map(h => parseFloat(h.activity.toFixed(2)));
            this.charts.hourlyActivity.data.datasets[0].data = activityData;

            // 根据活动强度设置颜色
            const colors = this.hourlyData.map(h => {
                switch (h.intensity) {
                    case 'vigorous': return '#FF2D55';
                    case 'moderate': return '#FF9500';
                    case 'light': return '#34C759';
                    default: return '#E5E5EA';
                }
            });
            this.charts.hourlyActivity.data.datasets[0].backgroundColor = colors;
            this.charts.hourlyActivity.update('none');
        }

        // 更新卡路里柱状图
        if (this.charts.hourlyCalorie) {
            const calorieData = this.hourlyData.map(h => parseFloat((h.calories || 0).toFixed(2)));
            this.charts.hourlyCalorie.data.datasets[0].data = calorieData;

            // 根据活动强度设置颜色
            const colors = this.hourlyData.map(h => {
                switch (h.intensity) {
                    case 'vigorous': return '#FF2D55';
                    case 'moderate': return '#FF9500';
                    case 'light': return '#34C759';
                    default: return '#E5E5EA';
                }
            });
            this.charts.hourlyCalorie.data.datasets[0].backgroundColor = colors;
            this.charts.hourlyCalorie.update('none');
        }
    }

    /**
     * 更新活动强度趋势图
     */
    updateIntensityTrend() {
        if (!this.charts.intensityTrend) {
            console.warn('⚠️ 趋势图未初始化');
            return;
        }

        // 显示最近10分钟的数据，每5秒采样一次以减少数据点
        const tenMinutesAgo = Date.now() - 10 * 60 * 1000;
        const recentData = this.activityHistory.filter(item => item.timestamp >= tenMinutesAgo);

        // 如果数据太少，不更新
        if (recentData.length === 0) {
            console.log('📉 趋势图: 暂无数据');
            return;
        }

        // 每5秒采样一次 (假设每秒一个数据点)
        const sampledData = [];
        for (let i = 0; i < recentData.length; i += 5) {
            sampledData.push(recentData[i]);
        }

        const labels = sampledData.map(item => {
            const date = new Date(item.timestamp);
            return `${String(date.getHours()).padStart(2, '0')}:${String(date.getMinutes()).padStart(2, '0')}:${String(date.getSeconds()).padStart(2, '0')}`;
        });

        const madData = sampledData.map(item => item.mad.toFixed(4));

        this.charts.intensityTrend.data.labels = labels;
        this.charts.intensityTrend.data.datasets[0].data = madData;
        this.charts.intensityTrend.update('none');

        if (sampledData.length % 20 === 0) {
            console.log(`📉 趋势图更新: ${sampledData.length}个数据点`);
        }
    }

    /**
     * 更新统计信息显示
     */
    updateStatistics() {
        // 更新步数显示
        const stepsEl = document.getElementById('activityTotalSteps');
        if (stepsEl) {
            stepsEl.textContent = this.totalSteps.toLocaleString();
        }

        // 更新步数百分比（圆圈中间）
        const stepsPercentEl = document.getElementById('activityStepsPercent');
        if (stepsPercentEl) {
            const percent = Math.round((this.totalSteps / this.dailyGoal) * 100);
            stepsPercentEl.textContent = `${percent}%`;
        }

        // 更新活动量显示
        const enmoEl = document.getElementById('activityTotalENMO');
        if (enmoEl) {
            enmoEl.textContent = this.totalENMO.toFixed(2);
        }

        // 更新活动量百分比（圆圈中间）
        const enmoPercentEl = document.getElementById('activityENMOPercent');
        if (enmoPercentEl) {
            const percent = Math.round((this.totalENMO / this.activityGoal) * 100);
            enmoPercentEl.textContent = `${percent}%`;
        }

        // 更新卡路里显示
        const calorieEl = document.getElementById('activityTotalCalories');
        if (calorieEl) {
            calorieEl.textContent = this.totalCalories.toFixed(2);
        }

        // 更新卡路里百分比（圆圈中间）
        const caloriePercentEl = document.getElementById('activityCaloriePercent');
        if (caloriePercentEl) {
            const percent = Math.round((this.totalCalories / this.calorieGoal) * 100);
            caloriePercentEl.textContent = `${percent}%`;
        }

        // 更新METs显示
        const metsEl = document.getElementById('activityCurrentMETs');
        if (metsEl) {
            metsEl.textContent = this.currentMETs.toFixed(1);
        }

        // 更新体重显示
        const weightEl = document.getElementById('activityPetWeight');
        if (weightEl) {
            weightEl.textContent = `${this.petWeight.toFixed(1)} kg`;
        }

        // 更新RER显示
        const rerEl = document.getElementById('activityRER');
        if (rerEl) {
            rerEl.textContent = `${this.rerDaily.toFixed(0)} kcal/day`;
        }

        // 更新当前强度显示
        const intensityEl = document.getElementById('activityCurrentIntensity');
        if (intensityEl) {
            const intensityText = {
                'resting': '静息',
                'light': '轻度活动',
                'moderate': '中度活动',
                'vigorous': '剧烈活动'
            };
            intensityEl.textContent = intensityText[this.currentIntensity] || '未知';

            // 设置颜色
            const intensityColors = {
                'resting': '#8E8E93',
                'light': '#34C759',
                'moderate': '#FF9500',
                'vigorous': '#FF2D55'
            };
            intensityEl.style.color = intensityColors[this.currentIntensity] || '#000';
        }

        // 更新最后更新时间
        const lastUpdateEl = document.getElementById('activityLastUpdate');
        if (lastUpdateEl) {
            const date = new Date(this.lastUpdateTime);
            lastUpdateEl.textContent = `${date.getHours()}:${String(date.getMinutes()).padStart(2, '0')}:${String(date.getSeconds()).padStart(2, '0')}`;
        }
    }

    /**
     * 重置每日数据 (在新的一天开始时调用)
     */
    resetDailyData() {
        this.totalSteps = 0;
        this.totalENMO = 0;
        this.totalCalories = 0;
        this.lastPeakGlobalIndex = -1;
        this.hourlyData = new Array(24).fill(null).map(() => ({
            steps: 0,
            activity: 0,
            calories: 0,
            intensity: 'resting'
        }));
        this.activityHistory = [];
        this.stepHistory = [];
        this.updateCharts();
        console.log('🔄 每日数据已重置');
    }

    /**
     * 获取统计摘要
     */
    getSummary() {
        return {
            totalSteps: this.totalSteps,
            totalENMO: this.totalENMO,
            totalCalories: this.totalCalories,
            petWeight: this.petWeight,
            rerDaily: this.rerDaily,
            currentMETs: this.currentMETs,
            stepsGoalPercent: Math.round((this.totalSteps / this.dailyGoal) * 100),
            activityGoalPercent: Math.round((this.totalENMO / this.activityGoal) * 100),
            calorieGoalPercent: Math.round((this.totalCalories / this.calorieGoal) * 100),
            currentIntensity: this.currentIntensity,
            lastUpdate: this.lastUpdateTime
        };
    }
}

// 导出供全局使用
if (typeof window !== 'undefined') {
    window.ActivityMonitor = ActivityMonitor;
}
