// 活动监测模块关键修复补丁
// 请在浏览器控制台中运行此代码来修复步数检测问题

if (window.app && window.app.activityMonitor) {
    const m = window.app.activityMonitor;

    // 添加全局样本计数器
    if (typeof m.globalSampleCount === 'undefined') {
        m.globalSampleCount = m.accBufferX.length;
        console.log('✅ 已添加全局样本计数器:', m.globalSampleCount);
    }

    // 重写addAccelerometerData方法，增加全局计数
    const originalAdd = m.addAccelerometerData.bind(m);
    m.addAccelerometerData = function(ax, ay, az, timestamp) {
        originalAdd(ax, ay, az, timestamp);
        this.globalSampleCount++;
    };

    // 重写countStepsInWindow方法，使用真实的全局索引
    m.countStepsInWindow = function(accX, accY, accZ, startGlobalIndex) {
        const n = accX.length;
        if (n < 15) return 0;

        // 计算合加速度
        const svm = [];
        for (let i = 0; i < n; i++) {
            const magnitude = Math.sqrt(
                accX[i] * accX[i] +
                accY[i] * accY[i] +
                accZ[i] * accZ[i]
            );
            svm.push(magnitude);
        }

        // 简化的带通滤波
        const filtered = this.simpleBandpassFilter(svm);

        // 峰值检测
        const minPeakHeight = 0.10;
        let newSteps = 0;
        let peakValues = [];

        for (let i = 1; i < filtered.length - 1; i++) {
            // 使用真实的全局索引
            const globalIndex = startGlobalIndex + i;

            // 检测局部最大值
            if (filtered[i] > filtered[i - 1] &&
                filtered[i] > filtered[i + 1] &&
                filtered[i] > minPeakHeight) {

                // 检查是否与上次峰值距离足够远
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
    };

    // 重写processActivityMetrics，使用全局计数器
    m.processActivityMetrics = function() {
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
            console.log(`📊 [${this.activityHistory.length}秒] ENMO=${metrics.enmo.toFixed(4)}, MAD=${metrics.mad.toFixed(4)}, 强度=${metrics.intensity}, 新步数=${newSteps}, 总步数=${this.totalSteps}, 全局样本=${this.globalSampleCount}`);
        }

        // 累加ENMO（只在活动时）
        if (metrics.intensity !== 'resting') {
            this.totalENMO += metrics.enmo;
            if (this.activityHistory.length % 5 === 0) {
                console.log(`📈 累加ENMO: +${metrics.enmo.toFixed(4)}, 总计=${this.totalENMO.toFixed(2)}`);
            }
        }

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
            intensity: metrics.intensity
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
        this.updateHourlyStats(now, metrics.enmo, newSteps, metrics.intensity);

        // 更新当前状态
        this.currentIntensity = metrics.intensity;
        this.lastUpdateTime = now;

        // 更新图表 (节流)
        if (now - this.lastChartUpdate > this.chartUpdateInterval) {
            this.updateCharts();
            this.lastChartUpdate = now;
        }
    };

    console.log('✅ 步数检测修复补丁已应用！');
    console.log('   - 全局样本计数器:', m.globalSampleCount);
    console.log('   - 上次峰值索引:', m.lastPeakGlobalIndex);
    console.log('   - 请继续运动测试步数检测');
} else {
    console.error('❌ ActivityMonitor未初始化');
}
