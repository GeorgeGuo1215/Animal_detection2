/**
 * 毫米波雷达数据处理器 - JavaScript版本
 * 实现与Python版本相同的信号处理算法
 */

class RadarDataProcessor {
    constructor(samplingRate = 50) {
        this.fs = samplingRate;
        this.nShort = 500;   // 短数据长度
        this.NLong = 1000;   // 长数据长度
        this.vitalsOptions = {
            hrLow: 35 / 60,
            hrHigh: 260 / 60,
            rrLow: 6 / 60,
            rrHigh: 60 / 60
        };
        
        // 高通滤波器参数（简化版本）
        this.HPF_short_4_par = [
            0, -5.50164604360785e-05, -0.000170712307325988, -0.000321372765076530,
            -0.000523025797360845, -0.000744196112614598, -0.000983826634810562,
            -0.00127235081696804, -0.00160419939233453, -0.00193993867530017,
            -0.00228082848478637, -0.00264302585359132, -0.00301758574544834,
            -0.00343699236516377, -0.00390408151456911, -0.00439597535068256,
            -0.00492064436945537, -0.00544043127996703, -0.00594971362380537,
            -0.00647249773797565, -0.00699784286438748, -0.00754300147293485,
            -0.00810999030682790, -0.00868059148941056, -0.00925842014653436,
            -0.00988031890708563, -0.0105471202298266, -0.0112435445759278,
            -0.0119790033062446, -0.0127275069499252, -0.0134874967666727,
            -0.0142798607935558, -0.0150989968242221, -0.0159037541140797,
            -0.0166924941429137, -0.0174807050035277, -0.0182572362317448,
            -0.0190510330770520, -0.0198629881240401, -0.0206687276426293,
            -0.0214738649139352, -0.0223007879891433, -0.0231482012320617,
            -0.0240087850093419, -0.0248875902870985, -0.0257644553582338,
            -0.0266380712009247, -0.0275223504264536
        ];
        
        this.HPF_short_5_par = [
            -0.00339276769147847, -0.000289899591150812, -0.000301275418827859,
            -0.000312098312620977, -0.000322549062240940, -0.000332381770852629,
            -0.000341731680987501, -0.000350400526126641, -0.000358525665298154,
            -0.000365887160504275, -0.000372637078486777, -0.000378527224152246,
            -0.000383744164044131, -0.000388006414911702, -0.000391501840818984,
            -0.000393924679427624, -0.000395478711351671, -0.000395850492619180,
            -0.000395247988442467, -0.000393338704009786, -0.000390363416943796,
            -0.000385999580496542, -0.000380553095135587, -0.000373688291692988,
            -0.000365770714463747, -0.000356439027816037, -0.000346128807188419,
            -0.000334416670053275, -0.000321755744087934, -0.000307592555574287,
            -0.000292389547082744, -0.000275452034383983, -0.000257331448332866,
            -0.000237239705447390, -0.000216136464554341, -0.000193292824624094,
            -0.000170553182210679, -0.000146675220298725, -0.000123762543021994,
            -9.27732390070040e-05, -6.62743648944459e-05, -3.81338877599648e-05,
            -7.62116480334422e-06, 2.26657868080909e-05, 5.46318749383660e-05
        ];
    }

    /**
     * 解析数据文件内容
     * @param {string} fileContent - 文件内容
     * @returns {Object} 解析后的数据 {timestamps, iData, qData}
     */
    parseDataFile(fileContent) {
        const lines = fileContent.split('\n').filter(line => line.trim());
        const timestamps = [];
        const iData = [];
        const qData = [];
        let rejectedRows = 0;

        const parseStrictNumber = value => {
            if (typeof value !== 'string' || !value.trim()) return NaN;
            const normalized = value.trim();
            if (!/^[+-]?(?:\d+\.?\d*|\.\d+)(?:e[+-]?\d+)?$/i.test(normalized)) return NaN;
            return Number(normalized);
        };
        const appendSample = (timestamp, iText, qText) => {
            const iVal = parseStrictNumber(iText);
            const qVal = parseStrictNumber(qText);
            // Real boards use low-voltage I/Q. A generous bound keeps legacy
            // signed recordings while rejecting concatenated packet artefacts.
            if (!Number.isFinite(iVal) || !Number.isFinite(qVal)
                || Math.abs(iVal) >= 1000 || Math.abs(qVal) >= 1000) {
                rejectedRows += 1;
                return;
            }
            timestamps.push(timestamp);
            iData.push(iVal);
            qData.push(qVal);
        };

        for (const line of lines) {
            const trimmed = line.trim();
            if (trimmed.startsWith('$') && trimmed.includes(',')) {
                if (/^\$AT[+,*]/i.test(trimmed)) {
                    rejectedRows += 1;
                    continue;
                }
                const csv = trimmed.replace(/^\$/, '').replace(/\*\s*$/, '').split(',');
                if (csv.length === 16) {
                    appendSample(csv[1], csv[14], csv[15]);
                } else {
                    rejectedRows += 1;
                }
                continue;
            }

            const parts = trimmed.split(/\s+/);
            if (parts.length === 4) {
                // 格式: 日期 时间 I Q
                const ts = `${parts[0]} ${parts[1]}`;
                appendSample(ts, parts[2], parts[3]);
            } else if (parts.length === 3) {
                // 格式: 时间 I Q
                appendSample(parts[0], parts[1], parts[2]);
            } else {
                rejectedRows += 1;
            }
        }

        const estimatedSampleRate = this.estimateSampleRateFromTimestamps(timestamps);

        return {
            timestamps,
            iData: new Float64Array(iData),
            qData: new Float64Array(qData),
            length: iData.length,
            estimatedSampleRate,
            rejectedRows
        };
    }

    /**
     * Estimate the recording rate without assuming that every repository file
     * was produced by the current 50 Hz firmware. Second-resolution files use
     * median samples/complete second; millisecond timestamps use median delta.
     */
    estimateSampleRateFromTimestamps(timestamps) {
        if (!Array.isArray(timestamps) || timestamps.length < 2) return NaN;
        const toEpochMs = value => {
            if (typeof value === 'number' && Number.isFinite(value)) return value;
            const text = String(value || '').trim();
            if (/^\d{13,16}$/.test(text)) {
                const numeric = Number(text);
                if (Number.isFinite(numeric)) return numeric;
            }
            const legacy = text.match(/^(\d{4})-(\d{2})-(\d{2})-(\d{2})-(\d{2})-(\d{2})(?:[.-](\d{1,3}))?$/);
            if (legacy) {
                return new Date(
                    Number(legacy[1]), Number(legacy[2]) - 1, Number(legacy[3]),
                    Number(legacy[4]), Number(legacy[5]), Number(legacy[6]),
                    Number((legacy[7] || '0').padEnd(3, '0'))
                ).getTime();
            }
            const parsed = Date.parse(text.replace(' ', 'T'));
            return Number.isFinite(parsed) ? parsed : NaN;
        };
        const times = timestamps.map(toEpochMs).filter(Number.isFinite);
        if (times.length < 2) return NaN;
        const positiveDeltas = [];
        for (let i = 1; i < times.length; i++) {
            const delta = times[i] - times[i - 1];
            if (delta > 0 && delta < 10000) positiveDeltas.push(delta);
        }
        // Sub-second timestamps should have a positive delta for most samples.
        if (positiveDeltas.length >= Math.max(3, times.length * 0.6)) {
            const delta = this.median(positiveDeltas);
            return delta > 0 ? 1000 / delta : NaN;
        }

        const counts = new Map();
        for (const time of times) {
            const second = Math.floor(time / 1000);
            counts.set(second, (counts.get(second) || 0) + 1);
        }
        const perSecond = Array.from(counts.values()).filter(count => count > 0);
        return perSecond.length ? this.median(perSecond) : NaN;
    }

    /**
     * 圆拟合算法 - 校正I/Q信号的直流偏移和幅度不平衡
     * @param {Float64Array} iData - I通道数据
     * @param {Float64Array} qData - Q通道数据
     * @returns {Object} {center: [i_center, q_center], radius: number}
     */
    circleFitting(iData, qData) {
        const lenIQ = iData.length;
        
        // 计算均值
        const iMean = this.mean(iData);
        const qMean = this.mean(qData);
        
        // 计算矩阵元素
        let Mxx = 0, Myy = 0, Mxy = 0, Mxz = 0, Myz = 0, Mzz = 0;
        
        for (let i = 0; i < lenIQ; i++) {
            const Xi = iData[i] - iMean;
            const Yi = qData[i] - qMean;
            const Zi = Xi * Xi + Yi * Yi;
            
            Mxy += Xi * Yi;
            Mxx += Xi * Xi;
            Myy += Yi * Yi;
            Mxz += Xi * Zi;
            Myz += Yi * Zi;
            Mzz += Zi * Zi;
        }
        
        // 归一化
        Mxx /= lenIQ;
        Myy /= lenIQ;
        Mxy /= lenIQ;
        Mxz /= lenIQ;
        Myz /= lenIQ;
        Mzz /= lenIQ;
        
        const Mz = Mxx + Myy;
        const CovXY = Mxx * Myy - Mxy * Mxy;
        const Mxz2 = Mxz * Mxz;
        const Myz2 = Myz * Myz;
        
        // 求解三次方程
        const A2 = 4 * CovXY - 3 * Mz * Mz - Mzz;
        const A1 = Mzz * Mz + 4 * CovXY * Mz - Mxz2 - Myz2 - Mz * Mz * Mz;
        const A0 = Mxz2 * Myy + Myz2 * Mxx - Mzz * CovXY - 2 * Mxz * Myz * Mxy + Mz * Mz * CovXY;
        const A22 = A2 + A2;
        
        // 牛顿迭代法求解
        const epsilon = 1e-12;
        let yNew = 1e+20;
        const iterMax = 40;
        let xNew = 0;
        
        for (let iter = 0; iter < iterMax; iter++) {
            const yOld = yNew;
            yNew = A0 + xNew * (A1 + xNew * (A2 + 4 * xNew * xNew));
            
            if (Math.abs(yNew) > Math.abs(yOld)) {
                xNew = 0;
                break;
            }
            
            const Dy = A1 + xNew * (A22 + 16 * xNew * xNew);
            const xOld = xNew;
            xNew = xOld - yNew / Dy;
            
            if (Math.abs((xNew - xOld) / xNew) < epsilon) {
                break;
            }
            
            if (xNew < 0) {
                xNew = 0;
                break;
            }
        }
        
        // 计算圆心和半径
        const det = xNew * xNew - xNew * Mz + CovXY;
        let center, radius;
        
        if (Math.abs(det) > 1e-10) {
            center = [
                (Mxz * (Myy - xNew) - Myz * Mxy) / det / 2 + iMean,
                (Myz * (Mxx - xNew) - Mxz * Mxy) / det / 2 + qMean
            ];
            
            const centerOffset = [center[0] - iMean, center[1] - qMean];
            radius = Math.sqrt(centerOffset[0] * centerOffset[0] + centerOffset[1] * centerOffset[1] + Mz + 2 * xNew);
        } else {
            // 简化处理：使用均值作为圆心
            center = [iMean, qMean];
            radius = Math.sqrt(this.variance(iData) + this.variance(qData));
        }
        
        return { center, radius };
    }

    /**
     * ARCSIN相位解调算法
     * @param {Float64Array} iData - I通道数据
     * @param {Float64Array} qData - Q通道数据
     * @param {Array} center - 圆心坐标
     * @param {number} radius - 圆半径
     * @returns {Float64Array} 解调后的相位数据
     */
    arcsinDemodulation(iData, qData, center, radius) {
        const dataLength = iData.length;
        const phaseData = new Float64Array(dataLength);
        
        // 归一化I/Q数据
        const iNorm = new Float64Array(dataLength);
        const qNorm = new Float64Array(dataLength);
        
        for (let i = 0; i < dataLength; i++) {
            iNorm[i] = (iData[i] - center[0]) / radius;
            qNorm[i] = (qData[i] - center[1]) / radius;
        }
        
        // ARCSIN解调
        for (let i = 2; i < dataLength; i++) {
            const i2 = iNorm[i-1];
            const i1 = iNorm[i];
            const q2 = qNorm[i-1];
            const q1 = qNorm[i];
            
            // 防止除零错误
            const denominator = Math.sqrt((i2*i2 + q2*q2) * (i1*i1 + q1*q1));
            
            if (denominator > 1e-10) {
                const argument = (i2 * q1 - i1 * q2) / denominator;
                const clampedArg = Math.max(-1, Math.min(1, argument));
                const phaseIncrement = Math.asin(clampedArg);
                phaseData[i] = phaseData[i-1] + phaseIncrement;
            } else {
                phaseData[i] = phaseData[i-1];
            }
        }
        
        return phaseData;
    }

    /**
     * 滑动平均滤波
     * @param {Float64Array} data - 输入数据
     * @param {number} windowSize - 窗口大小
     * @returns {Float64Array} 滤波后的数据
     */
    movingAverage(data, windowSize) {
        const result = new Float64Array(data.length);
        const halfWindow = Math.floor(windowSize / 2);
        
        for (let i = 0; i < data.length; i++) {
            let sum = 0;
            let count = 0;
            
            const start = Math.max(0, i - halfWindow);
            const end = Math.min(data.length - 1, i + halfWindow);
            
            for (let j = start; j <= end; j++) {
                sum += data[j];
                count++;
            }
            
            result[i] = sum / count;
        }
        
        return result;
    }

    /**
     * 卷积运算
     * @param {Float64Array} signal - 信号数据
     * @param {Array} kernel - 卷积核
     * @returns {Float64Array} 卷积结果
     */
    convolve(signal, kernel) {
        const result = new Float64Array(signal.length);
        const kernelCenter = Math.floor(kernel.length / 2);
        
        for (let i = 0; i < signal.length; i++) {
            let sum = 0;
            
            for (let j = 0; j < kernel.length; j++) {
                const signalIndex = i - kernelCenter + j;
                if (signalIndex >= 0 && signalIndex < signal.length) {
                    sum += signal[signalIndex] * kernel[j];
                }
            }
            
            result[i] = sum;
        }
        
        return result;
    }

    /**
     * 应用数字滤波器 (参考main.py的滤波逻辑)
     * @param {Float64Array} phaseData - 相位数据
     * @returns {Object} {respiratoryWave, heartbeatWave}
     */
    applyFilters(phaseData) {
        try {
            // 呼吸波形提取 (参考main.py第245-246行和第177-179行)
            // 使用滑动平均滤波，第一次5，第二次2.5
            let respiratoryWave = this.movingAverage(Array.from(phaseData), 5);
            respiratoryWave = this.movingAverage(respiratoryWave, 2.5);
            
            // 去除最小值，类似于main.py第179行
            const minVal = Math.min(...respiratoryWave);
            respiratoryWave = respiratoryWave.map(val => val - minVal);
            
            // 心跳波形提取 (参考main.py第181-182行)
            // 使用高通滤波器，类似于np.convolve
            let heartbeatWave = this.convolve(Array.from(phaseData), this.HPF_short_4_par);
            heartbeatWave = this.convolve(heartbeatWave, this.HPF_short_5_par);
            
            return { 
                respiratoryWave: new Float64Array(respiratoryWave), 
                heartbeatWave: new Float64Array(heartbeatWave) 
            };
            
        } catch (error) {
            console.error('滤波处理错误:', error);
            // 返回原始数据作为备用
            return { 
                respiratoryWave: phaseData, 
                heartbeatWave: phaseData 
            };
        }
    }

    /**
     * FFT变换（使用自定义FFT实现）
     * @param {Float64Array} data - 输入数据
     * @returns {Array} FFT结果幅度
     */
    fft(data) {
        // 使用实数FFT
        return FFT.realFFT(Array.from(data));
    }

    /**
     * 滑动窗口分析 - 提取时间序列的心率和呼吸频率
     * @param {Float64Array} iData - I通道数据
     * @param {Float64Array} qData - Q通道数据
     * @param {Float64Array} phaseData - 相位数据
     * @param {number} windowSize - 窗口大小（秒）
     * @param {number} stepSize - 步长（秒）
     * @returns {Object} {heartRateTimeSeries, respiratoryRateTimeSeries, timeAxis}
     */
    extractVitalSignsTimeSeries(iData, qData, phaseData, windowSize = 10, stepSize = 1) {
        const windowSamples = Math.floor(windowSize * this.fs);
        const stepSamples = Math.floor(stepSize * this.fs);
        
        const heartRateTimeSeries = [];
        const respiratoryRateTimeSeries = [];
        const timeAxis = [];
        
        // 滑动窗口分析
        for (let start = 0; start + windowSamples <= iData.length; start += stepSamples) {
            const end = start + windowSamples;
            
            // 提取窗口数据
            const windowI = iData.slice(start, end);
            const windowQ = qData.slice(start, end);
            const windowPhase = phaseData ? phaseData.slice(start, end) : null;
            
            // 计算该窗口的心率和呼吸频率
            const vitalSigns = this.extractVitalSignsWindow(windowI, windowQ, windowPhase);
            
            heartRateTimeSeries.push(vitalSigns.heartRate);
            respiratoryRateTimeSeries.push(vitalSigns.respiratoryRate);
            timeAxis.push(start / this.fs); // 时间轴（秒）
        }
        
        const validHeartRates = heartRateTimeSeries.filter(Number.isFinite);
        const validRespiratoryRates = respiratoryRateTimeSeries.filter(Number.isFinite);
        return {
            heartRateTimeSeries,
            respiratoryRateTimeSeries,
            timeAxis,
            // 保持兼容性，返回平均值
            heartRate: validHeartRates.length > 0
                ? validHeartRates.reduce((a, b) => a + b, 0) / validHeartRates.length
                : NaN,
            respiratoryRate: validRespiratoryRates.length > 0
                ? validRespiratoryRates.reduce((a, b) => a + b, 0) / validRespiratoryRates.length
                : NaN
        };
    }

    _getVitalsExtractorApi() {
        if (this._vitalsExtractorApi) return this._vitalsExtractorApi;
        if (typeof window !== 'undefined' && window.VitalsExtractor) return window.VitalsExtractor;
        if (typeof globalThis !== 'undefined' && globalThis.VitalsExtractor) return globalThis.VitalsExtractor;
        // Allow the same processor to be used by Node-based offline tools and
        // regression tests. Browser builds do not expose require().
        if (typeof module !== 'undefined' && module.exports && typeof require === 'function') {
            try {
                this._vitalsExtractorApi = require('./cwt-vitals.js');
                return this._vitalsExtractorApi;
            } catch (_) {
                // Return null below; callers must emit an invalid result rather
                // than silently falling back to the old fixed-bin estimator.
            }
        }
        return null;
    }

    _extractWithVitalsApi(iData, qData, method = 'freqBP') {
        const api = this._getVitalsExtractorApi();
        if (!api || typeof api.create !== 'function') return null;
        const extractor = api.create(method, this.fs, this.vitalsOptions);
        const result = extractor.run(iData, qData, this.fs, {});
        return {
            heartRate: result.HR_bpm,
            respiratoryRate: result.RR_bpm,
            heartRateConfidence: result.HR_confidence || 0,
            respiratoryRateConfidence: result.RR_confidence || 0,
            quality: result.quality,
            enhancedResult: result
        };
    }

    /**
     * 单个窗口的生理参数提取
     * @param {Float64Array} iData - I通道数据
     * @param {Float64Array} qData - Q通道数据
     * @param {Float64Array} phaseData - 相位数据
     * @returns {Object} {heartRate, respiratoryRate}
     */
    extractVitalSignsWindow(iData, qData, phaseData) {
        const enhanced = this._extractWithVitalsApi(iData, qData, 'freqBP');
        if (enhanced) return enhanced;

        return {
            heartRate: NaN,
            respiratoryRate: NaN,
            heartRateConfidence: 0,
            respiratoryRateConfidence: 0,
            reason: 'VitalsExtractor未加载'
        };

        // 去除直流分量
        const u1 = this.removeDC(iData);
        const u2 = this.removeDC(qData);
        
        // 构造复数信号并执行FFT
        const complexSignal = [];
        for (let i = 0; i < u1.length; i++) {
            complexSignal.push([u1[i], u2[i]]);
        }
        
        // 零填充提高频率分辨率
        const paddedLength = Math.pow(2, Math.ceil(Math.log2(u1.length)));
        while (complexSignal.length < paddedLength) {
            complexSignal.push([0, 0]);
        }
        
        // FFT分析
        const fftResult = FFT.fft(complexSignal);
        const magnitude = fftResult.map(([real, imag]) => Math.sqrt(real * real + imag * imag));
        
        // 取前半部分频谱
        const halfSpectrum = magnitude.slice(0, Math.floor(magnitude.length / 4));
        
        // 频率轴
        const freqAxis = halfSpectrum.map((_, i) => i * this.fs / (halfSpectrum.length * 4));
        
        // 呼吸频率范围 (0.1-0.5 Hz, 对应6-30 bpm)
        const breathFreqIndices = freqAxis.map((freq, i) => 
            freq >= 0.1 && freq <= 0.5 ? i : -1
        ).filter(i => i !== -1);
        
        // 心率频率范围 (0.8-3.0 Hz, 对应48-180 bpm)
        const heartFreqIndices = freqAxis.map((freq, i) => 
            freq >= 0.8 && freq <= 3.0 ? i : -1
        ).filter(i => i !== -1);
        
        // 找到峰值频率
        let respiratoryRate = 0;
        let heartRate = 0;
        
        if (breathFreqIndices.length > 0) {
            const breathSpectrum = breathFreqIndices.map(i => halfSpectrum[i]);
            
            // 使用改进的峰值检测
            const breathPeaks = this.findPeaks(breathSpectrum, 2);
            
            if (breathPeaks.length > 0) {
                // 选择幅值最大的峰值
                let maxPeakIdx = 0;
                let maxPeakVal = breathSpectrum[breathPeaks[0]];
                
                for (let i = 1; i < breathPeaks.length; i++) {
                    if (breathSpectrum[breathPeaks[i]] > maxPeakVal) {
                        maxPeakVal = breathSpectrum[breathPeaks[i]];
                        maxPeakIdx = i;
                    }
                }
                
                const breathFreqHz = freqAxis[breathFreqIndices[breathPeaks[maxPeakIdx]]];
                respiratoryRate = Math.round(breathFreqHz * 60);
            } else {
                // 回退到全局最大值
                const maxBreathIdx = breathSpectrum.indexOf(Math.max(...breathSpectrum));
                const breathFreqHz = freqAxis[breathFreqIndices[maxBreathIdx]];
                respiratoryRate = Math.round(breathFreqHz * 60);
            }
        }
        
        if (heartFreqIndices.length > 0) {
            const heartSpectrum = heartFreqIndices.map(i => halfSpectrum[i]);
            
            // 改进的峰值检测：使用局部最大值而不是全局最大值
            // 避免噪声干扰，找到最显著的峰值
            const peaks = this.findPeaks(heartSpectrum, 3); // 最小距离3个点
            
            if (peaks.length > 0) {
                // 选择幅值最大的峰值
                let maxPeakIdx = 0;
                let maxPeakVal = heartSpectrum[peaks[0]];
                
                for (let i = 1; i < peaks.length; i++) {
                    if (heartSpectrum[peaks[i]] > maxPeakVal) {
                        maxPeakVal = heartSpectrum[peaks[i]];
                        maxPeakIdx = i;
                    }
                }
                
                const heartFreqHz = freqAxis[heartFreqIndices[peaks[maxPeakIdx]]];
                heartRate = Math.round(heartFreqHz * 60);
            } else {
                // 回退到全局最大值
                const maxHeartIdx = heartSpectrum.indexOf(Math.max(...heartSpectrum));
                const heartFreqHz = freqAxis[heartFreqIndices[maxHeartIdx]];
                heartRate = Math.round(heartFreqHz * 60);
            }
        }
        
        return { heartRate, respiratoryRate };
    }

    /**
     * 查找局部峰值
     * @param {Array} data - 输入数据
     * @param {number} minDistance - 峰值之间的最小距离
     * @returns {Array} 峰值索引数组
     */
    findPeaks(data, minDistance = 1) {
        const peaks = [];
        
        for (let i = 1; i < data.length - 1; i++) {
            // 检查是否为局部最大值
            if (data[i] > data[i - 1] && data[i] > data[i + 1]) {
                // 检查与前一个峰值的距离
                if (peaks.length === 0 || i - peaks[peaks.length - 1] >= minDistance) {
                    peaks.push(i);
                } else {
                    // 如果距离太近，保留幅值更大的那个
                    if (data[i] > data[peaks[peaks.length - 1]]) {
                        peaks[peaks.length - 1] = i;
                    }
                }
            }
        }
        
        return peaks;
    }

    /**
     * 参考main.py的频谱分析方法提取生理参数
     * @param {Float64Array} iData - I通道数据
     * @param {Float64Array} qData - Q通道数据
     * @param {Float64Array} phaseData - 相位数据
     * @returns {Object} {heartRate, respiratoryRate}
     */
    extractVitalSigns(iData, qData, phaseData) {
        try {
            const enhanced = this._extractWithVitalsApi(iData, qData, 'freqBP');
            if (enhanced) return enhanced;

            return {
                heartRate: NaN,
                respiratoryRate: NaN,
                heartRateConfidence: 0,
                respiratoryRateConfidence: 0,
                reason: 'VitalsExtractor未加载'
            };

            // 参考main.py第268-282行的频谱分析方法
            const u1 = this.removeDC(iData);
            const u2 = this.removeDC(qData);
            
            // 构造复数信号 (参考main.py第271行)
            const complexSignal = [];
            const dataLength = Math.min(u1.length, u2.length);
            
            for (let i = 0; i < dataLength; i++) {
                complexSignal.push([u1[i], u2[i]]);
            }
            
            // 零填充到合适长度 (参考main.py第272行)
            const paddedLength = dataLength + dataLength; // 类似于np.append(new, np.zeros(N))
            while (complexSignal.length < paddedLength) {
                complexSignal.push([0, 0]);
            }
            
            // FFT分析
            const fftResult = FFT.fft(complexSignal);
            const magnitude = fftResult.map(([real, imag]) => Math.sqrt(real * real + imag * imag));
            
            // 取前半部分频谱，限制到合理的频率范围
            const halfLength = Math.floor(magnitude.length / 2);
            const spectrumLength = Math.min(halfLength, 512); // 限制频谱长度
            const spectrum = magnitude.slice(0, spectrumLength);
            
            // 计算频率分辨率
            const sampleRate = this.fs || 100; // 默认100Hz采样率
            const freqResolution = sampleRate / magnitude.length; // 频率分辨率 = 采样率 / FFT长度
            
            console.log(`FFT参数: 数据长度=${dataLength}, FFT长度=${magnitude.length}, 频谱长度=${spectrumLength}, 频率分辨率=${freqResolution.toFixed(4)}Hz`);
            
            // 呼吸频率范围：6-30 bpm = 0.1-0.5 Hz
            const breathStartIdx = Math.max(1, Math.round(0.1 / freqResolution));
            const breathEndIdx = Math.min(spectrumLength-1, Math.round(0.5 / freqResolution));
            const breathSpectrum = spectrum.slice(breathStartIdx, breathEndIdx + 1);
            
            // 心率频率范围：48-180 bpm = 0.8-3.0 Hz  
            const heartStartIdx = Math.max(1, Math.round(0.8 / freqResolution));
            const heartEndIdx = Math.min(spectrumLength-1, Math.round(3.0 / freqResolution));
            const heartSpectrum = spectrum.slice(heartStartIdx, heartEndIdx + 1);
            
            // 调试输出
            console.log(`频谱分析: 总长度=${spectrumLength}, 呼吸范围=[${breathStartIdx}, ${breathEndIdx}] (${(breathStartIdx*freqResolution).toFixed(3)}-${(breathEndIdx*freqResolution).toFixed(3)}Hz), 心率范围=[${heartStartIdx}, ${heartEndIdx}] (${(heartStartIdx*freqResolution).toFixed(3)}-${(heartEndIdx*freqResolution).toFixed(3)}Hz)`);
            console.log(`频谱峰值: 呼吸频段最大=${Math.max(...breathSpectrum).toFixed(2)}, 心率频段最大=${Math.max(...heartSpectrum).toFixed(2)}`);
            
            // 找到峰值频率
            let respiratoryRate = 0;
            let heartRate = 0;
            
            if (breathSpectrum.length > 0) {
                const maxBreathIdx = breathSpectrum.indexOf(Math.max(...breathSpectrum));
                const actualFreqIdx = breathStartIdx + maxBreathIdx;
                const breathFreqHz = actualFreqIdx * freqResolution;
                respiratoryRate = Math.round(breathFreqHz * 60); // 转换为bpm
                console.log(`呼吸检测: 相对索引=${maxBreathIdx}, 绝对索引=${actualFreqIdx}, 频率=${breathFreqHz.toFixed(3)}Hz, 呼吸率=${respiratoryRate}bpm`);
            }
            
            if (heartSpectrum.length > 0) {
                const maxHeartIdx = heartSpectrum.indexOf(Math.max(...heartSpectrum));
                const actualFreqIdx = heartStartIdx + maxHeartIdx;
                const heartFreqHz = actualFreqIdx * freqResolution;
                heartRate = Math.round(heartFreqHz * 60); // 转换为bpm
                console.log(`心率检测: 相对索引=${maxHeartIdx}, 绝对索引=${actualFreqIdx}, 频率=${heartFreqHz.toFixed(3)}Hz, 心率=${heartRate}bpm`);
            }
            
            // 放宽合理性检查范围，避免过早使用默认值
            if (respiratoryRate < 5 || respiratoryRate > 40) {
                console.log(`呼吸率${respiratoryRate}超出范围，使用默认值`);
                respiratoryRate = NaN;
            }
            
            if (heartRate < 40 || heartRate > 200) {
                console.log(`心率${heartRate}超出范围，使用默认值`);
                heartRate = NaN;
            }
            
            console.log(`最终结果: 心率=${heartRate}bpm, 呼吸率=${respiratoryRate}bpm`);
            return { heartRate, respiratoryRate };
            
        } catch (error) {
            console.error('提取生理参数错误:', error);
            return { heartRate: NaN, respiratoryRate: NaN, reason: error.message };
        }
    }

    /**
     * 基于 main.py 风格的HR/RR提取与波形生成
     * - 相位累计：使用ARCSIN增量累加（参考main.py 460-466）
     * - 频谱法：对相位微分做滑动平均后FFT，按固定区间映射到bpm（参考main.py 214-223）
     * - 波形：呼吸=对相位做两次滑动平均并去最小值；心跳=对相位做两级高通卷积
     */
    extractVitalSignsMainPy(iData, qData) {
        const dataLength = Math.min(iData.length, qData.length);
        if (dataLength < 128) {
            return {
                heartRate: NaN,
                respiratoryRate: NaN,
                heartRateConfidence: 0,
                respiratoryRateConfidence: 0,
                reason: '样本不足',
                phase: new Float64Array(dataLength),
                respiratoryWave: new Float64Array(dataLength),
                heartbeatWave: new Float64Array(dataLength)
            };
        }

        const enhanced = this._extractWithVitalsApi(iData, qData, 'freqBP');
        if (enhanced) {
            const waveforms = enhanced.enhancedResult.waveforms || {};
            return {
                ...enhanced,
                phase: new Float64Array(waveforms.phase || dataLength),
                respiratoryWave: new Float64Array(waveforms.respiratory || dataLength),
                heartbeatWave: new Float64Array(waveforms.heartbeat || dataLength)
            };
        }

        return {
            heartRate: NaN,
            respiratoryRate: NaN,
            heartRateConfidence: 0,
            respiratoryRateConfidence: 0,
            reason: 'VitalsExtractor未加载',
            phase: new Float64Array(dataLength),
            respiratoryWave: new Float64Array(dataLength),
            heartbeatWave: new Float64Array(dataLength)
        };

        // 计算圆心与尺度（近似 main.py 中 crf 与 crf_sqrt）
        const iMean = this.mean(iData);
        const qMean = this.mean(qData);
        const iVar = this.variance(iData);
        const qVar = this.variance(qData);
        const crf_sqrt = Math.sqrt(iVar + qVar) || 1;
        const center = [iMean, qMean];

        // 归一化
        const iNorm = new Float64Array(dataLength);
        const qNorm = new Float64Array(dataLength);
        for (let i = 0; i < dataLength; i++) {
            iNorm[i] = (iData[i] - center[0]) / crf_sqrt;
            qNorm[i] = (qData[i] - center[1]) / crf_sqrt;
        }

        // 相位增量累计（main.py 460-466）
        const phase = new Float64Array(dataLength);
        for (let i = 2; i < dataLength; i++) {
            const i2 = iNorm[i-1];
            const i1 = iNorm[i];
            const q2 = qNorm[i-1];
            const q1 = qNorm[i];
            const denom = Math.sqrt((i2*i2 + q2*q2) * (i1*i1 + q1*q1));
            if (denom > 1e-10) {
                const arg = (i2 * q1 - i1 * q2) / denom;
                const clamped = Math.max(-1, Math.min(1, arg));
                phase[i] = phase[i-1] + Math.asin(clamped);
            } else {
                phase[i] = phase[i-1];
            }
        }

        // 呼吸波形（main.py 245-247 + 177-179，第一次5，第二次2.5）
        let respiratoryWave = this.movingAverage(Array.from(phase), 5);
        respiratoryWave = this.movingAverage(respiratoryWave, 2.5);
        const minResp = Math.min(...respiratoryWave);
        respiratoryWave = respiratoryWave.map(v => v - minResp);

        // 心跳波形（main.py 181-183）
        let heartbeatWave = this.convolve(Array.from(phase), this.HPF_short_4_par);
        heartbeatWave = this.convolve(heartbeatWave, this.HPF_short_5_par);

        // 频谱法（main.py 213-223：对相位差分做滑动平均 -> FFT）
        const moveStep = 2.5;
        const diffPhase = new Float64Array(dataLength);
        for (let i = 1; i < dataLength; i++) diffPhase[i] = phase[i] - phase[i-1];
        let dispShort = this.movingAverage(Array.from(diffPhase), moveStep);

        // FFT（与 main.py 行为接近：realFFT 后取前半部，再取前25个点）
        // 归一化差分，减小直流/漂移影响
        const meanDisp = dispShort.reduce((a,b)=>a+b,0)/dispShort.length;
        const stdDisp = Math.sqrt(dispShort.reduce((s,v)=>s+(v-meanDisp)*(v-meanDisp),0)/dispShort.length) || 1;
        const dispNorm = dispShort.map(v => (v-meanDisp)/stdDisp);
        const ps = FFT.realFFT(Array.from(dispNorm));
        const halfLen = Math.floor(ps.length / 2);
        const spectrum = ps.slice(0, Math.min(halfLen, 50));
        const shortSpec = spectrum.slice(0, Math.min(25, spectrum.length));

        // main.py：bre_slice_short = [2:4]，heart_slice_short = [9:20]
        let respiratoryRate = NaN;
        let heartRate = NaN;
        if (shortSpec.length >= 21) {
            const breSlice = shortSpec.slice(2, 4); // 索引2,3
            const heartSlice = shortSpec.slice(9, 21); // 索引9..20

            if (breSlice.length > 0) {
                const breMaxIdx = breSlice.indexOf(Math.max(...breSlice));
                respiratoryRate = (breMaxIdx + 2) * 6; // 与 main.py 一致
            }
            if (heartSlice.length > 0) {
                const heartMaxIdx = heartSlice.indexOf(Math.max(...heartSlice));
                heartRate = (heartMaxIdx + 9) * 6; // 与 main.py 一致
            }
        }

        // 合理性范围（稍作放宽）
        if (respiratoryRate < 5 || respiratoryRate > 60) respiratoryRate = NaN;
        if (heartRate < 30 || heartRate > 300) heartRate = NaN;

        return {
            heartRate,
            respiratoryRate,
            phase,
            respiratoryWave: new Float64Array(respiratoryWave),
            heartbeatWave: new Float64Array(heartbeatWave)
        };
    }

    /**
     * 处理单个文件
     * @param {string} fileName - 文件名
     * @param {string} fileContent - 文件内容
     * @returns {Object} 处理结果
     */
    processSingleFile(fileName, fileContent) {
        try {
            // 解析数据
            const parsedData = this.parseDataFile(fileContent);
            
            if (parsedData.length === 0) {
                throw new Error('文件中没有有效数据');
            }
            if (Number.isFinite(parsedData.estimatedSampleRate)
                && parsedData.estimatedSampleRate >= 2
                && parsedData.estimatedSampleRate <= 500) {
                this.fs = parsedData.estimatedSampleRate;
            }
            
            // 圆拟合校正
            const { center, radius } = this.circleFitting(parsedData.iData, parsedData.qData);
            
            // 相位解调
            const phaseData = this.arcsinDemodulation(parsedData.iData, parsedData.qData, center, radius);
            
            // 滤波处理
            const { respiratoryWave, heartbeatWave } = this.applyFilters(phaseData);
            
            // 提取生理参数（时间序列）
            const vitalSignsTimeSeries = this.extractVitalSignsTimeSeries(
                parsedData.iData, parsedData.qData, phaseData, 20, 2
            );
            
            return {
                fileName,
                dataPoints: parsedData.length,
                samplingRate: this.fs,
                rejectedRows: parsedData.rejectedRows,
                heartRate: vitalSignsTimeSeries.heartRate, // 平均值
                respiratoryRate: vitalSignsTimeSeries.respiratoryRate, // 平均值
                // 新增时间序列数据
                heartRateTimeSeries: vitalSignsTimeSeries.heartRateTimeSeries,
                respiratoryRateTimeSeries: vitalSignsTimeSeries.respiratoryRateTimeSeries,
                timeAxis: vitalSignsTimeSeries.timeAxis,
                circleCenter: center,
                circleRadius: radius,
                timestamps: parsedData.timestamps,
                iData: parsedData.iData,
                qData: parsedData.qData,
                phaseData,
                respiratoryWave,
                heartbeatWave,
                status: 'success'
            };
            
        } catch (error) {
            return {
                fileName,
                error: error.message,
                status: 'error'
            };
        }
    }

    // 辅助函数
    mean(array) {
        return array.reduce((sum, val) => sum + val, 0) / array.length;
    }

    median(array) {
        const values = Array.from(array || []).filter(Number.isFinite).sort((a, b) => a - b);
        if (!values.length) return NaN;
        const middle = Math.floor(values.length / 2);
        return values.length % 2
            ? values[middle]
            : (values[middle - 1] + values[middle]) / 2;
    }

    variance(array) {
        const mean = this.mean(array);
        return array.reduce((sum, val) => sum + (val - mean) ** 2, 0) / array.length;
    }

    removeDC(array) {
        const mean = this.mean(array);
        return array.map(val => val - mean);
    }
}

// 导出类供其他文件使用
if (typeof module !== 'undefined' && module.exports) {
    module.exports = RadarDataProcessor;
}
