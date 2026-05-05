/**
 * CWT/频域带通心率呼吸率提取器
 * 参考 calcutePP.m：
 *   RR: 0.1-0.5 Hz 带通后 FFT 找峰
 *   HR: 0.5-3.0 Hz 带通后，对 RR 高次谐波施加高斯衰减再找峰
 */
(function () {
    'use strict';

    const DEFAULTS = {
        rrLow: 0.1,
        rrHigh: 0.5,
        hrLow: 0.5,
        hrHigh: 3.0,
        harmonicBw: 0.05,
        harmonicBwGrowth: 0,
        harmonicDepth: 0.1,
        maxHarmonic: 6,
        hrMinFromRrRatio: 2.2,
        minProminenceRatio: 0.1,
        minSamples: 128,
        bandEdgeBins: 1,
        rolloffHz: 0.03,
        cascadeOrder: 2,
        voicesPerOctave: 12
    };

    function nextPow2(n) {
        return Math.pow(2, Math.ceil(Math.log2(Math.max(1, n))));
    }

    function cloneArrayLike(arr) {
        return Array.from(arr || [], v => Number.isFinite(v) ? v : 0);
    }

    function detrend(arr) {
        const n = arr.length;
        if (n <= 1) return arr.slice();

        let sumX = 0;
        let sumY = 0;
        let sumXX = 0;
        let sumXY = 0;
        for (let i = 0; i < n; i++) {
            sumX += i;
            sumY += arr[i];
            sumXX += i * i;
            sumXY += i * arr[i];
        }

        const denom = n * sumXX - sumX * sumX;
        const slope = Math.abs(denom) > 1e-12 ? (n * sumXY - sumX * sumY) / denom : 0;
        const intercept = (sumY - slope * sumX) / n;
        return arr.map((v, i) => v - (slope * i + intercept));
    }

    function hannWindow(n) {
        if (n <= 1) return [1];
        const out = new Array(n);
        for (let i = 0; i < n; i++) {
            out[i] = 0.5 - 0.5 * Math.cos(2 * Math.PI * i / (n - 1));
        }
        return out;
    }

    function applyHann(re, im, length = re.length) {
        const n = Math.min(length, re.length);
        const win = hannWindow(n);
        const outRe = re.slice();
        const outIm = im.slice();
        let energy = 0;
        for (let i = 0; i < n; i++) {
            outRe[i] *= win[i];
            outIm[i] *= win[i];
            energy += win[i] * win[i];
        }
        for (let i = n; i < outRe.length; i++) {
            outRe[i] = 0;
            outIm[i] = 0;
        }
        return { re: outRe, im: outIm, windowSumSq: energy };
    }

    function padComplex(re, im, n) {
        const outRe = new Array(n).fill(0);
        const outIm = new Array(n).fill(0);
        for (let i = 0; i < Math.min(re.length, n); i++) {
            outRe[i] = re[i];
            outIm[i] = im[i] || 0;
        }
        return { re: outRe, im: outIm };
    }

    function preprocessComplex(iData, qData) {
        const iArr = detrend(cloneArrayLike(iData));
        const qArr = detrend(cloneArrayLike(qData));
        const nSamples = Math.min(iArr.length, qArr.length);
        const nFft = nextPow2(nSamples);
        const padded = padComplex(iArr.slice(0, nSamples), qArr.slice(0, nSamples), nFft);
        const windowed = applyHann(padded.re, padded.im, nSamples);
        return { ...windowed, nSamples, nFft };
    }

    function fftComplex(reInput, imInput, inverse = false) {
        const n = reInput.length;
        const re = reInput.slice();
        const im = imInput.slice();

        let j = 0;
        for (let i = 1; i < n; i++) {
            let bit = n >> 1;
            while (j & bit) {
                j ^= bit;
                bit >>= 1;
            }
            j ^= bit;
            if (i < j) {
                [re[i], re[j]] = [re[j], re[i]];
                [im[i], im[j]] = [im[j], im[i]];
            }
        }

        for (let len = 2; len <= n; len <<= 1) {
            const angle = (inverse ? 2 : -2) * Math.PI / len;
            const wLenRe = Math.cos(angle);
            const wLenIm = Math.sin(angle);
            for (let i = 0; i < n; i += len) {
                let wRe = 1;
                let wIm = 0;
                for (let k = 0; k < len / 2; k++) {
                    const even = i + k;
                    const odd = even + len / 2;
                    const uRe = re[even];
                    const uIm = im[even];
                    const vRe = re[odd] * wRe - im[odd] * wIm;
                    const vIm = re[odd] * wIm + im[odd] * wRe;
                    re[even] = uRe + vRe;
                    im[even] = uIm + vIm;
                    re[odd] = uRe - vRe;
                    im[odd] = uIm - vIm;
                    const nextWRe = wRe * wLenRe - wIm * wLenIm;
                    wIm = wRe * wLenIm + wIm * wLenRe;
                    wRe = nextWRe;
                }
            }
        }

        if (inverse) {
            for (let i = 0; i < n; i++) {
                re[i] /= n;
                im[i] /= n;
            }
        }
        return { re, im };
    }

    function oneSidedSpectrum(re, im, fs, norm = 1) {
        const half = Math.floor(re.length / 2);
        const f = new Array(half);
        const amp = new Array(half);
        const scale = norm > 0 ? Math.sqrt(norm) : 1;
        for (let i = 0; i < half; i++) {
            f[i] = i * fs / re.length;
            amp[i] = Math.sqrt(re[i] * re[i] + im[i] * im[i]) / scale;
        }
        return { f, amp };
    }

    function parabolicRefine(freq, amp, idx) {
        if (idx <= 0 || idx >= amp.length - 1) {
            return { index: idx, freq: freq[idx], amp: amp[idx], offset: 0 };
        }
        const y0 = amp[idx - 1];
        const y1 = amp[idx];
        const y2 = amp[idx + 1];
        const denom = y0 - 2 * y1 + y2;
        if (!Number.isFinite(denom) || Math.abs(denom) < 1e-12) {
            return { index: idx, freq: freq[idx], amp: amp[idx], offset: 0 };
        }
        const offset = Math.max(-0.5, Math.min(0.5, 0.5 * (y0 - y2) / denom));
        const df = freq[1] - freq[0];
        const refinedAmp = y1 - 0.25 * (y0 - y2) * offset;
        return {
            index: idx,
            freq: freq[idx] + offset * df,
            amp: refinedAmp,
            offset
        };
    }

    function findPeaksProminence(freq, amp, low, high, ratio, options = {}) {
        const df = freq.length > 1 ? Math.abs(freq[1] - freq[0]) : 0;
        const edgeBins = Number.isFinite(options.edgeBins) ? options.edgeBins : 0;
        let lowSafe = low + edgeBins * df;
        let highSafe = high - edgeBins * df;
        if (lowSafe >= highSafe) {
            lowSafe = low;
            highSafe = high;
        }

        let minVal = Infinity;
        let maxVal = -Infinity;
        for (let i = 0; i < amp.length; i++) {
            if (freq[i] < lowSafe || freq[i] > highSafe) continue;
            minVal = Math.min(minVal, amp[i]);
            maxVal = Math.max(maxVal, amp[i]);
        }
        if (!Number.isFinite(minVal) || !Number.isFinite(maxVal) || maxVal <= minVal) return [];

        const minProminence = Number.isFinite(options.minProminence)
            ? options.minProminence
            : ratio * (maxVal - minVal);
        const candidates = [];
        for (let i = 1; i < amp.length - 1; i++) {
            if (freq[i] < lowSafe || freq[i] > highSafe) continue;
            if (!(amp[i] > amp[i - 1] && amp[i] >= amp[i + 1])) continue;

            let leftMin = amp[i];
            for (let j = i - 1; j >= 0; j--) {
                leftMin = Math.min(leftMin, amp[j]);
                if (amp[j] > amp[i]) break;
            }

            let rightMin = amp[i];
            for (let j = i + 1; j < amp.length; j++) {
                rightMin = Math.min(rightMin, amp[j]);
                if (amp[j] > amp[i]) break;
            }

            const keyCol = Math.max(leftMin, rightMin);
            const prominence = amp[i] - keyCol;
            if (prominence >= minProminence) {
                const refined = parabolicRefine(freq, amp, i);
                candidates.push({
                    index: i,
                    freq: refined.freq,
                    binFreq: freq[i],
                    amp: refined.amp,
                    binAmp: amp[i],
                    prominence,
                    offset: refined.offset
                });
            }
        }

        candidates.sort((a, b) => b.prominence - a.prominence || b.amp - a.amp);
        return candidates;
    }

    function summarizePeaks(peaks) {
        return (peaks || []).slice(0, 5).map(p => ({
            freq: p.freq,
            bpm: p.freq * 60,
            binFreq: p.binFreq,
            amp: p.amp,
            prominence: p.prominence
        }));
    }

    function selectRrPeak(peaks, rrHintFreq) {
        if (!peaks.length) return null;
        const top = peaks[0];
        const closeEnough = p => p.prominence >= top.prominence * 0.75;

        if (Number.isFinite(rrHintFreq) && rrHintFreq > 0) {
            const nearHint = peaks
                .filter(closeEnough)
                .sort((a, b) => Math.abs(a.freq - rrHintFreq) - Math.abs(b.freq - rrHintFreq))[0];
            if (nearHint) return nearHint;
        }

        // 静息宠物常见 RR 在 12-21 bpm 附近。只作为相近峰 tie-break，
        // 不覆盖明显更强的主峰，避免硬编码生理值。
        const preferred = peaks.find(p => p.freq >= 0.2 && p.freq <= 0.35 && closeEnough(p));
        return preferred || top;
    }

    function gaussianHarmonicSuppression(freq, amp, rrFreq, options) {
        const out = amp.slice();
        if (!Number.isFinite(rrFreq) || rrFreq <= 0) return out;
        const baseBw = options.harmonicBw;
        const growth = Number.isFinite(options.harmonicBwGrowth) ? options.harmonicBwGrowth : 0;
        for (let k = 2; k <= options.maxHarmonic; k++) {
            const harmonic = k * rrFreq;
            if (harmonic > options.hrHigh + baseBw) break;
            const bwK = baseBw + growth * (k - 2);
            const sigma = Math.max(bwK, 1e-6);
            for (let i = 0; i < out.length; i++) {
                const x = (freq[i] - harmonic) / sigma;
                const weight = 1 - (1 - options.harmonicDepth) * Math.exp(-0.5 * x * x);
                out[i] *= weight;
            }
        }
        return out;
    }

    function raisedCosineBandWeight(freq, low, high, rolloffHz) {
        const af = Math.abs(freq);
        if (af < low - rolloffHz || af > high + rolloffHz) return 0;
        if (af >= low && af <= high) return 1;
        if (af < low) {
            const x = (af - (low - rolloffHz)) / Math.max(rolloffHz, 1e-6);
            return 0.5 - 0.5 * Math.cos(Math.PI * Math.max(0, Math.min(1, x)));
        }
        const x = ((high + rolloffHz) - af) / Math.max(rolloffHz, 1e-6);
        return 0.5 - 0.5 * Math.cos(Math.PI * Math.max(0, Math.min(1, x)));
    }

    class VitalsExtractorBase {
        constructor(fs = 50, options = {}) {
            this.fs = fs;
            this.options = { ...DEFAULTS, ...options };
        }

        setSampleRate(fs) {
            if (Number.isFinite(fs) && fs > 0) this.fs = fs;
        }

        run(iData, qData, fs = this.fs, hints = {}) {
            this.setSampleRate(fs);
            const nSamples = Math.min((iData || []).length, (qData || []).length);
            if (nSamples < this.options.minSamples) {
                return this._emptyResult('样本不足');
            }

            const signal = preprocessComplex(iData, qData);
            const rrFiltered = this.bandpass(signal.re, signal.im, this.options.rrLow, this.options.rrHigh, this.fs);
            const rrSpec = this.spectrumFromTime(rrFiltered.re, rrFiltered.im, this.fs, signal.nSamples);
            const rrPeaks = findPeaksProminence(
                rrSpec.f,
                rrSpec.amp,
                this.options.rrLow,
                this.options.rrHigh,
                this.options.minProminenceRatio,
                { edgeBins: this.options.bandEdgeBins }
            );
            const rrPeak = selectRrPeak(rrPeaks, hints.rrHintFreq);
            const rrFreq = rrPeak ? rrPeak.freq : NaN;
            const rrBpm = Number.isFinite(rrFreq) ? rrFreq * 60 : NaN;

            const hrFiltered = this.bandpass(signal.re, signal.im, this.options.hrLow, this.options.hrHigh, this.fs);
            const hrSpec = this.spectrumFromTime(hrFiltered.re, hrFiltered.im, this.fs, signal.nSamples);
            const rrUsedForHarmonic = Number.isFinite(hints.rrHintFreq) && hints.rrHintFreq > 0
                ? hints.rrHintFreq
                : rrFreq;
            const hrRobustAmp = gaussianHarmonicSuppression(hrSpec.f, hrSpec.amp, rrUsedForHarmonic, this.options);
            const hrRawPeaks = findPeaksProminence(
                hrSpec.f,
                hrSpec.amp,
                this.options.hrLow,
                this.options.hrHigh,
                this.options.minProminenceRatio,
                { edgeBins: this.strictMatlab ? 0 : this.options.bandEdgeBins }
            );
            const hrSearchLow = (!this.strictMatlab && Number.isFinite(rrFreq) && rrFreq > 0)
                ? Math.max(this.options.hrLow, rrFreq * this.options.hrMinFromRrRatio)
                : this.options.hrLow;
            const hrPeaks = findPeaksProminence(
                hrSpec.f,
                hrRobustAmp,
                hrSearchLow,
                this.options.hrHigh,
                this.options.minProminenceRatio,
                { edgeBins: this.strictMatlab ? 0 : this.options.bandEdgeBins }
            );
            const hrPeak = hrPeaks.length ? hrPeaks[0] : null;
            const hrFreq = hrPeak ? hrPeak.freq : NaN;
            const hrBpm = Number.isFinite(hrFreq) ? hrFreq * 60 : NaN;

            return {
                RR_bpm: Number.isFinite(rrBpm) ? rrBpm : NaN,
                HR_bpm: Number.isFinite(hrBpm) ? hrBpm : NaN,
                RR_freq: rrFreq,
                HR_freq: hrFreq,
                RR_peak: rrPeak,
                HR_peak: hrPeak,
                method: this.methodName,
                samples: nSamples,
                fs: this.fs,
                spectrum: {
                    rr: { f: rrSpec.f, P: rrSpec.amp, peakFreq: rrFreq, peaks: rrPeaks.slice(0, 5), selectedPeak: rrPeak },
                    hr: { f: hrSpec.f, P_raw: hrSpec.amp, P_robust: hrRobustAmp, peakFreq: hrFreq, peaks: hrPeaks.slice(0, 5), rawPeaks: hrRawPeaks.slice(0, 5), selectedPeak: hrPeak }
                },
                diagnostics: {
                    rrPeaksTop: summarizePeaks(rrPeaks),
                    hrPeaksRawTop: summarizePeaks(hrRawPeaks),
                    hrPeaksRobustTop: summarizePeaks(hrPeaks),
                    rrUsedForHarmonic,
                    rrHintFreq: hints.rrHintFreq,
                    strictMatlab: !!this.strictMatlab
                },
                options: {
                    bandEdgeBins: this.options.bandEdgeBins,
                    hrLow: this.options.hrLow,
                    hrHigh: this.options.hrHigh,
                    hrSearchLow,
                    rrLow: this.options.rrLow,
                    rrHigh: this.options.rrHigh,
                    harmonicBw: this.options.harmonicBw,
                    harmonicDepth: this.options.harmonicDepth,
                    maxHarmonic: this.options.maxHarmonic,
                    hrMinFromRrRatio: this.options.hrMinFromRrRatio
                }
            };
        }

        spectrumFromTime(re, im, fs, validLength) {
            const windowed = applyHann(re, im, validLength);
            const transformed = fftComplex(windowed.re, windowed.im, false);
            return oneSidedSpectrum(transformed.re, transformed.im, fs, windowed.windowSumSq);
        }

        bandpass() {
            throw new Error('bandpass() must be implemented by subclass');
        }

        _emptyResult(reason) {
            return {
                RR_bpm: NaN,
                HR_bpm: NaN,
                RR_freq: NaN,
                HR_freq: NaN,
                method: this.methodName,
                reason,
                samples: 0,
                fs: this.fs,
                spectrum: null
            };
        }
    }

    class VitalsExtractorFreqBP extends VitalsExtractorBase {
        constructor(fs, options) {
            super(fs, options);
            this.methodName = 'freqBP';
        }

        bandpass(re, im, low, high, fs) {
            const fft = fftComplex(re, im, false);
            const n = fft.re.length;
            const order = Math.max(1, this.options.cascadeOrder | 0);
            for (let k = 0; k < n; k++) {
                const freq = k <= n / 2 ? k * fs / n : -(n - k) * fs / n;
                let weight = raisedCosineBandWeight(freq, low, high, this.options.rolloffHz);
                if (order > 1) weight = Math.pow(weight, order);
                fft.re[k] *= weight;
                fft.im[k] *= weight;
            }
            return fftComplex(fft.re, fft.im, true);
        }
    }

    class VitalsExtractorCWT extends VitalsExtractorBase {
        constructor(fs, options) {
            super(fs, options);
            this.methodName = 'cwt';
            this.strictMatlab = true;
            this.voicesPerOctave = options?.voicesPerOctave || this.options.voicesPerOctave;
            this.morletOmega0 = options?.morletOmega0 || 6;
        }

        bandpass(re, im, low, high, fs) {
            const fft = fftComplex(re, im, false);
            const n = fft.re.length;
            const accRe = new Array(n).fill(0);
            const accIm = new Array(n).fill(0);
            const centers = this._logCenters(low * 0.8, high * 1.2, this.voicesPerOctave);
            const sigmaLog = Math.log(2) / Math.max(2, this.voicesPerOctave);

            centers.forEach(center => {
                for (let k = 0; k < n; k++) {
                    const freq = k <= n / 2 ? k * fs / n : -(n - k) * fs / n;
                    const af = Math.max(Math.abs(freq), 1e-9);
                    const logDistance = Math.log(af / center);
                    const waveletWeight = Math.exp(-0.5 * (logDistance / sigmaLog) ** 2);
                    const bandWeight = this.strictMatlab
                        ? (af >= low && af <= high ? 1 : 0)
                        : raisedCosineBandWeight(freq, low, high, this.options.rolloffHz);
                    const weight = waveletWeight * bandWeight / Math.sqrt(center);
                    accRe[k] += fft.re[k] * weight;
                    accIm[k] += fft.im[k] * weight;
                }
            });

            const admissibilityNorm = Math.max(1e-6, centers.length * Math.log(2) / this.voicesPerOctave);
            for (let k = 0; k < n; k++) {
                accRe[k] /= admissibilityNorm;
                accIm[k] /= admissibilityNorm;
            }
            return fftComplex(accRe, accIm, true);
        }

        _logCenters(low, high, voicesPerOctave) {
            const out = [];
            const start = Math.log2(Math.max(low, 1e-6));
            const end = Math.log2(Math.max(high, low * 1.01));
            const count = Math.max(2, Math.ceil((end - start) * voicesPerOctave) + 1);
            for (let i = 0; i < count; i++) {
                out.push(2 ** (start + (end - start) * i / (count - 1)));
            }
            return out;
        }
    }

    window.VitalsExtractor = {
        create(method = 'freqBP', fs = 50, options = {}) {
            return method === 'cwt'
                ? new VitalsExtractorCWT(fs, options)
                : new VitalsExtractorFreqBP(fs, options);
        },
        VitalsExtractorBase,
        VitalsExtractorFreqBP,
        VitalsExtractorCWT,
        utils: {
            fftComplex,
            findPeaksProminence,
            gaussianHarmonicSuppression,
            parabolicRefine,
            detrend,
            hannWindow,
            applyHann
        }
    };
})();
