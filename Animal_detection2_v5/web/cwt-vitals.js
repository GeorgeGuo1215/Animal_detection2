/**
 * Canine millimetre-wave vital-sign extractor.
 *
 * Processing chain:
 *   robust I/Q cleanup -> circle calibration -> atan2 phase unwrap
 *   -> frequency-adaptive HR/RR filtering -> spectral/autocorrelation scoring
 *   -> confidence and signal-quality reporting.
 *
 * The public API intentionally remains compatible with the previous module:
 *   VitalsExtractor.create(method, fs, options)
 *   extractor.setSampleRate(fs)
 *   extractor.run(iData, qData, fs, hints)
 */
(function (root, factory) {
    'use strict';

    const api = factory();
    if (root) root.VitalsExtractor = api;
    if (typeof module !== 'undefined' && module.exports) module.exports = api;
})(typeof window !== 'undefined' ? window : globalThis, function () {
    'use strict';

    const DEFAULTS = {
        // Wide technical ranges. The UI can narrow these for a particular dog.
        rrLow: 0.10,             // 6 breaths/min
        rrHigh: 1.00,            // 60 breaths/min
        hrLow: 0.50,             // 30 beats/min
        hrHigh: 5.00,            // 300 beats/min
        harmonicBw: 0.045,
        harmonicBwGrowth: 0.002,
        rrFundamentalDepth: 0.02,
        harmonicDepth: 0.08,
        harmonicDepthGrowth: 0.38,
        maxHarmonic: 18,
        minProminenceRatio: 0.045,
        minSamples: 128,
        minHrCycles: 4,
        minRrCycles: 1.5,
        zeroPadFactor: 4,
        bandEdgeBins: 0,
        rolloffHz: 0.035,
        cascadeOrder: 2,
        voicesPerOctave: 12,
        minSignalStd: 1e-6,
        maxCircleResidual: 0.45,
        maxOutlierFraction: 0.20,
        // In real collar data a strong respiratory/motion component can occur
        // at roughly half the true cardiac frequency. RR settings must never
        // decide whether an HR octave candidate exists; measured RR is only
        // soft evidence after both HR hypotheses have been formed.
        octaveCorrectionBelowHz: 1.10,   // review low hypotheses through ~66 bpm
        octaveUpperMinHz: 0.90,
        octaveRatioLow: 1.72,
        octaveRatioHigh: 2.28,
        octaveToleranceOctaves: 0.10,
        octaveResolutionBins: 3,
        octaveMinUpperRawRelative: 0.20,
        octaveMinUpperProminence: 0.15,
        octaveMinUpperScore: 0.12,
        octaveMinUpperScoreRatio: 0.27,
        octaveStrongUpperRawRelative: 0.38,
        octaveStrongUpperProminence: 0.38,
        // A weak upper candidate is still enough to make a low 35-65 bpm
        // result uncertain. It is not enough, by itself, to double the result.
        octavePlausibleUpperRawRelative: 0.10,
        octavePlausibleUpperProminence: 0.08,
        octavePlausibleUpperScoreRatio: 0.10,
        // Review non-octave alternatives as well. Real collar artefacts are not
        // always exactly half-rate (for example 55 bpm motion versus 80 bpm HR).
        lowHeartReviewBelowHz: 1.10,
        lowHeartAlternativeMinHz: 1.05,
        lowHeartAlternativeSeparationHz: 0.18,
        lowHeartAlternativeMinRawRelative: 0.32,
        lowHeartAlternativeMinProminence: 0.28,
        lowHeartAlternativeMinScoreRatio: 0.45,
        crossBandMinRrConfidence: 0.10,
        crossBandHarmonicCloseness: 0.55,
        // IMU-derived local mask. Short collar movements are repaired in the
        // uniformly sampled phase series instead of invalidating the complete
        // 20/30 s window. Long/mostly dirty windows are still rejected.
        motionMaskMaxFraction: 0.45,
        motionMaskMaxRunSec: 6.0
    };

    const EPS = 1e-12;

    function clamp(value, low, high) {
        return Math.max(low, Math.min(high, value));
    }

    function nextPow2(n) {
        return Math.pow(2, Math.ceil(Math.log2(Math.max(1, n))));
    }

    function finiteArray(arr) {
        const values = Array.from(arr || [], value => Number(value));
        let firstFinite = values.findIndex(Number.isFinite);
        if (firstFinite < 0) return values.map(() => 0);

        for (let i = 0; i < firstFinite; i++) values[i] = values[firstFinite];
        let last = values[firstFinite];
        for (let i = firstFinite + 1; i < values.length; i++) {
            if (Number.isFinite(values[i])) last = values[i];
            else values[i] = last;
        }
        return values;
    }

    function mean(values) {
        if (!values.length) return NaN;
        return values.reduce((sum, value) => sum + value, 0) / values.length;
    }

    function median(values) {
        const sorted = values.filter(Number.isFinite).slice().sort((a, b) => a - b);
        if (!sorted.length) return NaN;
        const middle = Math.floor(sorted.length / 2);
        return sorted.length % 2
            ? sorted[middle]
            : (sorted[middle - 1] + sorted[middle]) / 2;
    }

    function quantile(values, q) {
        const sorted = values.filter(Number.isFinite).slice().sort((a, b) => a - b);
        if (!sorted.length) return NaN;
        const position = clamp(q, 0, 1) * (sorted.length - 1);
        const lower = Math.floor(position);
        const upper = Math.ceil(position);
        const fraction = position - lower;
        return sorted[lower] * (1 - fraction) + sorted[upper] * fraction;
    }

    function standardDeviation(values) {
        if (!values.length) return 0;
        const avg = mean(values);
        return Math.sqrt(values.reduce((sum, value) => sum + (value - avg) ** 2, 0) / values.length);
    }

    function medianAbsoluteDeviation(values, center = median(values)) {
        if (!Number.isFinite(center)) return 0;
        return 1.4826 * median(values.map(value => Math.abs(value - center)));
    }

    function winsorize(values, z = 8) {
        const center = median(values);
        let scale = medianAbsoluteDeviation(values, center);
        if (!(scale > EPS)) scale = standardDeviation(values);
        if (!(scale > EPS)) return { values: values.slice(), clippedFraction: 0 };
        const low = center - z * scale;
        const high = center + z * scale;
        let clipped = 0;
        const result = values.map(value => {
            if (value < low) {
                clipped += 1;
                return low;
            }
            if (value > high) {
                clipped += 1;
                return high;
            }
            return value;
        });
        return { values: result, clippedFraction: clipped / Math.max(1, values.length) };
    }

    function detrend(values) {
        const n = values.length;
        if (n <= 1) return values.slice();

        let sumX = 0;
        let sumY = 0;
        let sumXX = 0;
        let sumXY = 0;
        for (let i = 0; i < n; i++) {
            sumX += i;
            sumY += values[i];
            sumXX += i * i;
            sumXY += i * values[i];
        }

        const denominator = n * sumXX - sumX * sumX;
        const slope = Math.abs(denominator) > EPS
            ? (n * sumXY - sumX * sumY) / denominator
            : 0;
        const intercept = (sumY - slope * sumX) / n;
        return values.map((value, index) => value - (slope * index + intercept));
    }

    function hannWindow(n) {
        if (n <= 1) return [1];
        const window = new Array(n);
        for (let i = 0; i < n; i++) {
            window[i] = 0.5 - 0.5 * Math.cos(2 * Math.PI * i / (n - 1));
        }
        return window;
    }

    function applyHann(re, im, validLength = re.length) {
        const n = Math.min(validLength, re.length);
        const window = hannWindow(n);
        const outRe = re.slice();
        const outIm = im.slice();
        let energy = 0;
        for (let i = 0; i < n; i++) {
            outRe[i] *= window[i];
            outIm[i] *= window[i];
            energy += window[i] * window[i];
        }
        for (let i = n; i < outRe.length; i++) {
            outRe[i] = 0;
            outIm[i] = 0;
        }
        return { re: outRe, im: outIm, windowSumSq: energy };
    }

    function padReal(values, length) {
        const re = new Array(length).fill(0);
        const im = new Array(length).fill(0);
        for (let i = 0; i < Math.min(values.length, length); i++) re[i] = values[i];
        return { re, im };
    }

    function fftComplex(reInput, imInput, inverse = false) {
        const n = reInput.length;
        if (n !== imInput.length || (n & (n - 1)) !== 0) {
            throw new Error('FFT length must be a power of two');
        }
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

        for (let length = 2; length <= n; length <<= 1) {
            const angle = (inverse ? 2 : -2) * Math.PI / length;
            const stepRe = Math.cos(angle);
            const stepIm = Math.sin(angle);
            for (let offset = 0; offset < n; offset += length) {
                let weightRe = 1;
                let weightIm = 0;
                for (let k = 0; k < length / 2; k++) {
                    const even = offset + k;
                    const odd = even + length / 2;
                    const oddRe = re[odd] * weightRe - im[odd] * weightIm;
                    const oddIm = re[odd] * weightIm + im[odd] * weightRe;
                    const evenRe = re[even];
                    const evenIm = im[even];
                    re[even] = evenRe + oddRe;
                    im[even] = evenIm + oddIm;
                    re[odd] = evenRe - oddRe;
                    im[odd] = evenIm - oddIm;

                    const nextWeightRe = weightRe * stepRe - weightIm * stepIm;
                    weightIm = weightRe * stepIm + weightIm * stepRe;
                    weightRe = nextWeightRe;
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

    function oneSidedSpectrum(re, im, fs, normalization = 1) {
        const half = Math.floor(re.length / 2) + 1;
        const frequency = new Array(half);
        const amplitude = new Array(half);
        const scale = normalization > 0 ? Math.sqrt(normalization) : 1;
        for (let i = 0; i < half; i++) {
            frequency[i] = i * fs / re.length;
            amplitude[i] = Math.hypot(re[i], im[i]) / scale;
        }
        return { f: frequency, amp: amplitude };
    }

    function fitCircle(iValues, qValues) {
        const n = Math.min(iValues.length, qValues.length);
        if (n < 3) return null;
        const meanI = mean(iValues);
        const meanQ = mean(qValues);
        let mxx = 0;
        let myy = 0;
        let mxy = 0;
        let mxz = 0;
        let myz = 0;
        let mzz = 0;

        for (let index = 0; index < n; index++) {
            const x = iValues[index] - meanI;
            const y = qValues[index] - meanQ;
            const z = x * x + y * y;
            mxx += x * x;
            myy += y * y;
            mxy += x * y;
            mxz += x * z;
            myz += y * z;
            mzz += z * z;
        }
        mxx /= n;
        myy /= n;
        mxy /= n;
        mxz /= n;
        myz /= n;
        mzz /= n;

        const mz = mxx + myy;
        const covariance = mxx * myy - mxy * mxy;
        const a2 = 4 * covariance - 3 * mz * mz - mzz;
        const a1 = mzz * mz + 4 * covariance * mz - mxz * mxz - myz * myz - mz ** 3;
        const a0 = mxz * mxz * myy + myz * myz * mxx - mzz * covariance
            - 2 * mxz * myz * mxy + mz * mz * covariance;

        let x = 0;
        let previousResidual = Infinity;
        for (let iteration = 0; iteration < 40; iteration++) {
            const residual = a0 + x * (a1 + x * (a2 + 4 * x * x));
            const derivative = a1 + x * (2 * a2 + 16 * x * x);
            if (!Number.isFinite(residual) || !Number.isFinite(derivative) || Math.abs(derivative) < EPS) break;
            if (Math.abs(residual) > previousResidual) {
                x = 0;
                break;
            }
            previousResidual = Math.abs(residual);
            const next = x - residual / derivative;
            if (!Number.isFinite(next) || next < 0) {
                x = 0;
                break;
            }
            if (Math.abs(next - x) <= EPS * Math.max(1, Math.abs(next))) {
                x = next;
                break;
            }
            x = next;
        }

        const determinant = x * x - x * mz + covariance;
        if (Math.abs(determinant) < EPS) return null;
        const centerI = (mxz * (myy - x) - myz * mxy) / determinant / 2 + meanI;
        const centerQ = (myz * (mxx - x) - mxz * mxy) / determinant / 2 + meanQ;
        const radius = Math.sqrt(
            (centerI - meanI) ** 2 + (centerQ - meanQ) ** 2 + mz + 2 * x
        );
        if (![centerI, centerQ, radius].every(Number.isFinite) || radius <= EPS) return null;
        return { center: [centerI, centerQ], radius };
    }

    function pcaProjection(iValues, qValues) {
        const meanI = mean(iValues);
        const meanQ = mean(qValues);
        let varianceI = 0;
        let varianceQ = 0;
        let covariance = 0;
        for (let i = 0; i < iValues.length; i++) {
            const x = iValues[i] - meanI;
            const y = qValues[i] - meanQ;
            varianceI += x * x;
            varianceQ += y * y;
            covariance += x * y;
        }
        const angle = 0.5 * Math.atan2(2 * covariance, varianceI - varianceQ);
        const cos = Math.cos(angle);
        const sin = Math.sin(angle);
        return detrend(iValues.map((value, index) =>
            (value - meanI) * cos + (qValues[index] - meanQ) * sin
        ));
    }

    function demodulateMotion(iData, qData, options) {
        const n = Math.min((iData || []).length, (qData || []).length);
        const rawI = finiteArray(Array.from(iData || []).slice(0, n));
        const rawQ = finiteArray(Array.from(qData || []).slice(0, n));
        const cleanI = winsorize(rawI);
        const cleanQ = winsorize(rawQ);
        const inputOutlierFraction = Math.max(cleanI.clippedFraction, cleanQ.clippedFraction);
        const circle = fitCircle(cleanI.values, cleanQ.values);

        let motion;
        let circleResidual = Infinity;
        let phaseOutlierFraction = 0;
        let mode = 'pca';

        if (circle) {
            const x = cleanI.values.map(value => value - circle.center[0]);
            const y = cleanQ.values.map(value => value - circle.center[1]);
            const radii = x.map((value, index) => Math.hypot(value, y[index]));
            circleResidual = median(radii.map(radius => Math.abs(radius - circle.radius)))
                / Math.max(circle.radius, EPS);

            const increments = new Array(n).fill(0);
            for (let index = 1; index < n; index++) {
                const cross = x[index - 1] * y[index] - y[index - 1] * x[index];
                const dot = x[index - 1] * x[index] + y[index - 1] * y[index];
                increments[index] = Math.atan2(cross, dot);
            }
            const incrementCenter = median(increments.slice(1));
            let incrementScale = medianAbsoluteDeviation(increments.slice(1), incrementCenter);
            if (!(incrementScale > EPS)) incrementScale = standardDeviation(increments.slice(1));
            if (incrementScale > EPS) {
                const limit = Math.max(0.02, 8 * incrementScale);
                let clipped = 0;
                for (let index = 1; index < increments.length; index++) {
                    const delta = increments[index] - incrementCenter;
                    if (Math.abs(delta) > limit) {
                        increments[index] = incrementCenter + Math.sign(delta) * limit;
                        clipped += 1;
                    }
                }
                phaseOutlierFraction = clipped / Math.max(1, n - 1);
            }

            const phase = new Array(n).fill(0);
            for (let index = 1; index < n; index++) phase[index] = phase[index - 1] + increments[index];
            motion = detrend(phase);
            mode = 'phase';
        } else {
            motion = pcaProjection(cleanI.values, cleanQ.values);
        }

        const motionStd = standardDeviation(motion);
        const robustRange = quantile(motion, 0.95) - quantile(motion, 0.05);
        const residualScore = Number.isFinite(circleResidual)
            ? clamp(1 - circleResidual / options.maxCircleResidual, 0, 1)
            : 0.35;
        const outlierFraction = Math.max(inputOutlierFraction, phaseOutlierFraction);
        const outlierScore = clamp(1 - outlierFraction / options.maxOutlierFraction, 0, 1);
        const amplitudeScore = motionStd > options.minSignalStd ? 1 : 0;
        const score = clamp(amplitudeScore * (0.55 * residualScore + 0.45 * outlierScore), 0, 1);

        return {
            motion,
            circle,
            quality: {
                score,
                valid: amplitudeScore > 0 && outlierFraction <= options.maxOutlierFraction,
                mode,
                signalStd: motionStd,
                robustRange,
                circleResidual,
                inputOutlierFraction,
                phaseOutlierFraction,
                reason: amplitudeScore === 0
                    ? '信号近似常量'
                    : outlierFraction > options.maxOutlierFraction
                    ? '异常点过多'
                    : ''
            }
        };
    }

    function repairMaskedMotion(motion, cleanMask, fs, options) {
        const original = motion.slice();
        const diagnostic = {
            applied: false,
            dirtyFraction: 0,
            longestDirtySec: 0,
            repairedRuns: 0,
            reason: 'no-mask'
        };
        if (!Array.isArray(cleanMask) || cleanMask.length !== motion.length || !motion.length) {
            return { values: original, diagnostic, usable: true };
        }

        const clean = cleanMask.map(Boolean);
        const runs = [];
        let dirtyCount = 0;
        let start = -1;
        for (let index = 0; index <= clean.length; index++) {
            const isDirty = index < clean.length && !clean[index];
            if (isDirty) {
                dirtyCount += 1;
                if (start < 0) start = index;
            } else if (start >= 0) {
                runs.push({ start, end: index - 1 });
                start = -1;
            }
        }
        diagnostic.dirtyFraction = dirtyCount / Math.max(1, clean.length);
        diagnostic.longestDirtySec = runs.length
            ? Math.max(...runs.map(run => run.end - run.start + 1)) / Math.max(fs, EPS)
            : 0;
        if (!runs.length) {
            diagnostic.reason = 'mask-clean';
            return { values: original, diagnostic, usable: true };
        }

        const maxFraction = clamp(Number(options.motionMaskMaxFraction) || 0.45, 0.05, 0.80);
        const maxRunSec = Math.max(0.5, Number(options.motionMaskMaxRunSec) || 6);
        if (diagnostic.dirtyFraction > maxFraction || diagnostic.longestDirtySec > maxRunSec) {
            diagnostic.reason = 'too-much-motion-for-repair';
            return { values: original, diagnostic, usable: false };
        }

        const repaired = original.slice();
        for (const run of runs) {
            const left = run.start - 1;
            const right = run.end + 1;
            if (left >= 0 && right < repaired.length) {
                const span = right - left;
                const leftValue = repaired[left];
                const rightValue = repaired[right];
                for (let index = run.start; index <= run.end; index++) {
                    const fraction = (index - left) / span;
                    repaired[index] = leftValue * (1 - fraction) + rightValue * fraction;
                }
            } else if (left >= 0) {
                for (let index = run.start; index <= run.end; index++) repaired[index] = repaired[left];
            } else if (right < repaired.length) {
                for (let index = run.start; index <= run.end; index++) repaired[index] = repaired[right];
            }
            diagnostic.repairedRuns += 1;
        }
        diagnostic.applied = true;
        diagnostic.reason = 'local-motion-interpolated';
        return { values: detrend(repaired), diagnostic, usable: true };
    }

    function preprocessMotion(iData, qData, options, hints = {}, fs = 1) {
        const demodulated = demodulateMotion(iData, qData, options);
        const repair = repairMaskedMotion(
            demodulated.motion,
            hints.motionCleanMask,
            fs,
            options
        );
        demodulated.motion = repair.values;
        demodulated.quality.motionRepair = repair.diagnostic;
        if (!repair.usable) {
            demodulated.quality.valid = false;
            demodulated.quality.reason = '运动污染占比过高，保留最近可靠值';
        } else if (repair.diagnostic.applied) {
            demodulated.quality.signalStd = standardDeviation(demodulated.motion);
            demodulated.quality.robustRange = quantile(demodulated.motion, 0.95)
                - quantile(demodulated.motion, 0.05);
        }
        const nSamples = demodulated.motion.length;
        const nFft = nextPow2(Math.max(nSamples, nSamples * options.zeroPadFactor));
        return {
            ...padReal(demodulated.motion, nFft),
            nSamples,
            nFft,
            motion: demodulated.motion,
            circle: demodulated.circle,
            quality: demodulated.quality
        };
    }

    function parabolicRefine(frequency, amplitude, index) {
        if (index <= 0 || index >= amplitude.length - 1 || frequency.length < 2) {
            return { index, freq: frequency[index], amp: amplitude[index], offset: 0 };
        }
        const left = amplitude[index - 1];
        const center = amplitude[index];
        const right = amplitude[index + 1];
        const denominator = left - 2 * center + right;
        if (!Number.isFinite(denominator) || Math.abs(denominator) < EPS) {
            return { index, freq: frequency[index], amp: center, offset: 0 };
        }
        const offset = clamp(0.5 * (left - right) / denominator, -0.5, 0.5);
        const df = frequency[1] - frequency[0];
        return {
            index,
            freq: frequency[index] + offset * df,
            amp: center - 0.25 * (left - right) * offset,
            offset
        };
    }

    function bandStatistics(frequency, amplitude, low, high) {
        const values = [];
        let max = -Infinity;
        let maxIndex = -1;
        for (let index = 0; index < amplitude.length; index++) {
            if (frequency[index] < low || frequency[index] > high) continue;
            const value = amplitude[index];
            values.push(value);
            if (value > max) {
                max = value;
                maxIndex = index;
            }
        }
        const floor = median(values);
        const spread = medianAbsoluteDeviation(values, floor);
        return {
            values,
            min: values.length ? Math.min(...values) : NaN,
            max,
            maxIndex,
            floor: Number.isFinite(floor) ? floor : 0,
            spread: Number.isFinite(spread) ? spread : 0
        };
    }

    function findPeaksProminence(frequency, amplitude, low, high, ratio, options = {}) {
        if (frequency.length < 3 || amplitude.length !== frequency.length) return [];
        const df = Math.abs(frequency[1] - frequency[0]);
        const edgeBins = Number.isFinite(options.edgeBins) ? options.edgeBins : 0;
        let lowSafe = low + edgeBins * df;
        let highSafe = high - edgeBins * df;
        if (lowSafe >= highSafe) {
            lowSafe = low;
            highSafe = high;
        }
        const stats = bandStatistics(frequency, amplitude, lowSafe, highSafe);
        if (!Number.isFinite(stats.max) || !Number.isFinite(stats.min) || stats.max <= stats.min + EPS) return [];
        const minimumProminence = Number.isFinite(options.minProminence)
            ? options.minProminence
            : ratio * (stats.max - stats.min);
        const candidates = [];

        for (let index = 1; index < amplitude.length - 1; index++) {
            if (frequency[index] < lowSafe || frequency[index] > highSafe) continue;
            if (!(amplitude[index] > amplitude[index - 1] && amplitude[index] >= amplitude[index + 1])) continue;

            let leftMinimum = amplitude[index];
            for (let cursor = index - 1; cursor >= 0; cursor--) {
                leftMinimum = Math.min(leftMinimum, amplitude[cursor]);
                if (amplitude[cursor] > amplitude[index] || frequency[cursor] < lowSafe) break;
            }
            let rightMinimum = amplitude[index];
            for (let cursor = index + 1; cursor < amplitude.length; cursor++) {
                rightMinimum = Math.min(rightMinimum, amplitude[cursor]);
                if (amplitude[cursor] > amplitude[index] || frequency[cursor] > highSafe) break;
            }
            const prominence = amplitude[index] - Math.max(leftMinimum, rightMinimum);
            if (prominence < minimumProminence) continue;
            const refined = parabolicRefine(frequency, amplitude, index);
            candidates.push({
                index,
                freq: refined.freq,
                binFreq: frequency[index],
                amp: refined.amp,
                binAmp: amplitude[index],
                prominence,
                offset: refined.offset
            });
        }

        // A tone on a configured band edge can fail the local-maximum test. Keep
        // the strongest band bin as a low-priority fallback rather than inventing
        // a default physiological value.
        if (!candidates.length && stats.maxIndex > 0 && stats.maxIndex < amplitude.length - 1) {
            const refined = parabolicRefine(frequency, amplitude, stats.maxIndex);
            candidates.push({
                index: stats.maxIndex,
                freq: refined.freq,
                binFreq: frequency[stats.maxIndex],
                amp: refined.amp,
                binAmp: amplitude[stats.maxIndex],
                prominence: Math.max(0, stats.max - stats.floor),
                offset: refined.offset,
                fallback: true
            });
        }

        candidates.sort((a, b) => b.prominence - a.prominence || b.amp - a.amp);
        return candidates;
    }

    function harmonicDepthForOrder(order, options, strength = 1) {
        if (order === 1) {
            const fundamentalDepth = clamp(options.rrFundamentalDepth, 0, 1);
            return 1 - (1 - fundamentalDepth) * clamp(strength, 0, 1);
        }
        const base = clamp(options.harmonicDepth, 0, 1);
        const growth = Math.max(0, options.harmonicDepthGrowth);
        const depth = 1 - (1 - base) * Math.exp(-growth * Math.max(0, order - 2));
        // Low-confidence RR must not carve deep notches into the HR spectrum.
        return 1 - (1 - depth) * clamp(strength, 0, 1);
    }

    function gaussianHarmonicSuppression(frequency, amplitude, rrFrequency, options, strength = 1) {
        const output = amplitude.slice();
        if (!Number.isFinite(rrFrequency) || rrFrequency <= 0 || strength <= 0) return output;
        // k=1 prevents the respiration fundamental itself from being reported
        // as a low heart rate when the configured RR and HR bands overlap.
        for (let order = 1; order <= options.maxHarmonic; order++) {
            const harmonic = order * rrFrequency;
            if (harmonic > options.hrHigh + options.harmonicBw) break;
            const bandwidth = Math.max(
                1e-6,
                options.harmonicBw + options.harmonicBwGrowth * Math.max(0, order - 2)
            );
            const depth = harmonicDepthForOrder(order, options, strength);
            for (let index = 0; index < output.length; index++) {
                const distance = (frequency[index] - harmonic) / bandwidth;
                const weight = 1 - (1 - depth) * Math.exp(-0.5 * distance * distance);
                output[index] *= weight;
            }
        }
        return output;
    }

    function nearestHarmonic(frequency, rrFrequency, options) {
        if (!Number.isFinite(frequency) || !Number.isFinite(rrFrequency) || rrFrequency <= 0) {
            return { order: null, distance: Infinity, closeness: 0, depth: 1 };
        }
        let bestOrder = null;
        let bestDistance = Infinity;
        for (let order = 1; order <= options.maxHarmonic; order++) {
            const harmonic = order * rrFrequency;
            if (harmonic > options.hrHigh + options.harmonicBw) break;
            const distance = Math.abs(frequency - harmonic);
            if (distance < bestDistance) {
                bestDistance = distance;
                bestOrder = order;
            }
        }
        const bandwidth = bestOrder === null
            ? options.harmonicBw
            : options.harmonicBw + options.harmonicBwGrowth * Math.max(0, bestOrder - 2);
        return {
            order: bestOrder,
            distance: bestDistance,
            closeness: Number.isFinite(bestDistance) ? Math.exp(-0.5 * (bestDistance / bandwidth) ** 2) : 0,
            depth: bestOrder === null ? 1 : harmonicDepthForOrder(bestOrder, options, 1)
        };
    }

    function normalizedAutocorrelation(values, lag) {
        const n = values.length;
        const integerLag = Math.round(lag);
        if (integerLag < 1 || integerLag >= n - 2) return 0;
        const avg = mean(values);
        let numerator = 0;
        let leftEnergy = 0;
        let rightEnergy = 0;
        for (let index = integerLag; index < n; index++) {
            const left = values[index] - avg;
            const right = values[index - integerLag] - avg;
            numerator += left * right;
            leftEnergy += left * left;
            rightEnergy += right * right;
        }
        const denominator = Math.sqrt(leftEnergy * rightEnergy);
        return denominator > EPS ? numerator / denominator : 0;
    }

    function autocorrelationSupport(values, fs, frequency) {
        if (!Number.isFinite(frequency) || frequency <= 0 || values.length < 8) {
            return { score: 0, correlation: 0, lag: NaN, freq: NaN };
        }
        const expectedLag = fs / frequency;
        const radius = Math.max(1, Math.round(expectedLag * 0.10));
        let bestCorrelation = -Infinity;
        let bestLag = Math.round(expectedLag);
        for (let lag = Math.max(2, Math.round(expectedLag) - radius);
             lag <= Math.min(values.length - 3, Math.round(expectedLag) + radius);
             lag++) {
            const correlation = normalizedAutocorrelation(values, lag);
            if (correlation > bestCorrelation) {
                bestCorrelation = correlation;
                bestLag = lag;
            }
        }
        const agreement = Math.exp(-Math.abs(bestLag - expectedLag) / Math.max(1, expectedLag * 0.10));
        const positiveCorrelation = clamp(bestCorrelation, 0, 1);
        return {
            score: positiveCorrelation * agreement,
            correlation: Number.isFinite(bestCorrelation) ? bestCorrelation : 0,
            lag: bestLag,
            freq: bestLag > 0 ? fs / bestLag : NaN
        };
    }

    function mergePeakCandidates(rawPeaks, robustPeaks) {
        // The RR-suppressed spectrum contains artificial local maxima on both
        // shoulders of every notch. Those shoulders must never become new HR
        // hypotheses. Form candidates only from the untouched spectrum and use
        // a nearby robust peak merely as additional evidence for that raw peak.
        return rawPeaks.map(rawPeak => {
            let nearest = null;
            let nearestDistance = Infinity;
            for (const robustPeak of robustPeaks) {
                const distance = Math.abs(robustPeak.index - rawPeak.index);
                if (distance <= 1 && distance < nearestDistance) {
                    nearest = robustPeak;
                    nearestDistance = distance;
                }
            }
            return {
                ...rawPeak,
                rawPeak,
                robustPeak: nearest
            };
        });
    }

    function scorePeakCandidates({
        candidates,
        frequency,
        rawAmplitude,
        robustAmplitude,
        filteredWave,
        fs,
        low,
        high,
        hintFrequency,
        rrFrequency,
        options,
        kind
    }) {
        if (!candidates.length) return { selected: null, ranked: [] };
        const rawStats = bandStatistics(frequency, rawAmplitude, low, high);
        const robustStats = bandStatistics(frequency, robustAmplitude, low, high);
        const maxProminence = Math.max(...candidates.map(candidate =>
            Math.max(candidate.rawPeak?.prominence || 0, candidate.robustPeak?.prominence || 0)
        ), EPS);
        const ranked = candidates.map(candidate => {
            const index = clamp(candidate.index, 0, rawAmplitude.length - 1);
            const rawRelative = rawAmplitude[index] / Math.max(rawStats.max, EPS);
            const robustRelative = robustAmplitude[index] / Math.max(robustStats.max, EPS);
            const prominence = Math.max(
                candidate.rawPeak?.prominence || 0,
                candidate.robustPeak?.prominence || 0
            );
            const prominenceRelative = prominence / maxProminence;
            const autocorrelation = autocorrelationSupport(filteredWave, fs, candidate.freq);
            const hintScore = Number.isFinite(hintFrequency) && hintFrequency > 0
                ? Math.exp(-Math.abs(candidate.freq - hintFrequency) / (kind === 'hr' ? 0.65 : 0.20))
                : 0.5;
            const harmonic = kind === 'hr'
                ? nearestHarmonic(candidate.freq, rrFrequency, options)
                : { order: null, distance: Infinity, closeness: 0, depth: 1 };

            // Respiration harmonics are evidence against a candidate, not an
            // unconditional veto: a real HR can genuinely coincide with k*RR.
            const harmonicEvidenceStrength = clamp(
                Number(options.harmonicEvidenceStrength) || 0,
                0,
                1
            );
            const harmonicPenalty = kind === 'hr'
                ? (harmonic.order === 1 ? 0.55 : 0.10)
                    * harmonic.closeness * (1 - harmonic.depth)
                    * harmonicEvidenceStrength
                : 0;
            const score = 0.25 * rawRelative
                + 0.30 * robustRelative
                + 0.22 * prominenceRelative
                + 0.18 * autocorrelation.score
                + 0.05 * hintScore
                - harmonicPenalty;
            return {
                ...candidate,
                score,
                rawRelative,
                robustRelative,
                prominenceRelative,
                autocorrelation,
                hintScore,
                harmonic
            };
        }).sort((a, b) => b.score - a.score);
        return { selected: ranked[0], ranked };
    }

    /**
     * Resolve a low/upper HR pair without tying it to the configured RR upper
     * bound. A single spectrum cannot always distinguish "true 40 + 80 second
     * harmonic" from "true 80 + 40 subharmonic artefact". In that irreducible
     * case the low hypothesis is returned with `ambiguous=true` and reduced
     * confidence; callers must hold an earlier reliable value instead of
     * establishing a new, potentially wrong baseline.
     */
    function resolveHeartRateOctave(
        ranked,
        rrFrequency,
        hintFrequency,
        hintMeta,
        options,
        frequencyResolution = NaN
    ) {
        const original = ranked?.[0] || null;
        const decision = {
            applied: false,
            ambiguous: false,
            reason: 'not-evaluated',
            reasonCodes: [],
            originalBpm: original ? original.freq * 60 : NaN,
            selectedBpm: original ? original.freq * 60 : NaN,
            upperBpm: NaN,
            upperToLowerRatio: NaN,
            upperScoreRatio: NaN,
            octaveError: NaN,
            octaveTolerance: NaN,
            frequencyResolution,
            hintBpm: Number.isFinite(hintFrequency) ? hintFrequency * 60 : NaN,
            hintTrusted: false,
            lowerRrHarmonicOrder: original?.harmonic?.order ?? null,
            lowerRrCloseness: original?.harmonic?.closeness || 0,
            lowHasRespiratoryArtifactEvidence: false,
            crossBandAmbiguous: false,
            alternativeBpm: NaN,
            alternativeScoreRatio: NaN
        };
        if (!original || !Number.isFinite(original.freq) || original.freq <= 0) {
            decision.reason = 'no-heart-candidate';
            decision.reasonCodes.push(decision.reason);
            return { selected: original, decision };
        }

        const reviewBelow = Math.max(
            Number(options.octaveCorrectionBelowHz) || 0,
            Number(options.lowHeartReviewBelowHz) || 0
        );
        if (!(reviewBelow > 0) || original.freq >= reviewBelow) {
            decision.reason = 'low-candidate-not-in-octave-zone';
            decision.reasonCodes.push(decision.reason);
            return { selected: original, decision };
        }

        const hintTrusted = !!hintMeta
            && hintMeta.status === 'fresh'
            && hintMeta.ambiguous !== true
            && Number(hintMeta.confidence) >= 0.50
            && Number(hintMeta.freshWindowCount) >= 2
            && (!Number.isFinite(Number(hintMeta.ageMs)) || Number(hintMeta.ageMs) <= 10000);
        decision.hintTrusted = hintTrusted;

        const rrEvidenceConfidence = clamp(Number(options.rrEvidenceConfidence) || 0, 0, 1);
        const rrEvidenceStrength = clamp(Number(options.harmonicEvidenceStrength) || 0, 0, 1);
        const harmonicClosenessThreshold = clamp(
            Number(options.crossBandHarmonicCloseness) || 0.55,
            0,
            1
        );
        const lowHarmonicOrder = original.harmonic?.order || 0;
        const lowHarmonicCloseness = original.harmonic?.closeness || 0;
        const lowHasRespiratoryArtifactEvidence = Number.isFinite(rrFrequency)
            && rrEvidenceConfidence >= (Number(options.crossBandMinRrConfidence) || 0.10)
            && rrEvidenceStrength > 0
            && lowHarmonicOrder >= 1
            && lowHarmonicCloseness >= harmonicClosenessThreshold;
        const lowIsRespirationFundamental = lowHasRespiratoryArtifactEvidence
            && lowHarmonicOrder === 1;
        decision.lowHasRespiratoryArtifactEvidence = lowHasRespiratoryArtifactEvidence;
        decision.crossBandAmbiguous = lowIsRespirationFundamental;

        const candidateEvidence = candidate => 0.45 * (candidate.rawRelative || 0)
            + 0.30 * (candidate.prominenceRelative || 0)
            + 0.20 * (candidate.robustRelative || 0)
            + 0.05 * Math.max(0, candidate.autocorrelation?.score || 0);
        const scoreRatio = candidate => (candidate?.score || 0)
            / Math.max(original.score || 0, EPS);
        const hintSupports = (candidate, tolerance = 0.15) => hintTrusted
            && Number.isFinite(hintFrequency)
            && Math.abs(hintFrequency - candidate.freq) <= tolerance
            && Math.abs(hintFrequency - original.freq) >= 0.20;
        const trustedHintSupportsLow = hintTrusted
            && Number.isFinite(hintFrequency)
            && Math.abs(hintFrequency - original.freq) <= 0.10;
        if (trustedHintSupportsLow) {
            // History is diagnostic context only. It must never veto stronger
            // current-window evidence or convert an ambiguous low peak into a
            // high-confidence baseline.
            decision.reasonCodes.push('fresh-history-supports-low-softly');
        }

        const ratioLow = Math.max(1.5, Number(options.octaveRatioLow) || 1.80);
        const ratioHigh = Math.max(ratioLow + 0.05, Number(options.octaveRatioHigh) || 2.20);
        const upperMinimum = Math.max(
            Number(options.octaveUpperMinHz) || 0.90
        );
        const upperCandidates = ranked.filter(candidate => {
            if (!candidate || candidate.index === original.index || !Number.isFinite(candidate.freq)) return false;
            const ratio = candidate.freq / original.freq;
            const octaveError = Math.abs(Math.log2(Math.max(ratio, EPS)) - 1);
            const resolutionTolerance = Number.isFinite(frequencyResolution) && frequencyResolution > 0
                ? (Number(options.octaveResolutionBins) || 3)
                    * frequencyResolution / Math.max(candidate.freq * Math.LN2, EPS)
                : 0;
            const octaveTolerance = Math.max(
                Number(options.octaveToleranceOctaves) || 0.10,
                resolutionTolerance
            );
            return ratio >= ratioLow && ratio <= ratioHigh
                && octaveError <= octaveTolerance
                && candidate.freq >= upperMinimum;
        });
        upperCandidates.sort((left, right) => candidateEvidence(right) - candidateEvidence(left));
        const upper = upperCandidates[0] || null;

        if (upper) {
            const upperScoreRatio = scoreRatio(upper);
            decision.upperBpm = upper.freq * 60;
            decision.upperToLowerRatio = upper.freq / original.freq;
            decision.upperScoreRatio = upperScoreRatio;
            decision.octaveError = Math.abs(Math.log2(decision.upperToLowerRatio) - 1);
            const resolutionTolerance = Number.isFinite(frequencyResolution) && frequencyResolution > 0
                ? (Number(options.octaveResolutionBins) || 3)
                    * frequencyResolution / Math.max(upper.freq * Math.LN2, EPS)
                : 0;
            decision.octaveTolerance = Math.max(
                Number(options.octaveToleranceOctaves) || 0.10,
                resolutionTolerance
            );

            const plausibleUpper = upper.rawPeak && !upper.rawPeak.fallback
                && (upper.rawRelative || 0)
                    >= (Number(options.octavePlausibleUpperRawRelative) || 0.10)
                && (upper.prominenceRelative || 0)
                    >= (Number(options.octavePlausibleUpperProminence) || 0.08)
                && upperScoreRatio
                    >= (Number(options.octavePlausibleUpperScoreRatio) || 0.10);
            const credibleUpper = upper.rawPeak && !upper.rawPeak.fallback
                && (upper.rawRelative || 0) >= options.octaveMinUpperRawRelative
                && (upper.prominenceRelative || 0) >= options.octaveMinUpperProminence
                && (upper.score || 0) >= options.octaveMinUpperScore
                && upperScoreRatio >= options.octaveMinUpperScoreRatio;
            const strongIndependentUpper = (upper.rawRelative || 0)
                    >= (Number(options.octaveStrongUpperRawRelative) || 0.38)
                && (upper.prominenceRelative || 0)
                    >= (Number(options.octaveStrongUpperProminence) || 0.38);
            const trustedHintSupportsUpper = hintSupports(upper);
            const respirationSupportsUpper = lowHasRespiratoryArtifactEvidence
                && credibleUpper
                && (lowIsRespirationFundamental
                    || (upper.harmonic?.closeness || 0) + 0.15 < lowHarmonicCloseness);
            decision.strongIndependentUpper = strongIndependentUpper;

            if (credibleUpper && (
                trustedHintSupportsUpper
                || strongIndependentUpper
                || respirationSupportsUpper
            )) {
                decision.applied = true;
                decision.reason = trustedHintSupportsUpper
                    ? 'trusted-fresh-heart-history-supports-upper'
                    : respirationSupportsUpper
                    ? 'low-candidate-matches-measured-respiration'
                    : 'strong-independent-upper-peak';
                decision.reasonCodes.push(decision.reason);
                decision.selectedBpm = upper.freq * 60;
                return { selected: upper, decision };
            }

            if (plausibleUpper) {
                decision.ambiguous = true;
                decision.reason = credibleUpper
                    ? 'low-upper-pair-physically-ambiguous'
                    : 'weak-double-frequency-alternative';
                decision.reasonCodes.push(
                    decision.reason,
                    'do-not-establish-new-baseline'
                );
                return { selected: original, decision };
            }
        }

        // A collar artefact is often 45-60 bpm while the cardiac candidate is
        // 70-100 bpm, so the pair is not necessarily an exact octave. Review
        // the whole higher-frequency candidate set, but only promote a higher
        // candidate when respiration supplies independent evidence against the
        // low one (or trusted history explicitly supports the higher track).
        const alternativeMinimum = Math.max(
            Number(options.lowHeartAlternativeMinHz) || 1.05,
            original.freq + (Number(options.lowHeartAlternativeSeparationHz) || 0.18)
        );
        const alternatives = ranked.filter(candidate => candidate
            && candidate.index !== original.index
            && Number.isFinite(candidate.freq)
            && candidate.freq >= alternativeMinimum
            && candidate.rawPeak
            && !candidate.rawPeak.fallback
        ).sort((left, right) => {
            const leftScore = candidateEvidence(left) + 0.08 * (1 - (left.harmonic?.closeness || 0));
            const rightScore = candidateEvidence(right) + 0.08 * (1 - (right.harmonic?.closeness || 0));
            return rightScore - leftScore;
        });
        const alternative = alternatives[0] || null;
        if (alternative) {
            const alternativeScoreRatio = scoreRatio(alternative);
            decision.alternativeBpm = alternative.freq * 60;
            decision.alternativeScoreRatio = alternativeScoreRatio;
            const credibleAlternative = (alternative.rawRelative || 0)
                    >= (Number(options.lowHeartAlternativeMinRawRelative) || 0.32)
                && (alternative.prominenceRelative || 0)
                    >= (Number(options.lowHeartAlternativeMinProminence) || 0.28)
                && alternativeScoreRatio
                    >= (Number(options.lowHeartAlternativeMinScoreRatio) || 0.45);
            const alternativeLessRespiratory = (alternative.harmonic?.closeness || 0) + 0.15
                < lowHarmonicCloseness;
            const trustedHintSupportsAlternative = hintSupports(alternative, 0.18);

            if (credibleAlternative && (
                trustedHintSupportsAlternative
                || (lowHasRespiratoryArtifactEvidence && alternativeLessRespiratory)
            )) {
                decision.applied = true;
                decision.reason = trustedHintSupportsAlternative
                    ? 'trusted-history-supports-higher-alternative'
                    : 'respiration-overlap-rejected-low-candidate';
                decision.reasonCodes.push(decision.reason);
                decision.selectedBpm = alternative.freq * 60;
                return { selected: alternative, decision };
            }

            if (credibleAlternative) {
                decision.ambiguous = true;
                decision.reason = 'low-higher-candidates-ambiguous';
                decision.reasonCodes.push(
                    decision.reason,
                    'do-not-establish-new-baseline'
                );
                return { selected: original, decision };
            }
        }

        if (lowHasRespiratoryArtifactEvidence) {
            decision.ambiguous = true;
            decision.reason = lowIsRespirationFundamental
                ? 'heart-candidate-overlaps-respiration'
                : 'heart-candidate-overlaps-respiration-harmonic';
            decision.reasonCodes.push(
                decision.reason,
                'do-not-establish-new-baseline'
            );
            return { selected: original, decision };
        }

        decision.reason = upper
            ? 'double-frequency-peak-too-weak'
            : 'no-credible-higher-heart-candidate';
        decision.reasonCodes.push(decision.reason);
        return { selected: original, decision };
    }

    function peakConfidence(selected, ranked, frequency, amplitude, low, high, durationSeconds, minCycles, signalQuality) {
        if (!selected) return { score: 0, ratio: 0, snrDb: -Infinity, cycles: 0 };
        const stats = bandStatistics(frequency, amplitude, low, high);
        const floor = Math.max(stats.floor, EPS);
        const selectedIndex = clamp(Math.round(selected.index), 0, amplitude.length - 1);
        const selectedAmplitude = Math.max(amplitude[selectedIndex] || 0, EPS);
        const snrDb = 20 * Math.log10(selectedAmplitude / floor);
        const snrScore = clamp((snrDb - 2) / 14, 0, 1);
        const sourcePeak = selected.rawPeak || selected;
        const prominenceFraction = clamp(
            (sourcePeak.prominence || 0) / Math.max(sourcePeak.amp || sourcePeak.binAmp || 0, EPS),
            0,
            1
        );
        const second = ranked.find(candidate => candidate.index !== selected.index);
        const ratio = second
            ? selected.score / Math.max(second.score, EPS)
            : selectedAmplitude / floor;
        const separationScore = second ? clamp((ratio - 1) / 0.75, 0, 1) : snrScore;
        const cycles = durationSeconds * selected.freq;
        const cycleScore = clamp(cycles / Math.max(minCycles, EPS), 0, 1);
        const autocorrelationScore = selected.autocorrelation?.score || 0;
        const spectralScore = 0.38 * snrScore
            + 0.27 * prominenceFraction
            + 0.15 * separationScore
            + 0.20 * autocorrelationScore;
        const score = clamp(spectralScore * cycleScore * (0.35 + 0.65 * signalQuality), 0, 1);
        return { score, ratio, snrDb, cycles, cycleScore, prominenceFraction, separationScore };
    }

    function summarizePeaks(peaks) {
        return (peaks || []).slice(0, 5).map(peak => ({
            freq: peak.freq,
            bpm: peak.freq * 60,
            binFreq: peak.binFreq,
            amp: peak.amp,
            prominence: peak.prominence,
            score: peak.score,
            rawRelative: peak.rawRelative,
            robustRelative: peak.robustRelative,
            prominenceRelative: peak.prominenceRelative,
            autocorrelation: peak.autocorrelation?.correlation,
            harmonicOrder: peak.harmonic?.order,
            harmonicCloseness: peak.harmonic?.closeness
        }));
    }

    function raisedCosineBandWeight(frequency, low, high, rolloffHz) {
        const absoluteFrequency = Math.abs(frequency);
        const rolloff = Math.max(rolloffHz, 1e-6);
        if (absoluteFrequency < low - rolloff || absoluteFrequency > high + rolloff) return 0;
        if (absoluteFrequency >= low && absoluteFrequency <= high) return 1;
        if (absoluteFrequency < low) {
            const x = clamp((absoluteFrequency - (low - rolloff)) / rolloff, 0, 1);
            return 0.5 - 0.5 * Math.cos(Math.PI * x);
        }
        const x = clamp(((high + rolloff) - absoluteFrequency) / rolloff, 0, 1);
        return 0.5 - 0.5 * Math.cos(Math.PI * x);
    }

    class VitalsExtractorBase {
        constructor(fs = 50, options = {}) {
            this.fs = Number.isFinite(fs) && fs > 0 ? fs : 50;
            this.options = { ...DEFAULTS, ...options };
            this.methodName = 'base';
        }

        setSampleRate(fs) {
            if (Number.isFinite(fs) && fs > 0) this.fs = fs;
        }

        setOptions(options = {}) {
            this.options = { ...this.options, ...options };
        }

        _effectiveBands(fs) {
            const nyquistSafe = fs * 0.45;
            return {
                rrLow: Math.max(0.02, this.options.rrLow),
                rrHigh: Math.min(this.options.rrHigh, nyquistSafe),
                hrLow: Math.max(0.10, this.options.hrLow),
                hrHigh: Math.min(this.options.hrHigh, nyquistSafe)
            };
        }

        run(iData, qData, fs = this.fs, hints = {}) {
            this.setSampleRate(fs);
            const nSamples = Math.min((iData || []).length, (qData || []).length);
            if (nSamples < this.options.minSamples) {
                return this._emptyResult('样本不足', nSamples);
            }
            const bands = this._effectiveBands(this.fs);
            if (bands.hrHigh <= bands.hrLow || bands.rrHigh <= bands.rrLow) {
                return this._emptyResult('采样率不足，无法覆盖设定频段', nSamples, {
                    requiredFs: Math.max(this.options.hrHigh, this.options.rrHigh) / 0.45
                });
            }

            const signal = preprocessMotion(iData, qData, this.options, hints, this.fs);
            if (Number.isFinite(hints.motionQuality)) {
                const motionQuality = clamp(hints.motionQuality, 0, 1);
                signal.quality.motionScore = motionQuality;
                signal.quality.score *= 0.20 + 0.80 * motionQuality;
                if (motionQuality < 0.08) {
                    signal.quality.valid = false;
                    signal.quality.reason = '佩戴端运动伪影过强';
                }
            }
            if (!signal.quality.valid || signal.quality.signalStd <= this.options.minSignalStd) {
                return this._emptyResult(signal.quality.reason || '信号质量不足', nSamples, {
                    quality: signal.quality,
                    circle: signal.circle,
                    bands
                });
            }
            const durationSeconds = nSamples / this.fs;

            const rrFiltered = this.bandpass(signal.re, signal.im, bands.rrLow, bands.rrHigh, this.fs);
            const rrWave = detrend(rrFiltered.re.slice(0, nSamples));
            const rrSpectrum = this.spectrumFromTime(rrWave, this.fs, signal.nFft);
            const rrPeaks = findPeaksProminence(
                rrSpectrum.f,
                rrSpectrum.amp,
                bands.rrLow,
                bands.rrHigh,
                this.options.minProminenceRatio,
                { edgeBins: this.options.bandEdgeBins }
            );
            const rrScored = scorePeakCandidates({
                candidates: rrPeaks,
                frequency: rrSpectrum.f,
                rawAmplitude: rrSpectrum.amp,
                robustAmplitude: rrSpectrum.amp,
                filteredWave: rrWave,
                fs: this.fs,
                low: bands.rrLow,
                high: bands.rrHigh,
                hintFrequency: hints.rrHintFreq,
                rrFrequency: NaN,
                options: { ...this.options, ...bands },
                kind: 'rr'
            });
            const rrPeak = rrScored.selected;
            const rrFrequency = rrPeak ? rrPeak.freq : NaN;
            const rrConfidenceDetail = peakConfidence(
                rrPeak,
                rrScored.ranked,
                rrSpectrum.f,
                rrSpectrum.amp,
                bands.rrLow,
                bands.rrHigh,
                durationSeconds,
                this.options.minRrCycles,
                signal.quality.score
            );
            const rrConfidence = rrConfidenceDetail.score;

            // Prefer the current window's RR when trustworthy. A prior RR is only
            // a fallback; it can never create a permanent spectral notch by itself.
            const rrHintTrusted = hints.rrHintMeta?.status === 'fresh'
                && hints.rrHintMeta?.ambiguous !== true
                && Number(hints.rrHintMeta?.confidence) >= 0.50
                && Number(hints.rrHintMeta?.freshWindowCount) >= 2;
            const currentRrUsable = rrConfidence
                    >= (Number(this.options.crossBandMinRrConfidence) || 0.10)
                && Number.isFinite(rrFrequency);
            const rrUsedForHarmonic = currentRrUsable
                ? rrFrequency
                : rrHintTrusted && Number.isFinite(hints.rrHintFreq) && hints.rrHintFreq > 0
                ? hints.rrHintFreq
                : NaN;
            // Continuous evidence avoids the old 0.249 -> no suppression / 0.250
            // -> deep suppression discontinuity. RR may down-weight a candidate,
            // but raw-spectrum candidates are never deleted.
            const harmonicStrength = currentRrUsable
                ? clamp(
                    (rrConfidence - (Number(this.options.crossBandMinRrConfidence) || 0.10))
                        / Math.max(0.35, 1 - (Number(this.options.crossBandMinRrConfidence) || 0.10)),
                    0.05,
                    1
                )
                : rrHintTrusted ? 0.15 : 0;

            const hrFiltered = this.bandpass(signal.re, signal.im, bands.hrLow, bands.hrHigh, this.fs);
            const hrWave = detrend(hrFiltered.re.slice(0, nSamples));
            const hrSpectrum = this.spectrumFromTime(hrWave, this.fs, signal.nFft);
            const suppressionOptions = {
                ...this.options,
                ...bands,
                harmonicEvidenceStrength: harmonicStrength,
                rrEvidenceConfidence: currentRrUsable ? rrConfidence : 0
            };
            const robustHrAmplitude = gaussianHarmonicSuppression(
                hrSpectrum.f,
                hrSpectrum.amp,
                rrUsedForHarmonic,
                suppressionOptions,
                harmonicStrength
            );
            const rawHrPeaks = findPeaksProminence(
                hrSpectrum.f,
                hrSpectrum.amp,
                bands.hrLow,
                bands.hrHigh,
                this.options.minProminenceRatio,
                { edgeBins: this.options.bandEdgeBins }
            );
            const robustHrPeaks = findPeaksProminence(
                hrSpectrum.f,
                robustHrAmplitude,
                bands.hrLow,
                bands.hrHigh,
                this.options.minProminenceRatio,
                { edgeBins: this.options.bandEdgeBins }
            );
            const mergedHrPeaks = mergePeakCandidates(rawHrPeaks, robustHrPeaks);
            const hrScored = scorePeakCandidates({
                candidates: mergedHrPeaks,
                frequency: hrSpectrum.f,
                rawAmplitude: hrSpectrum.amp,
                robustAmplitude: robustHrAmplitude,
                filteredWave: hrWave,
                fs: this.fs,
                low: bands.hrLow,
                high: bands.hrHigh,
                hintFrequency: hints.hrHintFreq,
                rrFrequency: rrUsedForHarmonic,
                options: suppressionOptions,
                kind: 'hr'
            });
            const octaveResolution = resolveHeartRateOctave(
                hrScored.ranked,
                rrUsedForHarmonic,
                hints.hrHintFreq,
                hints.hrHintMeta,
                suppressionOptions,
                1 / Math.max(durationSeconds, EPS)
            );
            const hrPeak = octaveResolution.selected;
            const hrFrequency = hrPeak ? hrPeak.freq : NaN;
            const hrConfidenceDetail = peakConfidence(
                hrPeak,
                hrScored.ranked,
                hrSpectrum.f,
                hrSpectrum.amp,
                bands.hrLow,
                bands.hrHigh,
                durationSeconds,
                this.options.minHrCycles,
                signal.quality.score
            );
            // A 1:2 pair without trustworthy history or artefact evidence is
            // not identifiable from one spectrum. Return the conservative low
            // hypothesis for diagnostics, but lower confidence so the tracker
            // holds its prior value instead of creating a wrong 40/80 baseline.
            const hrConfidence = octaveResolution.decision.ambiguous
                ? hrConfidenceDetail.score * 0.20
                : hrConfidenceDetail.score;

            const rrBpm = Number.isFinite(rrFrequency) ? rrFrequency * 60 : NaN;
            const hrBpm = Number.isFinite(hrFrequency) ? hrFrequency * 60 : NaN;
            const reason = !Number.isFinite(hrBpm) && !Number.isFinite(rrBpm)
                ? '未找到有效 HR/RR 峰'
                : !Number.isFinite(hrBpm)
                ? '未找到有效 HR 峰'
                : !Number.isFinite(rrBpm)
                ? '未找到有效 RR 峰'
                : '';

            return {
                RR_bpm: rrBpm,
                HR_bpm: hrBpm,
                RR_confidence: rrConfidence,
                HR_confidence: hrConfidence,
                RR_freq: rrFrequency,
                HR_freq: hrFrequency,
                RR_peak: rrPeak,
                HR_peak: hrPeak,
                method: this.methodName,
                samples: nSamples,
                fs: this.fs,
                durationSeconds,
                valid: Number.isFinite(hrBpm) || Number.isFinite(rrBpm),
                reason,
                quality: signal.quality,
                circle: signal.circle,
                waveforms: {
                    phase: signal.motion.slice(),
                    respiratory: rrWave,
                    heartbeat: hrWave
                },
                spectrum: {
                    rr: {
                        f: rrSpectrum.f,
                        P: rrSpectrum.amp,
                        peakFreq: rrFrequency,
                        peaks: rrScored.ranked.slice(0, 8),
                        selectedPeak: rrPeak
                    },
                    hr: {
                        f: hrSpectrum.f,
                        P_raw: hrSpectrum.amp,
                        P_robust: robustHrAmplitude,
                        peakFreq: hrFrequency,
                        peaks: hrScored.ranked.slice(0, 8),
                        rawPeaks: rawHrPeaks.slice(0, 8),
                        selectedPeak: hrPeak
                    }
                },
                diagnostics: {
                    rrPeaksTop: summarizePeaks(rrScored.ranked),
                    hrPeaksRawTop: summarizePeaks(rawHrPeaks),
                    hrPeaksRobustTop: summarizePeaks(hrScored.ranked),
                    rrPeakRatio: rrConfidenceDetail.ratio,
                    hrPeakRatio: hrConfidenceDetail.ratio,
                    rrSnrDb: rrConfidenceDetail.snrDb,
                    hrSnrDb: hrConfidenceDetail.snrDb,
                    rrCycles: rrConfidenceDetail.cycles,
                    hrCycles: hrConfidenceDetail.cycles,
                    hrAutocorrelation: hrPeak?.autocorrelation,
                    rrAutocorrelation: rrPeak?.autocorrelation,
                    hrHarmonicOrder: hrPeak?.harmonic?.order,
                    hrHarmonicDistance: hrPeak?.harmonic?.distance,
                    rrUsedForHarmonic,
                    harmonicStrength,
                    octaveCorrection: octaveResolution.decision,
                    rrHintFreq: hints.rrHintFreq,
                    hrHintFreq: hints.hrHintFreq,
                    effectiveBands: bands
                },
                options: {
                    ...bands,
                    bandEdgeBins: this.options.bandEdgeBins,
                    harmonicBw: this.options.harmonicBw,
                    harmonicDepth: this.options.harmonicDepth,
                    maxHarmonic: this.options.maxHarmonic,
                    minHrCycles: this.options.minHrCycles,
                    minRrCycles: this.options.minRrCycles
                }
            };
        }

        spectrumFromTime(values, fs, nFft = nextPow2(values.length)) {
            const padded = padReal(values, nFft);
            const windowed = applyHann(padded.re, padded.im, values.length);
            const transformed = fftComplex(windowed.re, windowed.im, false);
            return oneSidedSpectrum(transformed.re, transformed.im, fs, windowed.windowSumSq);
        }

        bandpass() {
            throw new Error('bandpass() must be implemented by a subclass');
        }

        _emptyResult(reason, samples = 0, extra = {}) {
            return {
                RR_bpm: NaN,
                HR_bpm: NaN,
                RR_confidence: 0,
                HR_confidence: 0,
                RR_freq: NaN,
                HR_freq: NaN,
                RR_peak: null,
                HR_peak: null,
                method: this.methodName,
                reason,
                valid: false,
                samples,
                fs: this.fs,
                spectrum: null,
                waveforms: null,
                quality: extra.quality || { score: 0, valid: false, reason },
                ...extra
            };
        }
    }

    class VitalsExtractorFreqBP extends VitalsExtractorBase {
        constructor(fs, options) {
            super(fs, options);
            this.methodName = 'phase-freqBP';
        }

        bandpass(re, im, low, high, fs) {
            const transformed = fftComplex(re, im, false);
            const n = transformed.re.length;
            const order = Math.max(1, this.options.cascadeOrder | 0);
            for (let index = 0; index < n; index++) {
                const frequency = index <= n / 2
                    ? index * fs / n
                    : -(n - index) * fs / n;
                let weight = raisedCosineBandWeight(frequency, low, high, this.options.rolloffHz);
                if (order > 1) weight = weight ** order;
                transformed.re[index] *= weight;
                transformed.im[index] *= weight;
            }
            return fftComplex(transformed.re, transformed.im, true);
        }
    }

    class VitalsExtractorCWT extends VitalsExtractorBase {
        constructor(fs, options) {
            super(fs, options);
            this.methodName = 'phase-cwt-filterbank';
            this.voicesPerOctave = options?.voicesPerOctave || this.options.voicesPerOctave;
            this._weightCache = new Map();
        }

        bandpass(re, im, low, high, fs) {
            const transformed = fftComplex(re, im, false);
            const n = transformed.re.length;
            const accumulatedRe = new Array(n).fill(0);
            const accumulatedIm = new Array(n).fill(0);
            const cacheKey = [n, fs, low, high, this.voicesPerOctave, this.options.rolloffHz].join('|');
            let cached = this._weightCache.get(cacheKey);
            if (!cached) {
                const centers = this._logCenters(
                    Math.max(low * 0.85, 1e-4),
                    high * 1.15,
                    this.voicesPerOctave
                );
                const sigmaLog = Math.log(2) / Math.max(2, this.voicesPerOctave);
                const weights = new Array(n).fill(0);
                for (let index = 0; index < n; index++) {
                    const frequency = index <= n / 2
                        ? index * fs / n
                        : -(n - index) * fs / n;
                    const absoluteFrequency = Math.max(Math.abs(frequency), 1e-9);
                    const bandWeight = raisedCosineBandWeight(
                        frequency,
                        low,
                        high,
                        this.options.rolloffHz
                    );
                    if (bandWeight <= 0) continue;
                    let sum = 0;
                    for (const center of centers) {
                        const logDistance = Math.log(absoluteFrequency / center);
                        const waveletWeight = Math.exp(-0.5 * (logDistance / sigmaLog) ** 2);
                        // Equalize pass-band gain across log-spaced centers.
                        // The previous 1/sqrt(center) factor systematically
                        // amplified 35-45 bpm components relative to 80-90 bpm
                        // and made half-rate artifacts win the HR ranking.
                        sum += waveletWeight;
                    }
                    weights[index] = sum * bandWeight;
                }
                cached = {
                    weights,
                    normalization: Math.max(1e-6, centers.length * Math.log(2) / this.voicesPerOctave)
                };
                this._weightCache.set(cacheKey, cached);
            }

            for (let index = 0; index < n; index++) {
                const weight = cached.weights[index] / cached.normalization;
                accumulatedRe[index] = transformed.re[index] * weight;
                accumulatedIm[index] = transformed.im[index] * weight;
            }
            return fftComplex(accumulatedRe, accumulatedIm, true);
        }

        _logCenters(low, high, voicesPerOctave) {
            const start = Math.log2(Math.max(low, 1e-6));
            const end = Math.log2(Math.max(high, low * 1.01));
            const count = Math.max(2, Math.ceil((end - start) * voicesPerOctave) + 1);
            const centers = [];
            for (let index = 0; index < count; index++) {
                centers.push(2 ** (start + (end - start) * index / (count - 1)));
            }
            return centers;
        }
    }

    return {
        create(method = 'freqBP', fs = 50, options = {}) {
            return method === 'cwt'
                ? new VitalsExtractorCWT(fs, options)
                : new VitalsExtractorFreqBP(fs, options);
        },
        DEFAULTS: { ...DEFAULTS },
        VitalsExtractorBase,
        VitalsExtractorFreqBP,
        VitalsExtractorCWT,
        utils: {
            fftComplex,
            findPeaksProminence,
            gaussianHarmonicSuppression,
            harmonicDepthForOrder,
            parabolicRefine,
            detrend,
            hannWindow,
            applyHann,
            fitCircle,
            demodulateMotion,
            median,
            medianAbsoluteDeviation,
            autocorrelationSupport,
            resolveHeartRateOctave
        }
    };
});
