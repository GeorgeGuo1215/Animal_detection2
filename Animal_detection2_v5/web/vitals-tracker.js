/**
 * HR/RR tracking state machine.
 *
 * The tracker deliberately starts without a physiological default. Each vital
 * warms up, holds and expires independently so a bad RR estimate cannot hide
 * a good HR estimate (or vice versa). update() accepts an optional context:
 * { timestampMs, motionState, motionQuality }.
 */
(function (root, factory) {
    'use strict';

    const VitalsTracker = factory();

    if (typeof module !== 'undefined' && module.exports) {
        module.exports = VitalsTracker;
    }
    if (root) {
        root.VitalsTracker = VitalsTracker;
    }
})(typeof window !== 'undefined' ? window : globalThis, function () {
    'use strict';

    const DEFAULTS = Object.freeze({
        minConfidence: 0.35,
        warmupWindows: 2,
        reacquireWindows: 2,
        smoothWindows: 3,
        jumpConfirmWindows: 3,
        // Deprecated window-count option retained for source compatibility.
        // Expiry is now time based so changing the UI update period cannot
        // unexpectedly shorten (or lengthen) the continuity hold.
        invalidExpireWindows: 5,
        holdMaxMs: 30000,
        hrHoldConfidenceTauMs: 12000,
        rrHoldConfidenceTauMs: 20000,
        motionRejectQuality: 0.08,
        hrJumpThreshold: 25,
        rrJumpThreshold: 8,
        hrConsistencyTolerance: 8,
        rrConsistencyTolerance: 3,
        hrGuardTolerance: 12,
        rrGuardTolerance: 4,
        hrReacquireTolerance: 12,
        rrReacquireTolerance: 4,
        // Adjacent 20 s windows updated every second share ~95% of their data.
        // The app can require real elapsed evidence before treating them as
        // independent confirmations; zero preserves the standalone API.
        warmupMinSpanMs: 0,
        jumpConfirmMinSpanMs: 0,
        evidenceHistoryMax: 30,
        pendingJumpMaxMisses: 2,
        // A low HR is not rejected. It simply waits for an agreeing long-window
        // result before becoming the very first baseline when this option is set.
        requireGuardForLowHrBelowBpm: 0
    });

    function finiteNumber(value) {
        return typeof value === 'number' && Number.isFinite(value);
    }

    function clampConfidence(value) {
        if (!finiteNumber(value)) return 0;
        return Math.max(0, Math.min(1, value));
    }

    function median(values) {
        const sorted = values.filter(finiteNumber).slice().sort((a, b) => a - b);
        if (!sorted.length) return NaN;
        const middle = Math.floor(sorted.length / 2);
        return sorted.length % 2
            ? sorted[middle]
            : (sorted[middle - 1] + sorted[middle]) / 2;
    }

    function minimumConfidence(entries) {
        if (!entries.length) return 0;
        return Math.min(...entries.map(entry => clampConfidence(entry.confidence)));
    }

    function positiveInteger(value, fallback) {
        return Number.isInteger(value) && value > 0 ? value : fallback;
    }

    function positiveNumber(value, fallback) {
        return finiteNumber(value) && value > 0 ? value : fallback;
    }

    function nonNegativeNumber(value, fallback) {
        return finiteNumber(value) && value >= 0 ? value : fallback;
    }

    class VitalsTracker {
        constructor(options = {}) {
            this.options = {
                ...DEFAULTS,
                ...options
            };

            this.options.minConfidence = clampConfidence(this.options.minConfidence);
            this.options.warmupWindows = positiveInteger(
                this.options.warmupWindows,
                DEFAULTS.warmupWindows
            );
            this.options.reacquireWindows = positiveInteger(
                this.options.reacquireWindows,
                DEFAULTS.reacquireWindows
            );
            this.options.smoothWindows = positiveInteger(
                this.options.smoothWindows,
                DEFAULTS.smoothWindows
            );
            this.options.jumpConfirmWindows = positiveInteger(
                this.options.jumpConfirmWindows,
                DEFAULTS.jumpConfirmWindows
            );
            this.options.invalidExpireWindows = positiveInteger(
                this.options.invalidExpireWindows,
                DEFAULTS.invalidExpireWindows
            );
            this.options.holdMaxMs = positiveNumber(
                this.options.holdMaxMs,
                DEFAULTS.holdMaxMs
            );
            this.options.hrHoldConfidenceTauMs = positiveNumber(
                this.options.hrHoldConfidenceTauMs,
                DEFAULTS.hrHoldConfidenceTauMs
            );
            this.options.rrHoldConfidenceTauMs = positiveNumber(
                this.options.rrHoldConfidenceTauMs,
                DEFAULTS.rrHoldConfidenceTauMs
            );
            this.options.motionRejectQuality = finiteNumber(this.options.motionRejectQuality)
                ? clampConfidence(this.options.motionRejectQuality)
                : DEFAULTS.motionRejectQuality;
            this.options.hrJumpThreshold = positiveNumber(
                this.options.hrJumpThreshold,
                DEFAULTS.hrJumpThreshold
            );
            this.options.rrJumpThreshold = positiveNumber(
                this.options.rrJumpThreshold,
                DEFAULTS.rrJumpThreshold
            );
            this.options.hrConsistencyTolerance = positiveNumber(
                this.options.hrConsistencyTolerance,
                DEFAULTS.hrConsistencyTolerance
            );
            this.options.rrConsistencyTolerance = positiveNumber(
                this.options.rrConsistencyTolerance,
                DEFAULTS.rrConsistencyTolerance
            );
            this.options.hrGuardTolerance = positiveNumber(
                this.options.hrGuardTolerance,
                DEFAULTS.hrGuardTolerance
            );
            this.options.rrGuardTolerance = positiveNumber(
                this.options.rrGuardTolerance,
                DEFAULTS.rrGuardTolerance
            );
            this.options.hrReacquireTolerance = positiveNumber(
                this.options.hrReacquireTolerance,
                DEFAULTS.hrReacquireTolerance
            );
            this.options.rrReacquireTolerance = positiveNumber(
                this.options.rrReacquireTolerance,
                DEFAULTS.rrReacquireTolerance
            );
            this.options.warmupMinSpanMs = nonNegativeNumber(
                this.options.warmupMinSpanMs,
                DEFAULTS.warmupMinSpanMs
            );
            this.options.jumpConfirmMinSpanMs = nonNegativeNumber(
                this.options.jumpConfirmMinSpanMs,
                DEFAULTS.jumpConfirmMinSpanMs
            );
            this.options.evidenceHistoryMax = positiveInteger(
                this.options.evidenceHistoryMax,
                DEFAULTS.evidenceHistoryMax
            );
            this.options.pendingJumpMaxMisses = positiveInteger(
                this.options.pendingJumpMaxMisses,
                DEFAULTS.pendingJumpMaxMisses
            );
            this.options.requireGuardForLowHrBelowBpm = nonNegativeNumber(
                this.options.requireGuardForLowHrBelowBpm,
                DEFAULTS.requireGuardForLowHrBelowBpm
            );

            this.reset();
        }

        reset() {
            this._lastUpdateAtMs = NaN;
            this._state = {
                HR: this._newMetricState(),
                RR: this._newMetricState()
            };
            return this._snapshot([]);
        }

        update(result = {}, guardResult = null, context = {}) {
            const messages = [];
            const nowMs = this._resolveTimestamp(context);
            this._updateMetric('HR', result, guardResult, context, nowMs, messages);
            this._updateMetric('RR', result, guardResult, context, nowMs, messages);
            return this._snapshot(messages, nowMs);
        }

        _newMetricState() {
            return {
                baseline: NaN,
                outputConfidence: 0,
                history: [],
                warmup: [],
                pendingJump: [],
                pendingJumpMisses: 0,
                invalidCount: 0,
                lastFreshAtMs: NaN,
                status: 'warming',
                everPublished: false,
                expiredAnchor: NaN
            };
        }

        _metricConfig(kind) {
            if (kind === 'HR') {
                return {
                    valueKey: 'HR_bpm',
                    confidenceKey: 'HR_confidence',
                    jumpThreshold: this.options.hrJumpThreshold,
                    consistencyTolerance: this.options.hrConsistencyTolerance,
                    guardTolerance: this.options.hrGuardTolerance,
                    reacquireTolerance: this.options.hrReacquireTolerance
                };
            }
            return {
                valueKey: 'RR_bpm',
                confidenceKey: 'RR_confidence',
                jumpThreshold: this.options.rrJumpThreshold,
                consistencyTolerance: this.options.rrConsistencyTolerance,
                guardTolerance: this.options.rrGuardTolerance,
                reacquireTolerance: this.options.rrReacquireTolerance
            };
        }

        _resolveTimestamp(context) {
            const supplied = context && finiteNumber(context.timestampMs)
                ? context.timestampMs
                : Date.now();
            if (!finiteNumber(this._lastUpdateAtMs)) {
                this._lastUpdateAtMs = supplied;
            } else {
                // A device clock can jump backwards when reconnecting. Clamp it
                // here so a stale result can never become younger again.
                this._lastUpdateAtMs = Math.max(this._lastUpdateAtMs, supplied);
            }
            return this._lastUpdateAtMs;
        }

        _contextMetricValue(value, kind) {
            if (!value || typeof value !== 'object' || Array.isArray(value)) return value;
            if (Object.prototype.hasOwnProperty.call(value, kind)) return value[kind];
            const lower = kind.toLowerCase();
            return Object.prototype.hasOwnProperty.call(value, lower) ? value[lower] : undefined;
        }

        _isMotionBlocked(kind, context) {
            if (!context || typeof context !== 'object') return false;
            const state = this._contextMetricValue(context.motionState, kind);
            if (state === true) return true;
            if (typeof state === 'string') {
                const normalized = state.trim().toLowerCase().replace(/[\s-]+/g, '_');
                if ([
                    'moving', 'motion', 'active', 'unstable', 'strong',
                    'strong_motion', 'high_motion'
                ].includes(normalized)) {
                    return true;
                }
            }

            const quality = this._contextMetricValue(context.motionQuality, kind);
            return finiteNumber(quality) && quality < this.options.motionRejectQuality;
        }

        _readCandidate(kind, result, guardResult, messages, state) {
            const config = this._metricConfig(kind);
            const primaryValue = result ? result[config.valueKey] : NaN;
            const primaryConfidence = clampConfidence(
                result ? result[config.confidenceKey] : 0
            );
            const guardValue = guardResult ? guardResult[config.valueKey] : NaN;
            const guardConfidence = clampConfidence(
                guardResult ? guardResult[config.confidenceKey] : 0
            );
            const primaryValid = finiteNumber(primaryValue)
                && primaryConfidence >= this.options.minConfidence;
            const guardValid = finiteNumber(guardValue)
                && guardConfidence >= this.options.minConfidence;

            if (primaryValid && guardValid
                && Math.abs(primaryValue - guardValue) > config.guardTolerance) {
                messages.push(
                    `${kind}短窗/长窗冲突(${primaryValue.toFixed(1)}/${guardValue.toFixed(1)})`
                );

                // A guard that only writes a warning is not a guard. Before a
                // baseline exists, reject both competing hypotheses. With an
                // established baseline, use only the window that independently
                // agrees with that track; otherwise hold the previous value.
                if (state && finiteNumber(state.baseline)) {
                    const primaryDistance = Math.abs(primaryValue - state.baseline);
                    const guardDistance = Math.abs(guardValue - state.baseline);
                    if (primaryDistance <= config.guardTolerance
                        && primaryDistance + 1 < guardDistance) {
                        messages.push(`${kind}冲突时采用接近基线的短窗`);
                        return {
                            valid: true,
                            value: primaryValue,
                            confidence: primaryConfidence,
                            guardValidated: false,
                            source: 'primary-baseline-supported'
                        };
                    }
                    if (guardDistance <= config.guardTolerance
                        && guardDistance + 1 < primaryDistance) {
                        messages.push(`${kind}冲突时采用接近基线的长窗`);
                        return {
                            valid: true,
                            value: guardValue,
                            confidence: guardConfidence,
                            guardValidated: true,
                            source: 'guard-baseline-supported'
                        };
                    }
                }
                return {
                    valid: false,
                    value: NaN,
                    confidence: 0,
                    reason: '短窗/长窗冲突'
                };
            }

            if (primaryValid) {
                return {
                    valid: true,
                    value: primaryValue,
                    confidence: primaryConfidence,
                    guardValidated: guardValid,
                    source: 'primary'
                };
            }

            if (guardValid) {
                messages.push(`${kind}使用长窗候选${guardValue.toFixed(1)}`);
                return {
                    valid: true,
                    value: guardValue,
                    confidence: guardConfidence,
                    guardValidated: true,
                    source: 'guard'
                };
            }

            return { valid: false, value: NaN, confidence: 0 };
        }

        _updateMetric(kind, result, guardResult, context, nowMs, messages) {
            const state = this._state[kind];
            const config = this._metricConfig(kind);

            if (this._isMotionBlocked(kind, context)) {
                this._handleUnavailable(kind, state, nowMs, messages, '运动中');
                return;
            }

            const candidate = this._readCandidate(kind, result, guardResult, messages, state);

            if (!candidate.valid) {
                this._handleUnavailable(
                    kind,
                    state,
                    nowMs,
                    messages,
                    candidate.reason || '低置信/无有效峰'
                );
                return;
            }

            state.invalidCount = 0;
            candidate.timestampMs = nowMs;

            if (!finiteNumber(state.baseline)) {
                if (kind === 'HR'
                    && this.options.requireGuardForLowHrBelowBpm > 0
                    && candidate.value < this.options.requireGuardForLowHrBelowBpm
                    && candidate.guardValidated !== true) {
                    this._handleUnavailable(
                        kind,
                        state,
                        nowMs,
                        messages,
                        '低心率候选等待长窗确认'
                    );
                    return;
                }
                if (
                    state.everPublished
                    && finiteNumber(state.expiredAnchor)
                    && Math.abs(candidate.value - state.expiredAnchor) <= config.reacquireTolerance
                ) {
                    this._acceptCandidate(kind, state, [candidate], nowMs, messages, '恢复');
                    return;
                }
                this._handleWarmup(
                    kind,
                    state,
                    candidate,
                    config,
                    nowMs,
                    messages,
                    state.everPublished ? this.options.reacquireWindows : this.options.warmupWindows,
                    state.everPublished ? '恢复确认' : '预热'
                );
                return;
            }

            const jump = Math.abs(candidate.value - state.baseline);
            if (jump > config.jumpThreshold) {
                this._handleLargeJump(kind, state, candidate, config, nowMs, messages);
                return;
            }

            if (state.pendingJump.length) {
                messages.push(`${kind}大跳变候选已取消`);
                state.pendingJump = [];
                state.pendingJumpMisses = 0;
            }

            state.history.push(candidate);
            while (state.history.length > this.options.smoothWindows) {
                state.history.shift();
            }
            state.baseline = median(state.history.map(entry => entry.value));
            state.outputConfidence = minimumConfidence(state.history);
            state.lastFreshAtMs = nowMs;
            state.status = 'fresh';
            state.everPublished = true;
            state.expiredAnchor = NaN;
        }

        _handleWarmup(
            kind,
            state,
            candidate,
            config,
            nowMs,
            messages,
            requiredWindows,
            label
        ) {
            const center = median(state.warmup.map(entry => entry.value));
            if (
                state.warmup.length
                && Math.abs(candidate.value - center) > config.consistencyTolerance
            ) {
                state.warmup = [];
                messages.push(`${kind}预热候选不一致，重新计数`);
            }

            state.warmup.push(candidate);
            while (state.warmup.length > this.options.evidenceHistoryMax) {
                state.warmup.shift();
            }

            const evidenceSpanMs = this._evidenceSpanMs(state.warmup);
            const spanReady = this.options.warmupMinSpanMs <= 0
                || evidenceSpanMs >= this.options.warmupMinSpanMs;
            if (state.warmup.length < requiredWindows || !spanReady) {
                state.status = state.everPublished ? 'expired' : 'warming';
                const spanText = spanReady
                    ? ''
                    : `，独立证据${(evidenceSpanMs / 1000).toFixed(1)}/${(this.options.warmupMinSpanMs / 1000).toFixed(1)}s`;
                messages.push(`${kind}${label}中${state.warmup.length}/${requiredWindows}${spanText}`);
                return;
            }

            this._acceptCandidate(kind, state, state.warmup, nowMs, messages, label);
        }

        _acceptCandidate(kind, state, entries, nowMs, messages, label) {
            state.baseline = median(entries.map(entry => entry.value));
            state.outputConfidence = minimumConfidence(entries);
            state.history = entries.slice(-this.options.smoothWindows);
            state.warmup = [];
            state.pendingJump = [];
            state.pendingJumpMisses = 0;
            state.invalidCount = 0;
            state.lastFreshAtMs = nowMs;
            state.status = 'fresh';
            state.everPublished = true;
            state.expiredAnchor = NaN;
            messages.push(`${kind}${label}完成${state.baseline.toFixed(1)}`);
        }

        _handleLargeJump(kind, state, candidate, config, nowMs, messages) {
            const pendingCenter = median(state.pendingJump.map(entry => entry.value));
            if (
                state.pendingJump.length
                && Math.abs(candidate.value - pendingCenter) > config.consistencyTolerance
            ) {
                state.pendingJump = [];
                messages.push(`${kind}大跳变候选不一致，重新计数`);
            }

            state.pendingJump.push(candidate);
            state.pendingJumpMisses = 0;
            while (state.pendingJump.length > this.options.evidenceHistoryMax) {
                state.pendingJump.shift();
            }

            const evidenceSpanMs = this._evidenceSpanMs(state.pendingJump);
            const spanReady = this.options.jumpConfirmMinSpanMs <= 0
                || evidenceSpanMs >= this.options.jumpConfirmMinSpanMs;
            if (state.pendingJump.length < this.options.jumpConfirmWindows || !spanReady) {
                state.status = 'held';
                const spanText = spanReady
                    ? ''
                    : `，独立证据${(evidenceSpanMs / 1000).toFixed(1)}/${(this.options.jumpConfirmMinSpanMs / 1000).toFixed(1)}s`;
                messages.push(`${kind}大跳变确认中${state.pendingJump.length}/${this.options.jumpConfirmWindows}${spanText}`);
                this._expireHeldMetricIfNeeded(kind, state, nowMs, messages, '跳变未确认');
                return;
            }

            this._acceptCandidate(kind, state, state.pendingJump, nowMs, messages, '接受新基线');
        }

        _handleUnavailable(kind, state, nowMs, messages, reason) {
            state.invalidCount += 1;
            state.warmup = [];
            if (state.pendingJump.length) {
                state.pendingJumpMisses += 1;
                if (state.pendingJumpMisses > this.options.pendingJumpMaxMisses) {
                    state.pendingJump = [];
                    state.pendingJumpMisses = 0;
                    messages.push(`${kind}跳变证据中断过久，重新确认`);
                } else {
                    messages.push(
                        `${kind}短暂缺测，保留跳变证据${state.pendingJumpMisses}/${this.options.pendingJumpMaxMisses}`
                    );
                }
            }

            if (!finiteNumber(state.baseline)) {
                state.status = state.everPublished ? 'expired' : 'warming';
                messages.push(`${kind}${reason}，${state.everPublished ? '结果已过期' : '等待可靠基线'}`);
                return;
            }

            const ageMs = finiteNumber(state.lastFreshAtMs)
                ? Math.max(0, nowMs - state.lastFreshAtMs)
                : this.options.holdMaxMs;
            if (ageMs <= this.options.holdMaxMs) {
                state.status = 'held';
                messages.push(
                    `${kind}${reason}，保持最近可靠值${Math.round(ageMs / 1000)}s`
                );
                return;
            }

            this._expireHeldMetric(kind, state, messages, `${reason}持续过久`);
        }

        _expireHeldMetricIfNeeded(kind, state, nowMs, messages, reason) {
            if (!finiteNumber(state.baseline) || !finiteNumber(state.lastFreshAtMs)) return false;
            if (nowMs - state.lastFreshAtMs <= this.options.holdMaxMs) return false;
            this._expireHeldMetric(kind, state, messages, reason);
            return true;
        }

        _expireHeldMetric(kind, state, messages, reason) {
            state.expiredAnchor = state.baseline;
            state.baseline = NaN;
            state.outputConfidence = 0;
            state.history = [];
            state.pendingJump = [];
            state.pendingJumpMisses = 0;
            state.status = 'expired';
            messages.push(`${kind}${reason}，最近可靠值已过期`);
        }

        _evidenceSpanMs(entries) {
            const times = entries
                .map(entry => entry.timestampMs)
                .filter(finiteNumber);
            if (times.length < 2) return 0;
            return Math.max(0, Math.max(...times) - Math.min(...times));
        }

        _snapshot(messages, nowMs = this._lastUpdateAtMs) {
            const hrValid = finiteNumber(this._state.HR.baseline);
            const rrValid = finiteNumber(this._state.RR.baseline);
            const metricSnapshot = (kind) => {
                const state = this._state[kind];
                const ageMs = finiteNumber(state.lastFreshAtMs) && finiteNumber(nowMs)
                    ? Math.max(0, nowMs - state.lastFreshAtMs)
                    : null;
                const tauMs = kind === 'HR'
                    ? this.options.hrHoldConfidenceTauMs
                    : this.options.rrHoldConfidenceTauMs;
                const confidence = state.status === 'held' && finiteNumber(ageMs)
                    ? state.outputConfidence * Math.exp(-ageMs / Math.max(1, tauMs))
                    : state.outputConfidence;
                return {
                    status: state.status,
                    held: state.status === 'held',
                    fresh: state.status === 'fresh',
                    ageMs,
                    confidence: clampConfidence(confidence)
                };
            };
            const hr = metricSnapshot('HR');
            const rr = metricSnapshot('RR');
            const hrConfidence = hrValid ? hr.confidence : 0;
            const rrConfidence = rrValid ? rr.confidence : 0;

            return {
                HR_bpm: hrValid ? this._state.HR.baseline : NaN,
                RR_bpm: rrValid ? this._state.RR.baseline : NaN,
                valid: { HR: hrValid, RR: rrValid },
                confidence: { HR: hrConfidence, RR: rrConfidence },
                // Flat aliases ease migration from the extractor result shape.
                HR_valid: hrValid,
                RR_valid: rrValid,
                HR_confidence: hrConfidence,
                RR_confidence: rrConfidence,
                held: { HR: hr.held, RR: rr.held },
                fresh: { HR: hr.fresh, RR: rr.fresh },
                stale: { HR: hr.held, RR: rr.held },
                ageMs: { HR: hr.ageMs, RR: rr.ageMs },
                status: { HR: hr.status, RR: rr.status },
                messages: messages.slice()
            };
        }
    }

    VitalsTracker.DEFAULTS = DEFAULTS;
    return VitalsTracker;
});
