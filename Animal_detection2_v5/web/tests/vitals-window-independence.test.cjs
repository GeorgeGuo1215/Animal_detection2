'use strict';

const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const test = require('node:test');
const vm = require('node:vm');
const VitalsTracker = require('../vitals-tracker.js');

function loadRadarWebApp() {
    const sourcePath = path.resolve(__dirname, '..', 'app.js');
    const source = `${fs.readFileSync(sourcePath, 'utf8')}\n;globalThis.__RadarWebApp = RadarWebApp;`;
    const noop = () => {};
    const document = {
        addEventListener: noop,
        getElementById: () => null,
        querySelector: () => null,
        querySelectorAll: () => [],
        body: { appendChild: noop, removeChild: noop }
    };
    const silentConsole = { log: noop, warn: noop, error: noop, timeStamp: noop };
    const localStorage = { getItem: () => null, setItem: noop, removeItem: noop };
    const window = { addEventListener: noop, document, localStorage, console: silentConsole };
    const sandbox = {
        window,
        document,
        localStorage,
        console: silentConsole,
        setTimeout,
        clearTimeout,
        setInterval,
        clearInterval,
        requestAnimationFrame: noop,
        cancelAnimationFrame: noop,
        performance,
        Date,
        Blob: function Blob() {},
        URL: { createObjectURL: () => '', revokeObjectURL: noop },
        alert: noop
    };
    sandbox.globalThis = sandbox;
    vm.createContext(sandbox);
    vm.runInContext(source, sandbox, { filename: sourcePath });
    return sandbox.__RadarWebApp;
}

const RadarWebApp = loadRadarWebApp();

function makeRealtimeApp({ fsHz = 50, windowSec = 20, availableSec = 12 } = {}) {
    const app = Object.create(RadarWebApp.prototype);
    const sampleCount = Math.round(fsHz * availableSec);
    let extractorRuns = 0;

    app.vitalsMethod = 'cwt';
    app.vitalsWindowSec = windowSec;
    app.vitalsMinConfidence = 0.28;
    app.vitalsWarmupMinCycles = 1;
    app.vitalsAcceptedCycles = 0;
    app.vitalsLastStableHR = null;
    app.vitalsLastStableRR = null;
    app.vitalsHrHistory = [];
    app.vitalsRrHistory = [];
    app.historyMaxLength = 3;
    app.vitalsLastGuardResult = null;
    app.vitalsLastGuardTs = 0;
    app.vitalsDisplayState = null;
    app.bleBufferI = new Array(sampleCount).fill(1);
    app.bleBufferQ = new Array(sampleCount).fill(0);
    app.vitalsTracker = new VitalsTracker({
        minConfidence: app.vitalsMinConfidence,
        warmupWindows: 1,
        smoothWindows: 3,
        jumpConfirmWindows: 3
    });
    app.vitalsExtractor = {
        setSampleRate() {},
        run() {
            extractorRuns += 1;
            return {
                HR_bpm: 82,
                RR_bpm: 18,
                HR_confidence: 0.95,
                RR_confidence: 0.95,
                diagnostics: {},
                quality: { score: 0.95 }
            };
        }
    };

    app._getVitalsFs = () => fsHz;
    app._getVitalsOptions = () => ({
        hrLow: 35 / 60,
        hrHigh: 260 / 60,
        rrLow: 6 / 60,
        rrHigh: 60 / 60
    });
    app._estimateCurrentBleFs = () => fsHz;
    app._buildVitalsHintMeta = () => null;
    app._analyzeVitalsMotion = count => ({
        state: 'stationary',
        quality: 0.95,
        metricQuality: { HR: 0.95, RR: 0.95 },
        cleanMask: new Array(count).fill(true),
        imuAvailable: false
    });
    app._applyVitalsMotionEvidence = result => result;
    app._runVitalsGuardWindow = () => null;
    app._publishVitalsResult = () => {};
    app.updateBluetoothVitalSigns = () => {};

    return {
        app,
        extractorRuns: () => extractorRuns
    };
}

function trackerSample(hr, rr = 18) {
    return {
        HR_bpm: hr,
        RR_bpm: rr,
        HR_confidence: 0.95,
        RR_confidence: 0.95
    };
}

test('实时算法未积满配置的完整20秒窗口时不得运行提取器或建立HR基线', () => {
    const { app, extractorRuns } = makeRealtimeApp({
        fsHz: 50,
        windowSec: 20,
        availableSec: 12
    });

    app._runVitalsCycle();

    assert.equal(extractorRuns(), 0, '12秒数据不能冒充配置的20秒分析窗口');
    assert.equal(app.vitalsLastStableHR, null, '不完整窗口不得进入跟踪器建立HR基线');
    assert.equal(Number.isNaN(app.vitalsTracker.update({}, null, { timestampMs: 0 }).HR_bpm), true);
});

test('warmupMinSpanMs阻止1秒高度重叠调用在2秒内建立基线', () => {
    const tracker = new VitalsTracker({
        warmupWindows: 2,
        warmupMinSpanMs: 10000
    });

    for (const timestampMs of [0, 1000, 2000]) {
        const snapshot = tracker.update(trackerSample(82), null, { timestampMs });
        assert.equal(
            Number.isNaN(snapshot.HR_bpm),
            true,
            `${timestampMs}ms时仍是同一20秒窗的高度重叠结果，不应建立基线`
        );
    }

    const independent = tracker.update(trackerSample(82), null, { timestampMs: 10000 });
    assert.equal(independent.HR_bpm, 82, '累计到配置的独立时间跨度后应允许建立基线');
    assert.equal(independent.valid.HR, true);
});

test('jumpConfirmMinSpanMs阻止1秒高度重叠调用在2秒内切换HR基线', () => {
    const tracker = new VitalsTracker({
        warmupWindows: 1,
        jumpConfirmWindows: 3,
        jumpConfirmMinSpanMs: 10000,
        hrJumpThreshold: 25
    });
    const baseline = tracker.update(trackerSample(82), null, { timestampMs: 0 });
    assert.equal(baseline.HR_bpm, 82, '测试夹具应先建立原基线');

    for (const timestampMs of [1000, 2000, 3000]) {
        const overlapping = tracker.update(trackerSample(150), null, { timestampMs });
        assert.equal(
            overlapping.HR_bpm,
            82,
            `${timestampMs}ms时高度重叠窗口不能确认大跳变`
        );
    }

    const independent = tracker.update(trackerSample(150), null, { timestampMs: 12000 });
    assert.equal(independent.HR_bpm, 150, '大跳变累计到独立时间跨度后应允许切换基线');
    assert.equal(independent.valid.HR, true);
});

