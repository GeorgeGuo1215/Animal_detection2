'use strict';

const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const test = require('node:test');
const vm = require('node:vm');

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
    return { RadarWebApp: sandbox.__RadarWebApp, sandbox };
}

const { RadarWebApp, sandbox } = loadRadarWebApp();

function assertApprox(actual, expected, tolerance = 1) {
    assert.ok(Number.isFinite(actual), `expected a finite rate, received ${String(actual)}`);
    assert.ok(
        Math.abs(actual - expected) <= tolerance,
        `expected ${expected} +/- ${tolerance} Hz, received ${actual}`
    );
}

function bareApp({ configuredFs = 50 } = {}) {
    const app = Object.create(RadarWebApp.prototype);
    app.processor = { fs: configuredFs };
    app.bleConnected = true;
    app._simInterval = null;
    app.bleStats = { received: 0, startRxTs: 0 };
    app.bleBufferTimestamps = [];
    app.bleBufferDeviceTimestamps = [];
    app.bleBufferArrivalTimestamps = [];
    app.bleBufferNotificationIds = [];
    return app;
}

test('批量 BLE 每通知两帧时，生命体征采样率按帧率 50 Hz 而非通知率 25 Hz', () => {
    const app = bareApp({ configuredFs: 50 });

    // 25 notifications/s, two samples in each notification. Legacy frames are
    // stamped with Date.now(), so both samples in one batch share a timestamp.
    for (let batch = 0; batch < 30; batch++) {
        const arrivalMs = batch * 40;
        app.bleBufferTimestamps.push(arrivalMs, arrivalMs);
    }

    assertApprox(app._estimateVitalsFsFromTimestamps(), 50);
    assertApprox(app._getVitalsFs(), 50);
});

test('同一BLE批次跨过1ms时钟刻度时仍按两帧计算', () => {
    const app = bareApp({ configuredFs: 50 });
    for (let batch = 0; batch < 30; batch++) {
        const arrivalMs = batch * 40;
        app.bleBufferTimestamps.push(arrivalMs, arrivalMs + 1);
    }

    assertApprox(app._estimateVitalsFsFromTimestamps(), 50);
    assertApprox(app._getVitalsFs(), 50);
});

test('每通知帧数1/1/1/5波动时按累计帧斜率得到50Hz而不是25Hz中位数', () => {
    const app = bareApp({ configuredFs: 50 });
    const framesPerNotification = [1, 1, 1, 5];
    for (let notification = 0; notification < 40; notification++) {
        const arrivalMs = notification * 40;
        const frameCount = framesPerNotification[notification % framesPerNotification.length];
        for (let frame = 0; frame < frameCount; frame++) {
            app.bleBufferTimestamps.push(arrivalMs);
        }
    }

    assertApprox(app._estimateVitalsFsFromTimestamps(), 50, 1.5);
    assertApprox(app._getVitalsFs(), 50, 1.5);
});

test('通知内解析延迟达到8ms也不能把50Hz帧流误认为通知率', () => {
    const app = bareApp({ configuredFs: 50 });
    const decodeDelays = [0, 1, 3, 8];
    for (let notification = 0; notification < 40; notification++) {
        const arrivalMs = notification * 40;
        app.bleBufferArrivalTimestamps.push(
            arrivalMs + decodeDelays[notification % decodeDelays.length],
            arrivalMs + 8
        );
        app.bleBufferNotificationIds.push(notification, notification);
    }

    assertApprox(app._estimateVitalsFsFromTimestamps(), 50, 2);
    assertApprox(app.vitalsFsEstimateInfo.packing.meanFramesPerNotification, 2, 0.01);
});

test('时间轴与累计帧率出现2比1冲突时禁止采用低采样率', () => {
    const app = bareApp({ configuredFs: 50 });
    app.bleStats.received = 100;
    app._estimateVitalsFsFromTimestamps = () => {
        app.vitalsFsEstimateInfo = { source: 'test-low-timeline', medianBatchSize: 1 };
        return 25;
    };
    app._estimateCurrentBleFs = () => 50;

    assertApprox(app._getVitalsFs(), 50);
    assert.equal(app.vitalsFsDiagnostic.antiHalfRateCorrection, true);
    assert.equal(app.vitalsFsDiagnostic.source, 'anti-half-rate-consensus');
});

test('近期时间轴50Hz时拒绝陈旧的100Hz连接平均，避免心率翻倍', () => {
    const app = bareApp({ configuredFs: 100 });
    app.bleStats.received = 500;
    app.bleBufferArrivalTimestamps = Array.from({ length: 120 }, (_, index) => index * 20);
    app.bleBufferI = Array.from({ length: 120 }, () => 1);
    app.bleBufferQ = Array.from({ length: 120 }, () => 1);
    app.bleBufferIMU_X = [];
    app.bleBufferIMU_Y = [];
    app.bleBufferIMU_Z = [];
    app.bleBufferACC_X = [];
    app.bleBufferACC_Y = [];
    app.bleBufferACC_Z = [];
    app.bleBufferTemperature = [];
    app.vitalsSamplingEpoch = 2;
    app.vitalsHrHistory = [80, 80, 80];
    app.vitalsRrHistory = [18, 18, 18];
    app.addBLELog = () => {};
    app._estimateCurrentBleFs = () => 100;

    assertApprox(app._getVitalsFs(), 50, 1);
    assert.equal(app.processor.fs, 50, '新处理率应同步为近期实测50Hz');
    assert.equal(app.vitalsSamplingEpoch, 3, '采样率阶跃必须开新epoch');
    assert.equal(app.vitalsFsDiagnostic.staleConnectionRateRejected, true);
    assert.equal(app.vitalsFsDiagnostic.source, 'recent-timeline-over-stale-connection');
});

test('逐样本设备时间戳显示真实 100 Hz 时，不得沿用配置值 50 Hz 缩小 BPM', () => {
    const app = bareApp({ configuredFs: 50 });
    app.bleBufferTimestamps = Array.from({ length: 100 }, (_, index) => 1_750_000_000_000 + index * 10);

    assertApprox(app._estimateVitalsFsFromTimestamps(), 100);
    assertApprox(app._getVitalsFs(), 100);
});

test('非法设备日期不得被JavaScript自动滚动成错误的采样时钟', () => {
    const app = bareApp();
    assert.equal(app._parseBleProtocolTimestamp('250231120000000'), null, '2月31日必须拒绝');
    assert.ok(Number.isFinite(app._parseBleProtocolTimestamp('250228120000000')));
});

test('自动配置实测 100 Hz 时采用实测值，不把它当成 50 Hz 配置成功', async (t) => {
    const app = bareApp({ configuredFs: 50 });
    app.bleTargetFs = 50;
    app.bleConfigStatus = 'pending';
    app.activityMonitor = { fs: 50 };
    app.sleepMonitor = { fs: 50 };
    app.addBLELog = () => {};
    app._updateConfigStatusUI = () => {};
    app.sendTcycleCommand = async () => {};

    const originalDate = sandbox.Date;
    const originalSetTimeout = sandbox.setTimeout;
    const originalBle = sandbox.window.BLE;
    let nowMs = 1_750_000_000_000;
    class FakeDate extends Date {
        static now() { return nowMs; }
    }

    sandbox.Date = FakeDate;
    sandbox.window.BLE = { writeCharacteristic: {}, send: async () => {} };
    sandbox.setTimeout = (resolve, delayMs) => {
        nowMs += delayMs;
        if (delayMs === 2500) app.bleStats.received += 250;
        resolve();
        return 1;
    };
    t.after(() => {
        sandbox.Date = originalDate;
        sandbox.setTimeout = originalSetTimeout;
        sandbox.window.BLE = originalBle;
    });

    await app._autoConfigTcycleWithRetry(1);

    assert.equal(app.bleTargetFs, 50, '指令目标仍应为 50 Hz');
    assert.equal(app.processor.fs, 100, '生命体征换算必须采用实测 100 Hz');
    assert.equal(app.activityMonitor.fs, 100);
    assert.equal(app.sleepMonitor.fs, 100);
    assert.equal(app.bleConfigStatus, 'configured');
});

test('BLE已连接时设置面板的默认50Hz不能覆盖已验证100Hz实时处理率', (t) => {
    const app = bareApp({ configuredFs: 100 });
    app.offlineProcessor = { fs: 100 };
    app.offlineSamplingRate = 100;
    app._getVitalsFs = () => 100;
    app._getVitalsOptions = () => ({ hrLow: 35 / 60, hrHigh: 260 / 60, rrLow: 6 / 60, rrHigh: 1 });
    app._initVitalsExtractor = () => null;
    app._resetVitalsStability = () => {};
    app.addBLELog = () => {};
    app.showMessage = () => {};
    app.toggleSettings = () => {};
    app.vitalsMinConfidence = 0.28;
    app.historyMaxLength = 3;
    app.vitalsHoldMaxMs = 30000;
    app.heartRateDelta = 25;

    const originalGetElementById = sandbox.document.getElementById;
    const values = {
        samplingRate: '50',
        heartRateSmoothing: '3',
        heartRateDelta: '25'
    };
    sandbox.document.getElementById = id => Object.hasOwn(values, id)
        ? { value: values[id] }
        : null;
    t.after(() => { sandbox.document.getElementById = originalGetElementById; });

    app.applySettings();

    assert.equal(app.processor.fs, 100, 'live processor must remain at verified 100 Hz');
    assert.equal(app.offlineProcessor.fs, 50, 'the input remains available for offline files');
    assert.equal(app.offlineSamplingRate, 50);
});

test('确认采样率阶跃时开启新epoch并清除混合速率窗口', () => {
    const app = bareApp({ configuredFs: 50 });
    app.bleBufferI = [1, 2, 3];
    app.bleBufferQ = [4, 5, 6];
    app.bleBufferIMU_X = [0, 0, 0];
    app.bleBufferIMU_Y = [0, 0, 0];
    app.bleBufferIMU_Z = [0, 0, 0];
    app.bleBufferACC_X = [0, 0, 0];
    app.bleBufferACC_Y = [0, 0, 0];
    app.bleBufferACC_Z = [1, 1, 1];
    app.bleBufferTemperature = [30, 30, 30];
    app.bleBufferTimestamps = [0, 20, 40];
    app.bleBufferDeviceTimestamps = [0, 20, 40];
    app.bleBufferArrivalTimestamps = [0, 20, 40];
    app.bleBufferNotificationIds = [1, 2, 3];
    app.vitalsSamplingEpoch = 4;
    app.vitalsTracker = { preserved: true };
    app.addBLELog = () => {};

    const trackerBefore = app.vitalsTracker;
    app._applyBleSamplingRate(100, { updateTarget: false });

    assert.equal(app.processor.fs, 100);
    assert.equal(app.vitalsSamplingEpoch, 5);
    assert.equal(app.bleBufferI.length, 0);
    assert.equal(app.bleBufferQ.length, 0);
    assert.equal(app.bleBufferTimestamps.length, 0);
    assert.equal(app.bleBufferArrivalTimestamps.length, 0);
    assert.equal(app.vitalsTracker, trackerBefore, 'reliable tracker baseline is held across the epoch');
});

test('epoch切换后旧心率提示立即变为held且年龄实时增长', (t) => {
    const app = bareApp({ configuredFs: 50 });
    app.bleBufferI = [1];
    app.bleBufferQ = [1];
    app.bleBufferIMU_X = [];
    app.bleBufferIMU_Y = [];
    app.bleBufferIMU_Z = [];
    app.bleBufferACC_X = [];
    app.bleBufferACC_Y = [];
    app.bleBufferACC_Z = [];
    app.bleBufferTemperature = [];
    app.vitalsSamplingEpoch = 7;
    app.vitalsHoldMaxMs = 30000;
    app.vitalsHrHistory = [40, 40, 40];
    app.vitalsRrHistory = [18, 18, 18];
    app.vitalsLastReliable = {
        HR: { value: 40, timestampMs: 1_750_000_000_000, confidence: 0.9 },
        RR: { value: 18, timestampMs: 1_750_000_000_000, confidence: 0.9 }
    };
    app.vitalsDisplayState = {
        timestampMs: 1_750_000_000_000,
        HR: { value: 40, currentValue: 40, status: 'fresh', ageMs: 0, isFresh: true },
        RR: { value: 18, currentValue: 18, status: 'fresh', ageMs: 0, isFresh: true }
    };
    app.addBLELog = () => {};

    const originalDate = sandbox.Date;
    let nowMs = 1_750_000_005_000;
    class FakeDate extends Date { static now() { return nowMs; } }
    sandbox.Date = FakeDate;
    t.after(() => { sandbox.Date = originalDate; });

    app._beginVitalsSamplingEpoch('测试');
    assert.equal(app.vitalsDisplayState.HR.status, 'held');
    assert.equal(app.vitalsDisplayState.HR.isFresh, false);
    assert.equal(app.vitalsHrHistory.length, 0);
    let hint = app._buildVitalsHintMeta('HR');
    assert.equal(hint.status, 'held');
    assert.equal(hint.ageMs, 5000);
    assert.equal(hint.freshWindowCount, 0);

    nowMs += 8000;
    hint = app._buildVitalsHintMeta('HR');
    assert.equal(hint.ageMs, 13000, '提示年龄不能冻结在epoch切换时刻');
});
