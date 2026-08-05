'use strict';

const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const test = require('node:test');
const vm = require('node:vm');
const VitalsTracker = require('../vitals-tracker.js');

const SAMPLE_RATE_HZ = 50;

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

function deterministicNoise(index, amplitude, phase = 0) {
    return amplitude * (
        0.62 * Math.sin(index * 0.37 + phase)
        + 0.28 * Math.sin(index * 1.13 + phase * 0.7)
        + 0.10 * Math.cos(index * 2.03 - phase)
    );
}

function makeImuWindow({
    seconds = 20,
    accNoise = 0.003,
    gyroNoise = 0.20,
    motionStartSec = NaN,
    motionEndSec = NaN,
    motionAcc = 0,
    motionGyro = 0
} = {}) {
    const count = Math.round(seconds * SAMPLE_RATE_HZ);
    const accX = new Array(count);
    const accY = new Array(count);
    const accZ = new Array(count);
    const gyroX = new Array(count);
    const gyroY = new Array(count);
    const gyroZ = new Array(count);

    for (let index = 0; index < count; index++) {
        const time = index / SAMPLE_RATE_HZ;
        const moving = time >= motionStartSec && time < motionEndSec;
        // During movement use non-constant multi-axis energy. A constant offset
        // would correctly be removed as collar orientation/gravity, not motion.
        const burst = moving ? Math.sin(index * 1.71) : 0;
        const burst2 = moving ? Math.cos(index * 1.17 + 0.4) : 0;
        accX[index] = deterministicNoise(index, accNoise, 0.1) + motionAcc * burst;
        accY[index] = deterministicNoise(index, accNoise, 1.7) + motionAcc * 0.75 * burst2;
        accZ[index] = 1 + deterministicNoise(index, accNoise, 3.1) + motionAcc * 0.55 * burst;
        gyroX[index] = deterministicNoise(index, gyroNoise, 0.8) + motionGyro * burst;
        gyroY[index] = deterministicNoise(index, gyroNoise, 2.2) + motionGyro * 0.8 * burst2;
        gyroZ[index] = deterministicNoise(index, gyroNoise, 4.0) + motionGyro * 0.6 * burst;
    }
    return { accX, accY, accZ, gyroX, gyroY, gyroZ, count };
}

function bareApp(windowData) {
    const app = Object.create(RadarWebApp.prototype);
    app.bleBufferACC_X = windowData.accX;
    app.bleBufferACC_Y = windowData.accY;
    app.bleBufferACC_Z = windowData.accZ;
    app.bleBufferIMU_X = windowData.gyroX;
    app.bleBufferIMU_Y = windowData.gyroY;
    app.bleBufferIMU_Z = windowData.gyroZ;
    return app;
}

function vitalSample(hr, rr, hrConfidence = 0.93, rrConfidence = 0.93) {
    return {
        HR_bpm: hr,
        RR_bpm: rr,
        HR_confidence: hrConfidence,
        RR_confidence: rrConfidence
    };
}

function motionContext(timestampMs, motionState = 'stationary', motionQuality = 0.95) {
    return { timestampMs, motionState, motionQuality };
}

function bareVitalsApp() {
    const app = Object.create(RadarWebApp.prototype);
    app.vitalsMinConfidence = 0.28;
    app.vitalsWarmupMinCycles = 2;
    app.vitalsAcceptedCycles = 0;
    app.vitalsLastStableHR = null;
    app.vitalsLastStableRR = null;
    app.vitalsHrHistory = [];
    app.vitalsRrHistory = [];
    app.historyMaxLength = 3;
    app.vitalsTracker = new VitalsTracker({
        minConfidence: app.vitalsMinConfidence,
        warmupWindows: 2,
        smoothWindows: 3,
        jumpConfirmWindows: 3,
        holdMaxMs: 30000,
        hrJumpThreshold: 25,
        rrJumpThreshold: 8
    });
    app._getVitalsOptions = () => ({
        hrLow: 35 / 60,
        hrHigh: 260 / 60,
        rrLow: 6 / 60,
        rrHigh: 60 / 60
    });
    return app;
}

function warmVitalsApp(app, hr = 86, rr = 18) {
    const first = app._stabilizeVitalsResult(
        vitalSample(hr, rr),
        null,
        motionContext(0)
    );
    assert.equal(first.hrValid, false);
    assert.equal(first.rrValid, false);

    const ready = app._stabilizeVitalsResult(
        vitalSample(hr, rr),
        null,
        motionContext(1000)
    );
    assert.equal(ready.stableHR, hr);
    assert.equal(ready.stableRR, rr);
    assert.deepEqual(ready.status, { HR: 'fresh', RR: 'fresh' });
    return ready;
}

function assertMotionContract(analysis, sampleCount) {
    assert.ok(['stationary', 'recovering', 'moving'].includes(analysis.state));
    assert.ok(analysis.quality >= 0 && analysis.quality <= 1);
    assert.ok(analysis.cleanFraction >= 0 && analysis.cleanFraction <= 1);
    assert.ok(analysis.recentMotionFraction >= 0 && analysis.recentMotionFraction <= 1);
    assert.equal(analysis.cleanMask.length, sampleCount);
    for (const key of ['accClean', 'accStrong', 'gyroClean', 'gyroStrong']) {
        assert.ok(Number.isFinite(analysis.thresholds[key]), `missing threshold ${key}`);
        assert.ok(analysis.thresholds[key] >= 0, `negative threshold ${key}`);
    }
    assert.ok(analysis.thresholds.accStrong > analysis.thresholds.accClean);
    assert.ok(analysis.thresholds.gyroStrong > analysis.thresholds.gyroClean);
}

test('stationary sensor noise remains usable for continuous vital monitoring', () => {
    const data = makeImuWindow();
    const app = bareApp(data);
    const analysis = app._analyzeVitalsMotion(data.count, SAMPLE_RATE_HZ);

    assertMotionContract(analysis, data.count);
    assert.equal(analysis.state, 'stationary');
    assert.ok(analysis.quality >= 0.65, `stationary quality ${analysis.quality}`);
    assert.ok(analysis.cleanFraction >= 0.85, `stationary clean fraction ${analysis.cleanFraction}`);
    assert.ok(analysis.motionRmsG < analysis.thresholds.accClean);
});

test('one sub-second light movement is locally masked instead of blanking the whole window', () => {
    const data = makeImuWindow({
        motionStartSec: 8,
        motionEndSec: 8.6,
        motionAcc: 0.055,
        motionGyro: 7
    });
    const app = bareApp(data);
    const analysis = app._analyzeVitalsMotion(data.count, SAMPLE_RATE_HZ);

    assertMotionContract(analysis, data.count);
    assert.notEqual(analysis.state, 'moving');
    assert.ok(analysis.quality > 0, 'a brief movement must not zero the entire-window quality');
    assert.ok(analysis.cleanFraction >= 0.75, `clean fraction ${analysis.cleanFraction}`);
    assert.ok(analysis.cleanMask.some(value => !value), 'the disturbed local segment should be rejected');
    assert.ok(analysis.cleanMask.some(Boolean), 'clean samples surrounding the disturbance must remain usable');
});

test('a collar orientation step does not contaminate the remaining full window', () => {
    const data = makeImuWindow();
    const transitionStart = 8 * SAMPLE_RATE_HZ;
    const transitionEnd = transitionStart + Math.round(0.6 * SAMPLE_RATE_HZ);
    for (let index = transitionStart; index < data.count; index++) {
        const progress = Math.min(1, (index - transitionStart) / (transitionEnd - transitionStart));
        const angle = progress * Math.PI / 3;
        data.accX[index] = Math.sin(angle) + deterministicNoise(index, 0.003, 0.1);
        data.accY[index] = deterministicNoise(index, 0.003, 1.7);
        data.accZ[index] = Math.cos(angle) + deterministicNoise(index, 0.003, 3.1);
    }

    const analysis = bareApp(data)._analyzeVitalsMotion(data.count, SAMPLE_RATE_HZ);

    assertMotionContract(analysis, data.count);
    assert.notEqual(analysis.state, 'moving');
    assert.ok(analysis.quality > 0);
    assert.ok(analysis.recentMotionFraction < 0.10);
    assert.ok(analysis.cleanFraction >= 0.70, `clean fraction ${analysis.cleanFraction}`);
});

test('recent continuous strong movement is moving and cannot authorize a new vital estimate', () => {
    const data = makeImuWindow({
        motionStartSec: 17,
        motionEndSec: 20,
        motionAcc: 0.35,
        motionGyro: 65
    });
    const app = bareApp(data);
    const analysis = app._analyzeVitalsMotion(data.count, SAMPLE_RATE_HZ);

    assertMotionContract(analysis, data.count);
    assert.equal(analysis.state, 'moving');
    assert.ok(analysis.recentMotionFraction >= 0.5, `recent motion ${analysis.recentMotionFraction}`);
    assert.ok(analysis.quality < 0.5, `moving quality ${analysis.quality}`);
});

test('two clean seconds after a burst enter recovery and regain non-zero quality quickly', () => {
    const movingData = makeImuWindow({
        motionStartSec: 17,
        motionEndSec: 20,
        motionAcc: 0.35,
        motionGyro: 65
    });
    const moving = bareApp(movingData)._analyzeVitalsMotion(movingData.count, SAMPLE_RATE_HZ);

    const recoveryData = makeImuWindow({
        motionStartSec: 14,
        motionEndSec: 18,
        motionAcc: 0.35,
        motionGyro: 65
    });
    const recovered = bareApp(recoveryData)._analyzeVitalsMotion(recoveryData.count, SAMPLE_RATE_HZ);

    assertMotionContract(recovered, recoveryData.count);
    assert.ok(
        recovered.state === 'recovering' || recovered.state === 'stationary',
        `unexpected recovery state ${recovered.state}`
    );
    assert.ok(recovered.quality > 0, 'clean tail must restore usable quality');
    assert.ok(
        recovered.recentMotionFraction < moving.recentMotionFraction,
        'recent clean samples must reduce recent-motion occupancy'
    );
});

test('stationary-noise thresholds adapt upward without classifying baseline noise as motion', () => {
    const quietData = makeImuWindow({ accNoise: 0.002, gyroNoise: 0.12 });
    const noisyData = makeImuWindow({ accNoise: 0.009, gyroNoise: 0.75 });
    const quiet = bareApp(quietData)._analyzeVitalsMotion(quietData.count, SAMPLE_RATE_HZ);
    const noisy = bareApp(noisyData)._analyzeVitalsMotion(noisyData.count, SAMPLE_RATE_HZ);

    assertMotionContract(quiet, quietData.count);
    assertMotionContract(noisy, noisyData.count);
    assert.equal(quiet.state, 'stationary');
    assert.equal(noisy.state, 'stationary');
    assert.ok(noisy.thresholds.accClean > quiet.thresholds.accClean);
    assert.ok(noisy.thresholds.gyroClean > quiet.thresholds.gyroClean);
});

test('accelerometer values in m/s2 are normalized before applying g thresholds', () => {
    const data = makeImuWindow();
    for (const axis of [data.accX, data.accY, data.accZ]) {
        for (let index = 0; index < axis.length; index++) axis[index] *= 9.80665;
    }
    const analysis = bareApp(data)._analyzeVitalsMotion(data.count, SAMPLE_RATE_HZ);

    assertMotionContract(analysis, data.count);
    assert.equal(analysis.unit, 'm/s2');
    assert.ok(Math.abs(analysis.unitScale - 1 / 9.80665) < 1e-8);
    assert.equal(analysis.state, 'stationary');
    assert.ok(analysis.quality >= 0.65);
});

test('app stabilization forwards movement context and holds rather than adopting strong false peaks', () => {
    const app = bareVitalsApp();
    warmVitalsApp(app);

    const held = app._stabilizeVitalsResult(
        vitalSample(190, 52, 0.99, 0.99),
        vitalSample(188, 51, 0.99, 0.99),
        motionContext(3000, 'moving', 0.01)
    );

    assert.equal(held.stableHR, 86);
    assert.equal(held.stableRR, 18);
    assert.deepEqual(held.held, { HR: true, RR: true });
    assert.deepEqual(held.fresh, { HR: false, RR: false });
    assert.deepEqual(held.status, { HR: 'held', RR: 'held' });
    assert.deepEqual(held.ageMs, { HR: 2000, RR: 2000 });
});

test('app stabilization marks the first nearby clean result fresh after movement', () => {
    const app = bareVitalsApp();
    warmVitalsApp(app);
    app._stabilizeVitalsResult(
        vitalSample(190, 52),
        null,
        motionContext(3000, 'moving', 0.01)
    );

    const recovered = app._stabilizeVitalsResult(
        vitalSample(87, 19, 0.95, 0.95),
        null,
        motionContext(4000, 'stationary', 0.92)
    );

    assert.ok(Math.abs(recovered.stableHR - 86) <= 1);
    assert.ok(Math.abs(recovered.stableRR - 18) <= 1);
    assert.deepEqual(recovered.held, { HR: false, RR: false });
    assert.deepEqual(recovered.fresh, { HR: true, RR: true });
    assert.deepEqual(recovered.status, { HR: 'fresh', RR: 'fresh' });
    assert.deepEqual(recovered.ageMs, { HR: 0, RR: 0 });
});

test('app stabilization keeps HR and RR display freshness independent', () => {
    const app = bareVitalsApp();
    warmVitalsApp(app);

    const rrHeld = app._stabilizeVitalsResult(
        vitalSample(87, NaN, 0.95, 0),
        null,
        motionContext(2000)
    );
    assert.ok(Number.isFinite(rrHeld.stableHR));
    assert.equal(rrHeld.stableRR, 18);
    assert.deepEqual(rrHeld.status, { HR: 'fresh', RR: 'held' });
    assert.deepEqual(rrHeld.fresh, { HR: true, RR: false });
    assert.deepEqual(rrHeld.held, { HR: false, RR: true });

    const hrHeld = app._stabilizeVitalsResult(
        vitalSample(NaN, 19, 0, 0.95),
        null,
        motionContext(3000)
    );
    assert.ok(Number.isFinite(hrHeld.stableHR));
    assert.ok(Number.isFinite(hrHeld.stableRR));
    assert.deepEqual(hrHeld.status, { HR: 'held', RR: 'fresh' });
    assert.deepEqual(hrHeld.fresh, { HR: false, RR: true });
    assert.deepEqual(hrHeld.held, { HR: true, RR: false });
});
