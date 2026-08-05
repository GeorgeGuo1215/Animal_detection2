'use strict';

const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const test = require('node:test');
const vm = require('node:vm');

const { generatePhaseModulatedIq } = require('./synthetic-iq.cjs');

const SAMPLE_RATE_HZ = 50;
const HEART_RATE_BPM = 90;
const RESPIRATORY_RATE_BPM = 18;
const MOTION_RATE_BPM = 150;
const HEART_TOLERANCE_BPM = 5;
const RESPIRATORY_TOLERANCE_BPM = 3;
const EXTRACTOR_OPTIONS = Object.freeze({
    hrLow: 35 / 60,
    hrHigh: 260 / 60,
    rrLow: 6 / 60,
    rrHigh: 60 / 60,
    hrMinFromRrRatio: 2.2,
    minSamples: 128,
    motionMaskMaxFraction: 0.45,
    motionMaskMaxRunSec: 6
});

function loadVitalsExtractor() {
    const sourcePath = path.resolve(__dirname, '..', 'cwt-vitals.js');
    const source = fs.readFileSync(sourcePath, 'utf8');
    const sandbox = { window: {}, console };
    vm.createContext(sandbox);
    vm.runInContext(source, sandbox, { filename: sourcePath });
    assert.equal(typeof sandbox.window.VitalsExtractor?.create, 'function');
    return sandbox.window.VitalsExtractor;
}

function assertWithin(actual, expected, tolerance, label) {
    assert.ok(Number.isFinite(actual), `${label} must be finite; received ${String(actual)}`);
    const error = Math.abs(actual - expected);
    assert.ok(
        error <= tolerance,
        `${label}: expected ${expected} +/- ${tolerance}, received ${actual} (error ${error.toFixed(3)})`
    );
}

function injectMotionBurst(fixture, {
    startSecond,
    durationSeconds,
    motionRateBpm = MOTION_RATE_BPM,
    maskPadBeforeSeconds = 0.25,
    maskPadAfterSeconds = 0.50
}) {
    const iData = fixture.iData.slice();
    const qData = fixture.qData.slice();
    const sampleRate = fixture.sampleRateHz;
    const burstStart = Math.round(startSecond * sampleRate);
    const burstEnd = Math.min(iData.length, Math.round((startSecond + durationSeconds) * sampleRate));
    const motionHz = motionRateBpm / 60;

    for (let index = burstStart; index < burstEnd; index++) {
        const localTime = (index - burstStart) / sampleRate;
        const oscillation = Math.sin(2 * Math.PI * motionHz * localTime);
        const quadrature = Math.cos(2 * Math.PI * motionHz * localTime + 0.31);
        const impact = (index - burstStart) % 9 < 4 ? 1 : -1;
        // A collar bump changes both apparent phase and receiver voltage. The
        // amplitudes deliberately exceed the cardiac displacement by orders of
        // magnitude, while remaining finite input values.
        iData[index] += 0.95 * oscillation + 0.28 * impact;
        qData[index] += 0.82 * quadrature - 0.24 * impact;
    }

    const cleanMask = new Array(iData.length).fill(true);
    const maskStart = Math.max(0, burstStart - Math.round(maskPadBeforeSeconds * sampleRate));
    const maskEnd = Math.min(iData.length, burstEnd + Math.round(maskPadAfterSeconds * sampleRate));
    for (let index = maskStart; index < maskEnd; index++) cleanMask[index] = false;

    return {
        iData,
        qData,
        cleanMask,
        dirtySamples: maskEnd - maskStart
    };
}

function buildMostlyDirtyFixture() {
    const fixture = generatePhaseModulatedIq({
        heartRateBpm: HEART_RATE_BPM,
        respiratoryRateBpm: RESPIRATORY_RATE_BPM,
        sampleRateHz: SAMPLE_RATE_HZ,
        durationSeconds: 30,
        seed: 0x5045bad
    });
    const iData = fixture.iData.slice();
    const qData = fixture.qData.slice();
    const cleanMask = new Array(iData.length).fill(true);
    const blockSamples = Math.round(0.5 * SAMPLE_RATE_HZ);

    // Alternating dirty/clean half-second blocks yield exactly 50% dirty data.
    // Each run is short, so rejection is specifically caused by total dirty
    // fraction >45%, not by the independent six-second run limit.
    for (let blockStart = 0, blockIndex = 0;
        blockStart < iData.length;
        blockStart += blockSamples, blockIndex++) {
        if (blockIndex % 2 !== 0) continue;
        const blockEnd = Math.min(iData.length, blockStart + blockSamples);
        for (let index = blockStart; index < blockEnd; index++) {
            cleanMask[index] = false;
            const time = index / SAMPLE_RATE_HZ;
            iData[index] += 0.9 * Math.sin(2 * Math.PI * 2.7 * time);
            qData[index] += 0.8 * Math.cos(2 * Math.PI * 2.7 * time + 0.4);
        }
    }
    return { iData, qData, cleanMask };
}

const vitalsApi = loadVitalsExtractor();

for (const method of ['freqBP', 'cwt']) {
    test(`${method}: a masked 0.8 s motion burst is repaired without replacing 90/18`, () => {
        const fixture = generatePhaseModulatedIq({
            heartRateBpm: HEART_RATE_BPM,
            respiratoryRateBpm: RESPIRATORY_RATE_BPM,
            sampleRateHz: SAMPLE_RATE_HZ,
            durationSeconds: 30,
            seed: method === 'cwt' ? 0xc070018 : 0xf090018
        });
        const corrupted = injectMotionBurst(fixture, {
            startSecond: 14.4,
            durationSeconds: 0.8
        });
        const result = vitalsApi.create(method, SAMPLE_RATE_HZ, EXTRACTOR_OPTIONS).run(
            corrupted.iData,
            corrupted.qData,
            SAMPLE_RATE_HZ,
            { motionCleanMask: corrupted.cleanMask }
        );

        assertWithin(result.HR_bpm, HEART_RATE_BPM, HEART_TOLERANCE_BPM, `${method} repaired HR`);
        assertWithin(
            result.RR_bpm,
            RESPIRATORY_RATE_BPM,
            RESPIRATORY_TOLERANCE_BPM,
            `${method} repaired RR`
        );
        assert.ok(
            Math.abs(result.HR_bpm - MOTION_RATE_BPM) > 20,
            `${method} adopted the ${MOTION_RATE_BPM} bpm movement frequency as HR`
        );

        const repair = result.quality?.motionRepair;
        assert.ok(repair, `${method} must report motion-repair diagnostics`);
        assert.equal(repair.applied, true);
        assert.equal(repair.reason, 'local-motion-interpolated');
        assert.equal(repair.repairedRuns, 1);
        assert.ok(repair.dirtyFraction > 0 && repair.dirtyFraction < 0.10);
        assert.ok(repair.longestDirtySec >= 0.8);
        assert.equal(result.valid, true);
    });

    test(`${method}: more than 45% motion contamination is invalid, never a fabricated peak`, () => {
        const corrupted = buildMostlyDirtyFixture();
        const result = vitalsApi.create(method, SAMPLE_RATE_HZ, EXTRACTOR_OPTIONS).run(
            corrupted.iData,
            corrupted.qData,
            SAMPLE_RATE_HZ,
            { motionCleanMask: corrupted.cleanMask }
        );

        assert.equal(result.valid, false);
        assert.equal(Number.isNaN(result.HR_bpm), true);
        assert.equal(Number.isNaN(result.RR_bpm), true);
        assert.equal(result.HR_confidence, 0);
        assert.equal(result.RR_confidence, 0);
        assert.match(result.reason, /运动污染占比过高/);

        const repair = result.quality?.motionRepair;
        assert.ok(repair, `${method} invalid result must preserve repair diagnostics`);
        assert.equal(repair.applied, false);
        assert.equal(repair.reason, 'too-much-motion-for-repair');
        assert.ok(repair.dirtyFraction > 0.45);
        assert.ok(repair.longestDirtySec <= 0.5 + 1 / SAMPLE_RATE_HZ);
    });
}
