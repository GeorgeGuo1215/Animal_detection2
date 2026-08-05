'use strict';

const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const test = require('node:test');
const vm = require('node:vm');

const { generatePhaseModulatedIq } = require('./synthetic-iq.cjs');

const SAMPLE_RATE_HZ = 50;
const DURATION_SECONDS = 60;
const MAX_HEART_ERROR_BPM = 5;
const MAX_RESPIRATORY_ERROR_BPM = 3;

// The regression target intentionally covers canine rates beyond the old
// 0.5-3.0 Hz / 0.1-0.5 Hz defaults. Passing options is part of the existing
// create(method, fs, options) compatibility surface.
const EXTRACTOR_OPTIONS = Object.freeze({
    hrLow: 35 / 60,
    hrHigh: 260 / 60,
    rrLow: 6 / 60,
    rrHigh: 60 / 60,
    hrMinFromRrRatio: 2.2,
    minSamples: 128
});

const CLEAN_CASES = Object.freeze([
    // Explicit boundary coverage for the configurable technical range.
    { heartRateBpm: 90, respiratoryRateBpm: 6 },
    { heartRateBpm: 35, respiratoryRateBpm: 18 },
    { heartRateBpm: 45, respiratoryRateBpm: 18 },
    { heartRateBpm: 75, respiratoryRateBpm: 30 },
    // RR=45 lies inside the HR search band. This case catches an extractor
    // that reports the respiration fundamental as a 45 bpm heartbeat.
    { heartRateBpm: 120, respiratoryRateBpm: 45 },
    { heartRateBpm: 180, respiratoryRateBpm: 8 },
    { heartRateBpm: 220, respiratoryRateBpm: 18 },
    { heartRateBpm: 260, respiratoryRateBpm: 18 },
    { heartRateBpm: 90, respiratoryRateBpm: 60 }
]);

function loadVitalsExtractor() {
    const sourcePath = path.resolve(__dirname, '..', 'cwt-vitals.js');
    const source = fs.readFileSync(sourcePath, 'utf8');
    const sandbox = {
        window: {},
        console
    };
    vm.createContext(sandbox);
    vm.runInContext(source, sandbox, { filename: sourcePath });

    const api = sandbox.window.VitalsExtractor;
    assert.ok(api, 'cwt-vitals.js must publish window.VitalsExtractor');
    assert.equal(typeof api.create, 'function', 'VitalsExtractor.create must remain callable');
    return api;
}

function assertFiniteEstimate(actual, label, result) {
    assert.ok(
        Number.isFinite(actual),
        `${label} must be finite; received ${String(actual)} from ${JSON.stringify({
            reason: result && result.reason,
            method: result && result.method,
            samples: result && result.samples,
            fs: result && result.fs
        })}`
    );
}

function assertWithin(actual, expected, tolerance, label, result) {
    assertFiniteEstimate(actual, label, result);
    const error = Math.abs(actual - expected);
    assert.ok(
        error <= tolerance,
        `${label} expected ${expected} +/- ${tolerance}, received ${actual} (error ${error.toFixed(3)})`
    );
}

function isInvalidEstimate(value) {
    return value === null || value === undefined || !Number.isFinite(value);
}

function assertNoFabricatedBaseline(result, scenario) {
    const forbiddenDefaults = [70, 72, 80];
    for (const [name, value] of [['HR_bpm', result.HR_bpm], ['RR_bpm', result.RR_bpm]]) {
        assert.ok(
            isInvalidEstimate(value),
            `${scenario}: ${name} must be invalid when no rate is measurable; received ${String(value)}`
        );
        assert.ok(
            !forbiddenDefaults.includes(value),
            `${scenario}: ${name} must not fabricate the historical 70/72/80 bpm baseline`
        );
    }
}

const vitalsApi = loadVitalsExtractor();

for (const method of ['freqBP', 'cwt']) {
    test(`${method}: clean phase-modulated I/Q covers low, normal, and high canine rates`, async t => {
        for (const [caseIndex, rates] of CLEAN_CASES.entries()) {
            await t.test(`HR ${rates.heartRateBpm} bpm, RR ${rates.respiratoryRateBpm} bpm`, () => {
                const fixture = generatePhaseModulatedIq({
                    ...rates,
                    sampleRateHz: SAMPLE_RATE_HZ,
                    durationSeconds: DURATION_SECONDS,
                    seed: 0x600d0000 + caseIndex
                });
                const extractor = vitalsApi.create(method, SAMPLE_RATE_HZ, EXTRACTOR_OPTIONS);
                const result = extractor.run(
                    fixture.iData,
                    fixture.qData,
                    SAMPLE_RATE_HZ,
                    {}
                );

                assertWithin(
                    result.HR_bpm,
                    rates.heartRateBpm,
                    MAX_HEART_ERROR_BPM,
                    `${method} heart rate`,
                    result
                );
                assertWithin(
                    result.RR_bpm,
                    rates.respiratoryRateBpm,
                    MAX_RESPIRATORY_ERROR_BPM,
                    `${method} respiratory rate`,
                    result
                );
            });
        }
    });
}

for (const method of ['freqBP', 'cwt']) {
    test(`${method}: constant I/Q does not fabricate a normal baseline`, () => {
        const sampleCount = SAMPLE_RATE_HZ * DURATION_SECONDS;
        const extractor = vitalsApi.create(method, SAMPLE_RATE_HZ, EXTRACTOR_OPTIONS);
        const result = extractor.run(
            new Array(sampleCount).fill(1.62),
            new Array(sampleCount).fill(1.59),
            SAMPLE_RATE_HZ,
            {}
        );

        assertNoFabricatedBaseline(result, `${method} constant I/Q`);
    });

    test(`${method}: fewer than minSamples does not fabricate a normal baseline`, () => {
        const sampleCount = EXTRACTOR_OPTIONS.minSamples - 1;
        const fixture = generatePhaseModulatedIq({
            heartRateBpm: 75,
            respiratoryRateBpm: 18,
            sampleRateHz: SAMPLE_RATE_HZ,
            durationSeconds: sampleCount / SAMPLE_RATE_HZ,
            seed: 0x127
        });
        const extractor = vitalsApi.create(method, SAMPLE_RATE_HZ, EXTRACTOR_OPTIONS);
        const result = extractor.run(
            fixture.iData,
            fixture.qData,
            SAMPLE_RATE_HZ,
            {}
        );

        assertNoFabricatedBaseline(result, `${method} insufficient samples`);
    });
}
