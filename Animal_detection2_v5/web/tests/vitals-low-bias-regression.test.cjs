'use strict';

const assert = require('node:assert/strict');
const test = require('node:test');

const VitalsExtractor = require('../cwt-vitals.js');
const VitalsTracker = require('../vitals-tracker.js');
const { generatePhaseModulatedIq } = require('./synthetic-iq.cjs');

const SAMPLE_RATE_HZ = 50;
const TRACKER_MIN_CONFIDENCE = 0.35;
const HEART_TOLERANCE_BPM = 5;
const EXTRACTOR_OPTIONS = Object.freeze({
    hrLow: 35 / 60,
    hrHigh: 260 / 60,
    rrLow: 6 / 60,
    rrHigh: 60 / 60,
    minSamples: 128
});

function runExtractor(method, fixtureOptions, hints = {}) {
    const fixture = generatePhaseModulatedIq({
        sampleRateHz: SAMPLE_RATE_HZ,
        durationSeconds: 20,
        noiseStdVolts: 0.0005,
        ...fixtureOptions
    });
    return VitalsExtractor.create(method, SAMPLE_RATE_HZ, EXTRACTOR_OPTIONS).run(
        fixture.iData,
        fixture.qData,
        SAMPLE_RATE_HZ,
        hints
    );
}

function within(actual, expected, tolerance) {
    return Number.isFinite(actual) && Math.abs(actual - expected) <= tolerance;
}

function assertCorrectOrNotPublishable(result, expectedHeartRate, label) {
    const correct = within(result.HR_bpm, expectedHeartRate, HEART_TOLERANCE_BPM);
    const notPublishable = !Number.isFinite(result.HR_bpm)
        || result.HR_confidence < TRACKER_MIN_CONFIDENCE;

    assert.ok(
        correct || notPublishable,
        `${label}: expected HR near ${expectedHeartRate} bpm or a non-publishable `
            + `ambiguous result; received HR=${result.HR_bpm}, confidence=${result.HR_confidence}, `
            + `octave=${result.diagnostics?.octaveCorrection?.reason}`
    );
}

function trackerSample(hr, hrConfidence = 0.90) {
    return {
        HR_bpm: hr,
        RR_bpm: 18,
        HR_confidence: hrConfidence,
        RR_confidence: 0.90
    };
}

for (const method of ['freqBP', 'cwt']) {
    test(`${method}: a weak 80 bpm heartbeat under a stronger 40 bpm component is not published as confident 40`, () => {
        const result = runExtractor(method, {
            heartRateBpm: 80,
            respiratoryRateBpm: 18,
            seed: 0x408001,
            respiratoryPhaseRad: 0.18,
            cardiacPhaseRad: 0.02,
            phaseArtifacts: [{
                rateBpm: 40,
                amplitudeRad: 0.12,
                offsetRad: 2.20
            }]
        });

        // This one window is physically hard to identify. Returning the low
        // hypothesis for diagnostics is acceptable, but it must not be given
        // enough confidence to create a fresh 40 bpm tracker baseline.
        assertCorrectOrNotPublishable(result, 80, `${method} weak-heart ambiguity`);
        if (within(result.HR_bpm, 40, 7)) {
            assert.ok(
                result.HR_confidence < TRACKER_MIN_CONFIDENCE,
                `${method} assigned publishable confidence to the 40 bpm component`
            );
        }
    });

    test(`${method}: stronger 80 bpm evidence overrides a fresh but wrong 40 bpm hint`, () => {
        const result = runExtractor(method, {
            heartRateBpm: 80,
            respiratoryRateBpm: 18,
            seed: 0x408001,
            respiratoryPhaseRad: 0.18,
            cardiacPhaseRad: 0.11,
            phaseArtifacts: [{
                rateBpm: 40,
                amplitudeRad: 0.12,
                offsetRad: 2.20
            }]
        }, {
            hrHintFreq: 40 / 60,
            hrHintMeta: {
                status: 'fresh',
                ageMs: 0,
                confidence: 0.90,
                freshWindowCount: 4,
                ambiguous: false
            }
        });

        assert.ok(
            within(result.HR_bpm, 80, HEART_TOLERANCE_BPM),
            `${method} remained self-locked to the stale octave: HR=${result.HR_bpm}, `
                + `confidence=${result.HR_confidence}, `
                + `reason=${result.diagnostics?.octaveCorrection?.reason}`
        );
        assert.ok(
            result.HR_confidence >= TRACKER_MIN_CONFIDENCE,
            `${method} found the strong 80 bpm peak but left it non-publishable`
        );
    });

    test(`${method}: true HR 80 with RR 40 does not publish a 37/43 bpm notch shoulder as HR`, () => {
        const result = runExtractor(method, {
            heartRateBpm: 80,
            respiratoryRateBpm: 40,
            seed: 0x804032,
            respiratoryPhaseRad: 0.32,
            cardiacPhaseRad: 0.11
        });

        // There is no injected 37/43 bpm artefact here: those peaks are the
        // shoulders created around the strong 40 bpm respiratory suppression
        // notch. Prefer 80; if the window is still ambiguous, do not publish it.
        assertCorrectOrNotPublishable(result, 80, `${method} RR/HR cross-band ambiguity`);
        if (result.HR_bpm >= 34 && result.HR_bpm <= 48) {
            assert.ok(
                result.HR_confidence < TRACKER_MIN_CONFIDENCE,
                `${method} published a respiratory-notch shoulder as HR: `
                    + `${result.HR_bpm} bpm at confidence ${result.HR_confidence}`
            );
        }
    });
}

test('tracker does not establish a 40 bpm baseline when high-confidence short/long windows disagree 40/80', () => {
    const tracker = new VitalsTracker({ warmupWindows: 2 });
    const shortWindow = trackerSample(40);
    const longWindow = trackerSample(80);

    const first = tracker.update(shortWindow, longWindow);
    const second = tracker.update(shortWindow, longWindow);
    for (const [label, snapshot] of [['first', first], ['second', second]]) {
        assert.ok(
            !Number.isFinite(snapshot.HR_bpm) || within(snapshot.HR_bpm, 80, 1),
            `${label} conflict update established the short-window 40 bpm baseline: ${snapshot.HR_bpm}`
        );
    }

    // An implementation may either prefer the guard or reject the conflict.
    // Once both windows agree, either policy must converge to the 80 bpm track.
    if (!Number.isFinite(second.HR_bpm)) {
        tracker.update(trackerSample(80), trackerSample(80));
    }
    const recovered = tracker.update(trackerSample(80), trackerSample(80));
    assert.ok(
        within(recovered.HR_bpm, 80, 1),
        `tracker did not recover the guard-supported 80 bpm baseline: ${recovered.HR_bpm}`
    );
});

test('tracker never switches an existing 80 bpm baseline to conflicting short-window 40 bpm', () => {
    const tracker = new VitalsTracker({
        warmupWindows: 2,
        jumpConfirmWindows: 3
    });
    tracker.update(trackerSample(80), trackerSample(80));
    const ready = tracker.update(trackerSample(80), trackerSample(80));
    assert.ok(within(ready.HR_bpm, 80, 1), 'fixture must first establish an 80 bpm baseline');

    for (let index = 0; index < 5; index++) {
        const held = tracker.update(trackerSample(40), trackerSample(80));
        assert.ok(
            within(held.HR_bpm, 80, 1),
            `conflicting short window replaced the 80 bpm baseline on update ${index + 1}: `
                + `${held.HR_bpm}`
        );
    }
});
