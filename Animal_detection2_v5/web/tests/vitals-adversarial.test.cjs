'use strict';

const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const test = require('node:test');
const vm = require('node:vm');

const RadarDataProcessor = require('../radar-processor.js');
const { generatePhaseModulatedIq } = require('./synthetic-iq.cjs');

const EXTRACTOR_OPTIONS = Object.freeze({
    hrLow: 35 / 60,
    hrHigh: 260 / 60,
    rrLow: 6 / 60,
    rrHigh: 60 / 60,
    hrMinFromRrRatio: 2.2,
    minSamples: 128
});

const HEART_TOLERANCE_BPM = 5;
const RESPIRATORY_TOLERANCE_BPM = 3;
const CARDIAC_PHASE_RAD = 0.07;
const ARTIFACT_PHASE_RAD = 0.18;

// The exact half-rate cases reproduce the observed failure mode: a strong
// low-frequency component near 40 bpm is selected instead of an 80-90 bpm
// heartbeat. The 35 bpm case is a non-harmonic low-frequency control.
const LOW_FREQUENCY_ARTIFACT_CASES = Object.freeze([
    { heartRateBpm: 84, artifactRateBpm: 35, label: 'non-harmonic low-frequency control' },
    { heartRateBpm: 84, artifactRateBpm: 42, label: 'exact half-rate artifact' },
    { heartRateBpm: 86, artifactRateBpm: 43, label: 'exact half-rate artifact' },
    { heartRateBpm: 90, artifactRateBpm: 45, label: 'exact half-rate artifact' }
]);

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

function peakEvidence(result) {
    const peaks = result?.diagnostics?.hrPeaksRobustTop || [];
    return peaks.slice(0, 4).map(peak => Number(peak.bpm).toFixed(1)).join('/');
}

function buildAdversarialFixture(testCase, seed) {
    assert.ok(
        ARTIFACT_PHASE_RAD > CARDIAC_PHASE_RAD,
        'fixture invariant: the low-frequency artifact must be stronger than the cardiac phase'
    );
    return generatePhaseModulatedIq({
        heartRateBpm: testCase.heartRateBpm,
        respiratoryRateBpm: 18,
        sampleRateHz: 50,
        durationSeconds: 60,
        seed,
        respiratoryPhaseRad: 0.18,
        cardiacPhaseRad: CARDIAC_PHASE_RAD,
        phaseArtifacts: [{
            rateBpm: testCase.artifactRateBpm,
            amplitudeRad: ARTIFACT_PHASE_RAD,
            offsetRad: 2.20
        }]
    });
}

const vitalsApi = loadVitalsExtractor();

for (const method of ['freqBP', 'cwt']) {
    test(`${method}: stronger 35-45 bpm phase artifact must not replace an 84-90 bpm heartbeat`, async t => {
        for (const [index, testCase] of LOW_FREQUENCY_ARTIFACT_CASES.entries()) {
            await t.test(
                `true HR ${testCase.heartRateBpm}, artifact ${testCase.artifactRateBpm} (${testCase.label})`,
                () => {
                    const fixture = buildAdversarialFixture(testCase, 0xa7700000 + index);
                    const result = vitalsApi.create(method, 50, EXTRACTOR_OPTIONS).run(
                        fixture.iData,
                        fixture.qData,
                        50,
                        {}
                    );

                    assert.ok(
                        !Number.isFinite(result.HR_bpm) || result.HR_bpm < 29.5 || result.HR_bpm > 45.5,
                        `${method} selected the forbidden low/half-rate band: HR=${result.HR_bpm}, `
                            + `true=${testCase.heartRateBpm}, artifact=${testCase.artifactRateBpm}, `
                            + `top HR peaks=${peakEvidence(result)}`
                    );
                    assertWithin(
                        result.HR_bpm,
                        testCase.heartRateBpm,
                        HEART_TOLERANCE_BPM,
                        `${method} adversarial heart rate`
                    );
                }
            );
        }
    });
}

for (const method of ['freqBP', 'cwt']) {
    test(`${method}: a weak 90 bpm harmonic must not blindly double a true 45 bpm bradycardia`, () => {
        const fixture = generatePhaseModulatedIq({
            heartRateBpm: 45,
            respiratoryRateBpm: 18,
            sampleRateHz: 50,
            durationSeconds: 60,
            seed: 0x450090,
            cardiacPhaseRad: 0.11,
            phaseArtifacts: [{ rateBpm: 90, amplitudeRad: 0.04, offsetRad: 1.70 }]
        });
        const result = vitalsApi.create(method, 50, EXTRACTOR_OPTIONS).run(
            fixture.iData,
            fixture.qData,
            50,
            {}
        );

        assertWithin(result.HR_bpm, 45, HEART_TOLERANCE_BPM, `${method} true bradycardia`);
        assert.equal(result.diagnostics?.octaveCorrection?.applied, false);
    });
}

for (const method of ['freqBP', 'cwt']) {
    test(`${method}: 100 Hz samples must use 100 Hz metadata; a 50 Hz label exposes half-rate scaling`, () => {
        const fixture = generatePhaseModulatedIq({
            heartRateBpm: 90,
            respiratoryRateBpm: 18,
            sampleRateHz: 100,
            durationSeconds: 30,
            seed: 0x10000050
        });
        const extractor = vitalsApi.create(method, 100, EXTRACTOR_OPTIONS);
        const corrected = extractor.run(fixture.iData, fixture.qData, 100, {});
        assertWithin(corrected.HR_bpm, 90, HEART_TOLERANCE_BPM, `${method} corrected HR`);
        assertWithin(corrected.RR_bpm, 18, RESPIRATORY_TOLERANCE_BPM, `${method} corrected RR`);

        const mislabeled = extractor.run(fixture.iData, fixture.qData, 50, {});
        assert.equal(mislabeled.fs, 50, 'the array-only extractor reports the fs supplied by its caller');
        assert.equal(mislabeled.durationSeconds, 60, 'wrong fs must remain visible as a doubled duration');
        if (Number.isFinite(mislabeled.HR_bpm) && Number.isFinite(mislabeled.RR_bpm)) {
            assertWithin(mislabeled.HR_bpm, 45, HEART_TOLERANCE_BPM, `${method} exposed half-rate HR`);
            assertWithin(mislabeled.RR_bpm, 9, RESPIRATORY_TOLERANCE_BPM, `${method} exposed half-rate RR`);
            assert.ok(Math.abs(mislabeled.HR_bpm - 90) > HEART_TOLERANCE_BPM);
        } else {
            assert.equal(mislabeled.valid, false, 'an extractor that detects the mismatch must mark it invalid');
        }
    });
}

test('offline file pipeline replaces configured 50 Hz with timestamp-derived 100 Hz', () => {
    const fixture = generatePhaseModulatedIq({
        heartRateBpm: 90,
        respiratoryRateBpm: 18,
        sampleRateHz: 100,
        durationSeconds: 30,
        seed: 0x50000100
    });
    const epochMs = Date.UTC(2025, 0, 1, 0, 0, 0);
    const rows = fixture.iData.map((iValue, index) => {
        const timestamp = new Date(epochMs + Math.floor(index / 100) * 1000)
            .toISOString()
            .slice(0, 19)
            .replace('T', ' ');
        return `${timestamp} ${iValue} ${fixture.qData[index]}`;
    });

    const processor = new RadarDataProcessor(50);
    const result = processor.processSingleFile('synthetic-100hz.txt', rows.join('\n'));

    assert.equal(result.status, 'success');
    assert.equal(result.samplingRate, 100, 'timestamp-derived rate must replace the configured 50 Hz');
    assert.equal(processor.fs, 100);
    assertWithin(result.heartRate, 90, HEART_TOLERANCE_BPM, 'file-pipeline corrected HR');
    assertWithin(result.respiratoryRate, 18, RESPIRATORY_TOLERANCE_BPM, 'file-pipeline corrected RR');
});

for (const method of ['freqBP', 'cwt']) {
    test(`${method}: distinguishes true 75 + strong 37.5 subharmonic from true 40 + weak 80 harmonic`, async t => {
        await t.test('true 75 bpm wins over a stronger exact 37.5 bpm subharmonic', () => {
            const cardiacPhaseRad = 0.07;
            const artifactPhaseRad = 0.18;
            assert.ok(artifactPhaseRad > 2 * cardiacPhaseRad, 'the labelled subharmonic must dominate');
            const fixture = generatePhaseModulatedIq({
                heartRateBpm: 75,
                respiratoryRateBpm: 18,
                sampleRateHz: 50,
                durationSeconds: 60,
                seed: 75375,
                respiratoryPhaseRad: 0.18,
                cardiacPhaseRad,
                phaseArtifacts: [{
                    rateBpm: 37.5,
                    amplitudeRad: artifactPhaseRad,
                    offsetRad: 2.20
                }]
            });
            const result = vitalsApi.create(method, 50, EXTRACTOR_OPTIONS).run(
                fixture.iData,
                fixture.qData,
                50,
                {}
            );

            const peaks = result.diagnostics?.hrPeaksRobustTop || [];
            assert.ok(
                peaks.some(peak => Math.abs(peak.bpm - 37.5) <= 2),
                `${method} fixture must retain evidence of the dominant 37.5 bpm component`
            );
            assertWithin(result.HR_bpm, 75, HEART_TOLERANCE_BPM, `${method} true 75 bpm`);
            assertWithin(result.RR_bpm, 18, RESPIRATORY_TOLERANCE_BPM, `${method} true 18 bpm RR`);
            assert.ok(Math.abs(result.HR_bpm - 37.5) > 20, `${method} selected the half-rate artifact`);
        });

        await t.test('true 40 bpm is not blindly doubled by a weak 80 bpm harmonic', () => {
            const cardiacPhaseRad = 0.11;
            const harmonicPhaseRad = 0.04;
            assert.ok(harmonicPhaseRad < cardiacPhaseRad / 2, 'the 80 bpm component must remain weak');
            const fixture = generatePhaseModulatedIq({
                heartRateBpm: 40,
                respiratoryRateBpm: 18,
                sampleRateHz: 50,
                durationSeconds: 60,
                seed: 40080,
                respiratoryPhaseRad: 0.18,
                cardiacPhaseRad,
                phaseArtifacts: [{
                    rateBpm: 80,
                    amplitudeRad: harmonicPhaseRad,
                    offsetRad: 2.20
                }]
            });
            const result = vitalsApi.create(method, 50, EXTRACTOR_OPTIONS).run(
                fixture.iData,
                fixture.qData,
                50,
                {}
            );

            const peaks = result.diagnostics?.hrPeaksRobustTop || [];
            assert.ok(
                peaks.some(peak => Math.abs(peak.bpm - 40) <= 3),
                `${method} fixture must contain the labelled 40 bpm fundamental`
            );
            assert.ok(
                peaks.some(peak => Math.abs(peak.bpm - 80) <= 4),
                `${method} fixture must contain the weak 80 bpm ambiguity`
            );
            assertWithin(result.HR_bpm, 40, HEART_TOLERANCE_BPM, `${method} true 40 bpm`);
            assertWithin(result.RR_bpm, 18, RESPIRATORY_TOLERANCE_BPM, `${method} true 18 bpm RR`);
            assert.ok(Math.abs(result.HR_bpm - 80) > 20, `${method} blindly doubled true bradycardia`);
            assert.equal(result.diagnostics?.octaveCorrection?.applied, false);
            assert.equal(result.diagnostics?.octaveCorrection?.ambiguous, true);
            assert.ok(result.HR_confidence < 0.28, 'an irreducible 40/80 pair must not establish a fresh baseline');
        });
    });
}

for (const method of ['freqBP', 'cwt']) {
    test(`${method}: octave recovery is independent of RR settings, fixed-Hz edge, and stale low hints`, async t => {
        const buildFixture = (artifactRateBpm, seed) => generatePhaseModulatedIq({
            heartRateBpm: 75,
            respiratoryRateBpm: 18,
            sampleRateHz: 50,
            durationSeconds: 60,
            seed,
            respiratoryPhaseRad: 0.18,
            cardiacPhaseRad: 0.07,
            phaseArtifacts: [{ rateBpm: artifactRateBpm, amplitudeRad: 0.18, offsetRad: 2.20 }]
        });

        await t.test('changing RR maximum cannot change the selected HR octave', () => {
            const fixture = buildFixture(37.5, 0x75375001);
            for (const rrHigh of [30 / 60, 60 / 60, 90 / 60]) {
                const result = vitalsApi.create(method, 50, {
                    ...EXTRACTOR_OPTIONS,
                    rrHigh
                }).run(fixture.iData, fixture.qData, 50, {});
                assertWithin(result.HR_bpm, 75, HEART_TOLERANCE_BPM, `${method} RR-high invariant HR`);
            }
        });

        await t.test('40-to-75 near-octave pair is not lost at the old 0.08 Hz boundary', () => {
            const fixture = buildFixture(40, 0x75400001);
            const result = vitalsApi.create(method, 50, EXTRACTOR_OPTIONS).run(
                fixture.iData,
                fixture.qData,
                50,
                {}
            );
            assertWithin(result.HR_bpm, 75, HEART_TOLERANCE_BPM, `${method} 40-to-75 recovery`);
            assert.equal(result.diagnostics?.octaveCorrection?.applied, true);
            assert.ok(result.diagnostics?.octaveCorrection?.octaveError <= 0.10);
        });

        await t.test('an untrusted or held 40 bpm hint cannot permanently lock a true 75 bpm signal', () => {
            const fixture = buildFixture(37.5, 0x75375002);
            for (const hints of [
                { hrHintFreq: 40 / 60 },
                {
                    hrHintFreq: 40 / 60,
                    hrHintMeta: {
                        status: 'held',
                        ageMs: 3000,
                        confidence: 0.95,
                        freshWindowCount: 10,
                        ambiguous: false
                    }
                }
            ]) {
                const result = vitalsApi.create(method, 50, EXTRACTOR_OPTIONS).run(
                    fixture.iData,
                    fixture.qData,
                    50,
                    hints
                );
                assertWithin(result.HR_bpm, 75, HEART_TOLERANCE_BPM, `${method} stale-hint recovery`);
            }
        });
    });
}
