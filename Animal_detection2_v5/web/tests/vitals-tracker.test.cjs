'use strict';

const test = require('node:test');
const assert = require('node:assert/strict');
const VitalsTracker = require('../vitals-tracker.js');

function sample(hr, rr = 18, hrConfidence = 0.9, rrConfidence = 0.9) {
    return {
        HR_bpm: hr,
        RR_bpm: rr,
        HR_confidence: hrConfidence,
        RR_confidence: rrConfidence
    };
}

function warmup(tracker, hr = 85, rr = 18) {
    const first = tracker.update(sample(hr, rr));
    assert.equal(Number.isNaN(first.HR_bpm), true);
    assert.equal(Number.isNaN(first.RR_bpm), true);

    const second = tracker.update(sample(hr, rr));
    assert.equal(second.HR_bpm, hr);
    assert.equal(second.RR_bpm, rr);
    assert.deepEqual(second.valid, { HR: true, RR: true });
    return second;
}

test('does not invent a default and warms HR/RR independently', () => {
    const tracker = new VitalsTracker();
    const initial = tracker.reset();
    assert.equal(Number.isNaN(initial.HR_bpm), true);
    assert.equal(Number.isNaN(initial.RR_bpm), true);

    tracker.update(sample(85, NaN, 0.9, 0));
    const hrReady = tracker.update(sample(85, NaN, 0.9, 0));
    assert.equal(hrReady.HR_bpm, 85);
    assert.equal(Number.isNaN(hrReady.RR_bpm), true);
    assert.deepEqual(hrReady.valid, { HR: true, RR: false });

    tracker.update(sample(NaN, 18, 0, 0.9));
    const rrReady = tracker.update(sample(NaN, 18, 0, 0.9));
    assert.equal(rrReady.HR_bpm, 85);
    assert.equal(rrReady.RR_bpm, 18);
    assert.deepEqual(rrReady.valid, { HR: true, RR: true });
});

test('accepts a persistent 85 -> 45 bpm transition after three windows', () => {
    const tracker = new VitalsTracker();
    warmup(tracker, 85);

    assert.equal(tracker.update(sample(45)).HR_bpm, 85);
    assert.equal(tracker.update(sample(45)).HR_bpm, 85);
    const accepted = tracker.update(sample(45));

    assert.equal(accepted.HR_bpm, 45);
    assert.equal(accepted.valid.HR, true);
    assert.match(accepted.messages.join(' '), /接受新基线/);
});

test('accepts a persistent 85 -> 200 bpm transition after three windows', () => {
    const tracker = new VitalsTracker();
    warmup(tracker, 85);

    assert.equal(tracker.update(sample(200)).HR_bpm, 85);
    assert.equal(tracker.update(sample(201)).HR_bpm, 85);
    const accepted = tracker.update(sample(199));

    assert.equal(accepted.HR_bpm, 200);
    assert.equal(accepted.valid.HR, true);
});

test('tracks a large RR transition independently after three windows', () => {
    const tracker = new VitalsTracker();
    warmup(tracker, 85, 18);

    assert.equal(tracker.update(sample(85, 32)).RR_bpm, 18);
    assert.equal(tracker.update(sample(85, 31)).RR_bpm, 18);
    const accepted = tracker.update(sample(85, 33));

    assert.equal(accepted.HR_bpm, 85);
    assert.equal(accepted.RR_bpm, 32);
    assert.deepEqual(accepted.valid, { HR: true, RR: true });
});

test('rejects a single-window HR spike and keeps median smoothing', () => {
    const tracker = new VitalsTracker();
    warmup(tracker, 85);

    const spike = tracker.update(sample(200));
    assert.equal(spike.HR_bpm, 85);

    const recovered = tracker.update(sample(86));
    assert.equal(recovered.HR_bpm, 85);
    assert.equal(recovered.valid.HR, true);
    assert.match(recovered.messages.join(' '), /候选已取消/);

    tracker.update(sample(88));
    const smoothed = tracker.update(sample(87));
    assert.equal(smoothed.HR_bpm, 87);
});

test('expires stable values only after the configured hold duration', () => {
    const tracker = new VitalsTracker({ warmupWindows: 1, holdMaxMs: 5000 });
    tracker.update(sample(85, 18), null, { timestampMs: 0 });
    const invalid = sample(NaN, NaN, 0, 0);

    for (let timestampMs = 1000; timestampMs <= 5000; timestampMs += 1000) {
        const held = tracker.update(invalid, null, { timestampMs });
        assert.equal(held.HR_bpm, 85);
        assert.equal(held.RR_bpm, 18);
        assert.deepEqual(held.valid, { HR: true, RR: true });
        assert.deepEqual(held.held, { HR: true, RR: true });
    }

    const expired = tracker.update(invalid, null, { timestampMs: 5001 });
    assert.equal(Number.isNaN(expired.HR_bpm), true);
    assert.equal(Number.isNaN(expired.RR_bpm), true);
    assert.deepEqual(expired.valid, { HR: false, RR: false });
    assert.deepEqual(expired.confidence, { HR: 0, RR: 0 });
    assert.deepEqual(expired.status, { HR: 'expired', RR: 'expired' });
});
