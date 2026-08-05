'use strict';

const test = require('node:test');
const assert = require('node:assert/strict');
const VitalsTracker = require('../vitals-tracker.js');

function sample(hr, rr, hrConfidence = 0.92, rrConfidence = 0.92) {
    return {
        HR_bpm: hr,
        RR_bpm: rr,
        HR_confidence: hrConfidence,
        RR_confidence: rrConfidence
    };
}

function context(timestampMs, motionState = 'stationary', motionQuality = 0.95) {
    return { timestampMs, motionState, motionQuality };
}

function assertMetricMetadata(snapshot, kind, {
    status,
    held,
    fresh,
    ageMs
}) {
    assert.equal(snapshot.status[kind], status, `${kind} status`);
    assert.equal(snapshot.held[kind], held, `${kind} held flag`);
    assert.equal(snapshot.fresh[kind], fresh, `${kind} fresh flag`);
    if (ageMs !== undefined) {
        assert.equal(snapshot.ageMs[kind], ageMs, `${kind} age`);
    }
}

function warmup(tracker, hr = 86, rr = 18) {
    const first = tracker.update(sample(hr, rr), null, context(0));
    assert.equal(Number.isNaN(first.HR_bpm), true);
    assert.equal(Number.isNaN(first.RR_bpm), true);
    assertMetricMetadata(first, 'HR', {
        status: 'warming', held: false, fresh: false
    });
    assertMetricMetadata(first, 'RR', {
        status: 'warming', held: false, fresh: false
    });

    const ready = tracker.update(sample(hr, rr), null, context(1000));
    assert.equal(ready.HR_bpm, hr);
    assert.equal(ready.RR_bpm, rr);
    assertMetricMetadata(ready, 'HR', {
        status: 'fresh', held: false, fresh: true, ageMs: 0
    });
    assertMetricMetadata(ready, 'RR', {
        status: 'fresh', held: false, fresh: true, ageMs: 0
    });
    return ready;
}

test('short movement keeps the last trustworthy display and never accepts motion peaks', () => {
    const tracker = new VitalsTracker();
    warmup(tracker);

    // These values are intentionally plausible but wrong. A movement gate must
    // reject them before the normal persistent-jump confirmation can accept them.
    for (const timestampMs of [2000, 3000, 4000, 5000, 6000]) {
        const held = tracker.update(
            sample(180, 48, 0.99, 0.99),
            sample(181, 47, 0.99, 0.99),
            context(timestampMs, 'moving', 0.03)
        );
        assert.equal(held.HR_bpm, 86);
        assert.equal(held.RR_bpm, 18);
        assertMetricMetadata(held, 'HR', {
            status: 'held', held: true, fresh: false, ageMs: timestampMs - 1000
        });
        assertMetricMetadata(held, 'RR', {
            status: 'held', held: true, fresh: false, ageMs: timestampMs - 1000
        });
    }
});

test('the first nearby high-confidence window after movement restores fresh output', () => {
    const tracker = new VitalsTracker();
    warmup(tracker);
    tracker.update(sample(190, 55), null, context(3000, 'moving', 0.02));

    const recovered = tracker.update(
        sample(87, 19, 0.94, 0.93),
        null,
        context(4000, 'stationary', 0.90)
    );

    assert.ok(Math.abs(recovered.HR_bpm - 86) <= 1, `unexpected HR ${recovered.HR_bpm}`);
    assert.ok(Math.abs(recovered.RR_bpm - 18) <= 1, `unexpected RR ${recovered.RR_bpm}`);
    assertMetricMetadata(recovered, 'HR', {
        status: 'fresh', held: false, fresh: true, ageMs: 0
    });
    assertMetricMetadata(recovered, 'RR', {
        status: 'fresh', held: false, fresh: true, ageMs: 0
    });
});

test('persistent movement holds for at least 15 s, expires by 30 s, and never fabricates a new baseline', () => {
    const tracker = new VitalsTracker();
    warmup(tracker);

    const stillHeld = tracker.update(
        sample(210, 58, 1, 1),
        null,
        context(16000, 'moving', 0)
    );
    assert.equal(stillHeld.HR_bpm, 86, 'HR must remain visible for 15 s after its last fresh window');
    assert.equal(stillHeld.RR_bpm, 18, 'RR must remain visible for 15 s after its last fresh window');
    assertMetricMetadata(stillHeld, 'HR', {
        status: 'held', held: true, fresh: false, ageMs: 15000
    });

    const nearDeadline = tracker.update(
        sample(210, 58, 1, 1),
        null,
        context(30999, 'moving', 0)
    );
    assert.equal(nearDeadline.HR_bpm, 86);
    assert.equal(nearDeadline.RR_bpm, 18);

    const expired = tracker.update(
        sample(210, 58, 1, 1),
        null,
        context(31001, 'moving', 0)
    );
    assert.equal(Number.isNaN(expired.HR_bpm), true);
    assert.equal(Number.isNaN(expired.RR_bpm), true);
    assertMetricMetadata(expired, 'HR', {
        status: 'expired', held: false, fresh: false, ageMs: 30001
    });
    assertMetricMetadata(expired, 'RR', {
        status: 'expired', held: false, fresh: false, ageMs: 30001
    });
});

test('a nearby trustworthy measurement restores an expired display in one window', () => {
    const tracker = new VitalsTracker();
    warmup(tracker);
    tracker.update(sample(205, 54), null, context(31001, 'moving', 0));

    const recovered = tracker.update(
        sample(88, 19, 0.95, 0.95),
        null,
        context(32000, 'stationary', 0.94)
    );

    assert.ok(Number.isFinite(recovered.HR_bpm));
    assert.ok(Number.isFinite(recovered.RR_bpm));
    assertMetricMetadata(recovered, 'HR', {
        status: 'fresh', held: false, fresh: true, ageMs: 0
    });
    assertMetricMetadata(recovered, 'RR', {
        status: 'fresh', held: false, fresh: true, ageMs: 0
    });
});

test('HR and RR freshness are tracked independently', () => {
    const tracker = new VitalsTracker();
    warmup(tracker);

    const rrMissing = tracker.update(
        sample(87, NaN, 0.94, 0),
        null,
        context(2000)
    );
    assert.ok(Number.isFinite(rrMissing.HR_bpm));
    assert.equal(rrMissing.RR_bpm, 18);
    assertMetricMetadata(rrMissing, 'HR', {
        status: 'fresh', held: false, fresh: true, ageMs: 0
    });
    assertMetricMetadata(rrMissing, 'RR', {
        status: 'held', held: true, fresh: false, ageMs: 1000
    });

    const hrMissing = tracker.update(
        sample(NaN, 19, 0, 0.95),
        null,
        context(3000)
    );
    assert.ok(Number.isFinite(hrMissing.HR_bpm));
    assert.ok(Number.isFinite(hrMissing.RR_bpm));
    assertMetricMetadata(hrMissing, 'HR', {
        status: 'held', held: true, fresh: false, ageMs: 1000
    });
    assertMetricMetadata(hrMissing, 'RR', {
        status: 'fresh', held: false, fresh: true, ageMs: 0
    });
});

test('timestamp zero is a real time origin, not a missing timestamp', () => {
    const tracker = new VitalsTracker({ warmupWindows: 1 });
    const fresh = tracker.update(sample(86, 18), null, context(0));
    assertMetricMetadata(fresh, 'HR', {
        status: 'fresh', held: false, fresh: true, ageMs: 0
    });

    const held = tracker.update(sample(200, 50), null, context(1000, 'moving', 0));
    assert.equal(held.HR_bpm, 86);
    assertMetricMetadata(held, 'HR', {
        status: 'held', held: true, fresh: false, ageMs: 1000
    });
});

test('movement before any trustworthy baseline never fabricates HR or RR', () => {
    const tracker = new VitalsTracker({ warmupWindows: 1 });

    for (const timestampMs of [0, 5000, 15000, 31000]) {
        const unavailable = tracker.update(
            sample(195, 52, 1, 1),
            sample(196, 51, 1, 1),
            context(timestampMs, 'moving', 0)
        );
        assert.equal(Number.isNaN(unavailable.HR_bpm), true);
        assert.equal(Number.isNaN(unavailable.RR_bpm), true);
        assert.deepEqual(unavailable.valid, { HR: false, RR: false });
        assert.deepEqual(unavailable.status, { HR: 'warming', RR: 'warming' });
        assert.deepEqual(unavailable.held, { HR: false, RR: false });
        assert.deepEqual(unavailable.fresh, { HR: false, RR: false });
        assert.deepEqual(unavailable.ageMs, { HR: null, RR: null });
    }
});

test('a nearby clean window reacquires immediately after held values have expired', () => {
    const tracker = new VitalsTracker({ holdMaxMs: 15000 });
    warmup(tracker);

    const expired = tracker.update(
        sample(200, 50, 1, 1),
        null,
        context(16001, 'moving', 0)
    );
    assert.deepEqual(expired.status, { HR: 'expired', RR: 'expired' });

    const recovered = tracker.update(
        sample(88, 19, 0.94, 0.93),
        null,
        context(17000, 'stationary', 0.95)
    );
    assert.equal(recovered.HR_bpm, 88);
    assert.equal(recovered.RR_bpm, 19);
    assert.deepEqual(recovered.status, { HR: 'fresh', RR: 'fresh' });
    assert.deepEqual(recovered.fresh, { HR: true, RR: true });
    assert.deepEqual(recovered.ageMs, { HR: 0, RR: 0 });
});
