# Vital-sign algorithm tests

Current v5 result: **109 tests passed**.

Run from the repository's `source` directory:

```bash
node --test web/tests/*.test.cjs
```

The tests load `web/cwt-vitals.js` in a small VM sandbox and exercise only the
browser-compatible API:

```text
window.VitalsExtractor.create(method, fs, options).run(i, q, fs, hints)
```

The synthetic fixture uses a fixed random seed and phase-modulates a shared
quadrature I/Q carrier with breathing and cardiac motion. The clean-signal gate
is +/-5 bpm for heart rate and +/-3 breaths/min for respiratory rate. It covers
HR 35/45/75/90/120/180/220/260 and RR 6 through 60, including a 120 HR / 45 RR
case where the respiration fundamental overlaps the heart-search band.

Constant input and fewer than `minSamples` must produce an invalid estimate
(`NaN`, `null`, or `undefined`), never a fabricated 70, 72, or 80 bpm baseline.

The adversarial suite adds a 35-45 bpm phase component whose amplitude is more
than twice the labelled cardiac component. For true HR 84/86/90 bpm, especially
the exact 42/43/45 half-rate cases, the estimator must reject the low-frequency
peak and remain within +/-5 bpm of the labelled heartbeat.

Sampling rate is an input contract: raw I/Q arrays contain no independent
clock. A 100 Hz fixture passed as 50 Hz therefore exposes the mismatch as a
doubled duration and approximately half-rate HR/RR. The offline file-pipeline
test verifies the required upper-layer correction: timestamp-derived 100 Hz
must replace an initial 50 Hz configuration before vital-sign extraction.

The half-rate regression is fixed in both extractors without weakening the
+/-5 bpm assertion. Paired octave cases cover true 75 with a stronger 37.5
subharmonic and true 40 with a weak 80 harmonic. RR range settings cannot pick
the HR octave, a held/stale 40-bpm hint cannot lock a later 75-bpm signal, and a
single-window 40/80 pair without independent evidence must be marked ambiguous
at low confidence rather than establishing a fabricated baseline.

The v5 low-bias suite also reproduces the remaining failure with the correct
50 Hz clock: a weak 80 bpm heartbeat under a stronger 40 bpm component may be
returned only as a non-publishable ambiguity; once the 80 bpm evidence becomes
strong, even a fresh wrong 40 bpm hint cannot keep it locked. A true 80 bpm HR
with 40/min respiration cannot create publishable 37/43 bpm notch shoulders.
Conflicting 40/80 short and long windows can neither establish nor replace an
80 bpm baseline.

The app sampling-rate suite reproduces the real transport failure mode: 25 BLE
notifications/s whose completed-frame counts vary as `1/1/1/5` still average
50 frames/s. The estimator uses cumulative frame-index/time slope rather than
the median of per-notification instantaneous rates. Tests also cover 0--8 ms
decode delays, stable notification metadata, invalid device calendar dates,
both directions of a 2:1 clock conflict, a 50-Hz settings value that must not
overwrite verified live 100 Hz, sampling epochs, and stale hint aging.

The tracker and continuity suites cover independent warm-up, persistent
85 -> 45 and 85 -> 200 bpm transitions, a single-window spike, an RR
transition, motion-gated candidate rejection, a 30-second time-based hold,
fresh/held/expired metadata, independent HR/RR freshness and rapid
reacquisition after movement.

Window-independence tests require the app to collect the configured full 20 s
before its first extraction. Optional elapsed-time gates prevent 95%-overlapping
one-second updates from satisfying warm-up or large-jump confirmation before
their configured independent-evidence span has accumulated (2 s warm-up and
4 s jump confirmation in the web app).

The app motion suite uses deterministic ACC/Gyro windows to cover stationary
noise, a sub-second local movement, collar-orientation steps, recent strong
movement, two clean recovery seconds, adaptive thresholds and automatic
`m/s²`-to-`g` normalization. Local movement-mask tests inject strong I/Q phase
artifacts and verify that short dirty intervals are repaired while mostly dirty
windows become invalid instead of publishing a motion frequency.

The parser suite covers 3-column, 4-column and malformed input as well as
10/50/100 Hz timestamp-derived sampling-rate estimates.
