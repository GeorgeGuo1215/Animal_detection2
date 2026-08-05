'use strict';

/**
 * A tiny deterministic PRNG. Keeping it local makes failures reproducible on
 * every Node version and avoids adding a test dependency.
 */
function mulberry32(seed) {
    let state = seed >>> 0;
    return function random() {
        state = (state + 0x6D2B79F5) >>> 0;
        let value = state;
        value = Math.imul(value ^ (value >>> 15), value | 1);
        value ^= value + Math.imul(value ^ (value >>> 7), value | 61);
        return ((value ^ (value >>> 14)) >>> 0) / 4294967296;
    };
}

function gaussian(random) {
    // Box-Muller transform. random() can return zero, so keep both inputs in
    // the open interval (0, 1) before taking the logarithm.
    const u1 = Math.max(Number.EPSILON, random());
    const u2 = Math.max(Number.EPSILON, random());
    return Math.sqrt(-2 * Math.log(u1)) * Math.cos(2 * Math.PI * u2);
}

/**
 * Generate phase-modulated quadrature radar samples.
 *
 * The breathing and cardiac components modulate one shared phase rather than
 * being added independently to I and Q. DC offset, channel gain mismatch,
 * quadrature skew, slow drift, and small independent receiver noise keep the
 * fixture representative. Optional phaseArtifacts make it possible to build
 * deterministic adversarial cases without changing the labelled HR/RR.
 */
function generatePhaseModulatedIq({
    heartRateBpm,
    respiratoryRateBpm,
    sampleRateHz = 50,
    durationSeconds = 60,
    seed = 0x5eed1234,
    respiratoryPhaseRad = 0.32,
    cardiacPhaseRad = 0.11,
    noiseStdVolts = 0.0005,
    centerI = 1.62,
    centerQ = 1.59,
    radiusI = 0.12,
    radiusQ = 0.105,
    quadratureSkewRad = 3 * Math.PI / 180,
    phaseArtifacts = []
}) {
    if (!(heartRateBpm > 0) || !(respiratoryRateBpm > 0)) {
        throw new RangeError('heartRateBpm and respiratoryRateBpm must be positive');
    }
    if (!(sampleRateHz > 0) || !(durationSeconds > 0)) {
        throw new RangeError('sampleRateHz and durationSeconds must be positive');
    }

    const random = mulberry32(seed);
    const sampleCount = Math.round(sampleRateHz * durationSeconds);
    const iData = new Array(sampleCount);
    const qData = new Array(sampleCount);
    const heartHz = heartRateBpm / 60;
    const respiratoryHz = respiratoryRateBpm / 60;
    const artifacts = Array.from(phaseArtifacts || [], (artifact, index) => {
        const rateBpm = Number(artifact.rateBpm);
        const amplitudeRad = Number(artifact.amplitudeRad);
        const offsetRad = Number.isFinite(Number(artifact.offsetRad))
            ? Number(artifact.offsetRad)
            : 0.35 + index * 0.61;
        if (!(rateBpm > 0) || !(amplitudeRad >= 0)) {
            throw new RangeError('phaseArtifacts require positive rateBpm and non-negative amplitudeRad');
        }
        return { frequencyHz: rateBpm / 60, amplitudeRad, offsetRad };
    });

    for (let index = 0; index < sampleCount; index++) {
        const timeSeconds = index / sampleRateHz;
        const artifactPhase = artifacts.reduce((sum, artifact) => sum
            + artifact.amplitudeRad * Math.sin(
                2 * Math.PI * artifact.frequencyHz * timeSeconds + artifact.offsetRad
            ), 0);
        const phase = 0.70
            + respiratoryPhaseRad * Math.sin(2 * Math.PI * respiratoryHz * timeSeconds + 0.20)
            + cardiacPhaseRad * Math.sin(2 * Math.PI * heartHz * timeSeconds + 1.10)
            + artifactPhase
            + 0.02 * Math.sin(2 * Math.PI * 0.035 * timeSeconds);

        iData[index] = centerI
            + radiusI * Math.cos(phase)
            + noiseStdVolts * gaussian(random);
        qData[index] = centerQ
            + radiusQ * Math.sin(phase + quadratureSkewRad)
            + noiseStdVolts * gaussian(random);
    }

    return { iData, qData, sampleRateHz, durationSeconds };
}

module.exports = {
    generatePhaseModulatedIq,
    mulberry32
};
