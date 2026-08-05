'use strict';

const test = require('node:test');
const assert = require('node:assert/strict');

const RadarDataProcessor = require('../radar-processor.js');
const { generatePhaseModulatedIq } = require('./synthetic-iq.cjs');

function buildRows(counts, timestampForSecond) {
    const rows = [];
    let sampleIndex = 0;

    counts.forEach((count, secondIndex) => {
        const timestamp = timestampForSecond(secondIndex);
        for (let index = 0; index < count; index++) {
            const iValue = (0.25 + sampleIndex * 0.0001).toFixed(6);
            const qValue = (1.33 + sampleIndex * 0.0001).toFixed(6);
            rows.push(`${timestamp} ${iValue} ${qValue}`);
            sampleIndex++;
        }
    });

    return rows;
}

function assertApproxRate(actual, expected) {
    assert.equal(typeof actual, 'number', 'estimatedSampleRate 应为数值');
    assert.ok(Number.isFinite(actual), 'estimatedSampleRate 应为有限值');
    assert.ok(
        Math.abs(actual - expected) <= 0.5,
        `预期采样率约 ${expected} Hz，实际为 ${actual}`
    );
}

test('parseDataFile 解析三列 timestamp I Q，并按每秒计数中位数估计约 10 Hz', () => {
    const processor = new RadarDataProcessor();
    const rows = buildRows(
        [9, 10, 11],
        secondIndex => `2025-09-03-08-07-${String(35 + secondIndex).padStart(2, '0')}`
    );

    const parsed = processor.parseDataFile(`${rows.join('\n')}\n`);

    assert.equal(parsed.length, 30);
    assert.equal(parsed.timestamps.length, 30);
    assert.equal(parsed.timestamps[0], '2025-09-03-08-07-35');
    assert.ok(parsed.iData instanceof Float64Array);
    assert.ok(parsed.qData instanceof Float64Array);
    assertApproxRate(parsed.estimatedSampleRate, 10);
});

test('parseDataFile 解析四列 date time I Q，并按每秒计数中位数估计约 50 Hz', () => {
    const processor = new RadarDataProcessor();
    const rows = buildRows(
        [48, 50, 52],
        secondIndex => `2025-09-03 08:08:${String(secondIndex).padStart(2, '0')}`
    );

    const parsed = processor.parseDataFile(rows.join('\n'));

    assert.equal(parsed.length, 150);
    assert.equal(parsed.timestamps.length, 150);
    assert.equal(parsed.timestamps[0], '2025-09-03 08:08:00');
    assertApproxRate(parsed.estimatedSampleRate, 50);
});

test('parseDataFile 拒绝异常拼包和有限但极端的 I/Q，且有效数据估计约 100 Hz', () => {
    const processor = new RadarDataProcessor();
    const validRows = buildRows(
        [99, 100, 101],
        secondIndex => `2025-09-03 08:09:${String(secondIndex).padStart(2, '0')}`
    );
    const invalidRows = [
        // 两个四列数据包被错误拼在同一行，不能静默接受第一个包。
        '2025-09-03 08:09:00 0.250 1.330 2025-09-03 08:09:00 0.251 1.331',
        // parseFloat 可解析这些有限值，但它们显然不是合法雷达 I/Q。
        '2025-09-03 08:09:01 1000000000000 -1000000000000',
        '2025-09-03-08-09-02 1e300 -1e300'
    ];

    const parsed = processor.parseDataFile([...validRows, ...invalidRows].join('\n'));

    assert.equal(parsed.length, 300, '异常行不得进入有效样本');
    assert.equal(parsed.timestamps.length, 300);
    assert.ok(Array.from(parsed.iData).every(value => Math.abs(value) < 1_000));
    assert.ok(Array.from(parsed.qData).every(value => Math.abs(value) < 1_000));
    assertApproxRate(parsed.estimatedSampleRate, 100);
});

test('Node 离线处理器复用宽频提取器，不回退到旧固定 bin 心率', () => {
    const processor = new RadarDataProcessor(50);
    const fixture = generatePhaseModulatedIq({
        heartRateBpm: 180,
        respiratoryRateBpm: 18,
        sampleRateHz: 50,
        durationSeconds: 30,
        seed: 0x180018
    });

    const result = processor.extractVitalSignsMainPy(fixture.iData, fixture.qData);

    assert.ok(Number.isFinite(result.heartRate));
    assert.ok(Math.abs(result.heartRate - 180) <= 5, `HR=${result.heartRate}`);
    assert.ok(Number.isFinite(result.respiratoryRate));
    assert.ok(Math.abs(result.respiratoryRate - 18) <= 3, `RR=${result.respiratoryRate}`);
    assert.ok(result.heartRate > 120, '不得被旧 54–120 bpm 固定 bin 范围截断');
});
