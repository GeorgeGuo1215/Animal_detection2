'use strict';

const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const test = require('node:test');
const vm = require('node:vm');

function loadBluetoothManager() {
    const sourcePath = path.resolve(__dirname, '..', 'bluetooth.js');
    const source = fs.readFileSync(sourcePath, 'utf8');
    const noop = () => {};
    const window = {};
    const sandbox = {
        window,
        navigator: {},
        localStorage: { getItem: () => null, setItem: noop, removeItem: noop },
        console: { log: noop, warn: noop, error: noop },
        TextDecoder,
        TextEncoder,
        setInterval,
        clearInterval,
        setTimeout,
        clearTimeout
    };
    sandbox.globalThis = sandbox;
    vm.createContext(sandbox);
    vm.runInContext(source, sandbox, { filename: sourcePath });
    return window.BLE;
}

test('同一BLE通知解出的多帧携带相同notificationId和单调到达时间', () => {
    const manager = loadBluetoothManager();
    const frames = [];
    manager.onLine = (line, meta) => frames.push({ line, meta });

    manager._handleIncomingText('$FRAME_A*$FRAME_B*', {
        notificationId: 17,
        arrivalMs: 1_750_000_000_000,
        arrivalMonoMs: 1234.5
    });

    assert.equal(frames.length, 2);
    assert.deepEqual(frames.map(frame => frame.line), ['$FRAME_A', '$FRAME_B']);
    assert.deepEqual(frames.map(frame => frame.meta.notificationId), [17, 17]);
    assert.deepEqual(frames.map(frame => frame.meta.arrivalMonoMs), [1234.5, 1234.5]);
    assert.deepEqual(frames.map(frame => frame.meta.frameIndex), [0, 1]);
});
