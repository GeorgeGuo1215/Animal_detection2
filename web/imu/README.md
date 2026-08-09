# Web IMU vitals module

This directory is the browser-ready copy of the validated causal IMU HR/RR estimator. It is intentionally isolated from the existing radar pipeline.

- `src/`: production estimator and browser worker entry.
- `tests/`: causal real-time/runtime regression tests copied with the algorithm.
- `dist/imu-vitals.worker.js`: generated worker loaded by the web page (commit this file).
- `data/`, Gold alignment, validation plots and offline evaluation utilities are deliberately excluded from the web module.

The page sends only BLE samples classified as live. Backfill, recording, upload and radar buffers are not read or modified by this module. Acceleration enters as `g`, gyroscope as `deg/s`, and the worker converts units using the estimator's existing configuration.

```powershell
cd web/imu
npm.cmd install
npm.cmd test
npm.cmd run build
```

Edit the source files and rebuild `dist/imu-vitals.worker.js` before committing changes.
