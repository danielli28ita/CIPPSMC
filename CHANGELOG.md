# 1.0.0 — 2026-10-26

- Freeze the real-flight controller, the onboard Bernoulli-lemniscate task, the PX4 integration, and the Harmonic adapter: 28 modified or added files.
- Pin the PX4 v1.15.4 baseline and the 32 listed submodule commits. Add an independent fetch, build, and binary control-branch check.
- Include `log_1.ulg` as the metric sample. Export all 1153 initial parameters and the 36 controller and task values, keeping the exact float32 values.
- Add a separate Python environment and a lock file, without using the author's NumPy compatibility path.
- Add in-package recomputation, expected metrics, the original trajectory and stage tests, and the parameter-provenance check.
- Rewrite the build, parameter, hardware, and reproduction-boundary documents for this version.
- Do not change the control law and do not turn the flight-tuned values into every firmware default. No new flash and no new flight were performed for this package.

# 2026-10-27

- Publish the package under the paper title, with the short name BIPPSMC.
- Add Flight B (`data/log_2.ulg`) and redraw the README figures from Flight A and Flight B only.
