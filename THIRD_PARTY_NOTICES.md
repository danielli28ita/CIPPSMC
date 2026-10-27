# Third-party sources

- PX4-Autopilot v1.15.4, BSD 3-Clause. Source copyright headers and the root LICENSE are kept. The upstream tree and submodules are fetched at pinned commits, and each dependency keeps its own license.
- The frozen controller, onboard task, and integration files come from this project's working tree. Provenance, the baseline, and per-file SHA256 values are in `firmware/BASELINE.json`. Parameters and sample results were not imported from any other flight log.
- `LemniscateTrajectory` and `TrajectorySample` in the offline trajectory test come from the project's existing reference generator. A copy of that package license is in `LICENSES/BPPSMC-reference.txt`.
- Python libraries and Gazebo are installed through the system or Python package manager. Their binaries are not presented as source of this project. Version locks are in `environments/`, and their licenses are those of the upstream distributions.
- The ULogs were supplied for the flight samples declared by this package. The original bytes and checksums are kept.

This package does not include historical paper summary tables, flight samples of the other controllers, or other ULogs. Newly added packaging and analysis scripts use the BSD 3-Clause terms in the root LICENSE. Existing copyright notices stay on their files.
