# Results and Artifacts

This directory holds experiment outputs. Large or machine-specific files are ignored by Git so the repository stays reviewable.

- `logs/` — per-run mission logs, telemetry CSVs, rosbag2 recordings, PX4/agent console logs.
- `plots/` — generated figures (position error, tracking, mission timelines).
- `reports/` — human-readable experiment reports and summaries.

Repository policy:

- Keep `.gitkeep` files so the layout exists in a fresh clone.
- Record the run ID, commit, PX4 version, parameters and scenario in each report.
- Do not commit large rosbag2 directories or raw sensor data; summarize the metrics and keep representative excerpts.
- The first Milestone 1 run must be recorded in `docs/experiments/development_log.md`.
