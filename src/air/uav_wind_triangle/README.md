# UAV wind triangle

## Prerequisites

- Python 3.10 or newer.
- No third-party runtime dependency.

## Scope

Estimate required heading, ground speed, headwind and crosswind components from true airspeed, meteorological wind and a desired ground course. This is a planar, no-vertical-wind approximation.

## Run

```bash
PYTHONPATH=src/air/uav_wind_triangle python3 -m uav_wind_triangle.main
```

## Test

```bash
PYTHONPATH=src/air/uav_wind_triangle python3 -m unittest discover -s src/air/uav_wind_triangle/tests -v
```

## AI assistance disclosure

This module, tests and documentation were prepared with large-language-model assistance. The submitter reviewed and ran the content and is responsible for the submission.
