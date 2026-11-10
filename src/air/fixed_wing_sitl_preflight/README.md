# Fixed-wing SITL preflight

## Prerequisites

- Python 3.10 or newer.
- No runtime third-party packages.
- ROS 2 Humble and launch packages only for the installed launch entry.

## Purpose

This module performs a deterministic static check of PX4/ArduPilot fixed-wing
SITL configuration. It verifies firmware mode, airframe type, network ports,
geo origin, required control surfaces, sensors and failsafe settings.

It does not connect to a flight controller, validate aircraft dynamics, or
replace a real SITL arming and takeoff test.

## Run

```bash
PYTHONPATH=src/air/fixed_wing_sitl_preflight \
  python3 -m fixed_wing_sitl_preflight.main
```

## Test

```bash
PYTHONPATH=src/air/fixed_wing_sitl_preflight python3 -m unittest discover \
  -s src/air/fixed_wing_sitl_preflight/tests -v
```

## AI assistance disclosure

This module, tests and documentation were prepared with large-language-model
assistance. The submitter reviewed and ran the content and is responsible for
the final submission.
