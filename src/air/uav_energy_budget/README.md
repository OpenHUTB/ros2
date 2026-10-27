# UAV energy budget

## Prerequisites

- Python 3.10 or newer.
- No runtime third-party packages.
- ROS 2 Humble only for installed launch.

## Scope

This module estimates mission energy from hover, climb and cruise segments.
It includes payload mass scaling, headwind-adjusted ground speed, battery
efficiency and a configurable reserve. The model is intentionally simplified
and is not a high-fidelity battery or aerodynamic simulator.

## Run

```bash
PYTHONPATH=src/air/uav_energy_budget python3 -m uav_energy_budget.main
```

## Test

```bash
PYTHONPATH=src/air/uav_energy_budget python3 -m unittest discover \
  -s src/air/uav_energy_budget/tests -v
```

## AI assistance disclosure

This module, tests and documentation were prepared with large-language-model
assistance. The submitter reviewed and ran the content and is responsible for
the final submission.
