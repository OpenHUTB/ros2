# UAV radio link budget

## Prerequisites

- Python 3.10 or newer.
- No third-party runtime dependency.

## Scope

Estimate free-space communication range from transmit power, antenna gains, receiver sensitivity, fade margin and additional losses. Terrain, multipath and obstructions are outside the model.

## Run

```bash
PYTHONPATH=src/air/uav_radio_link_budget python3 -m uav_radio_link_budget.main
```

## Test

```bash
PYTHONPATH=src/air/uav_radio_link_budget python3 -m unittest discover -s src/air/uav_radio_link_budget/tests -v
```

## AI assistance disclosure

This module, tests and documentation were prepared with large-language-model assistance. The submitter reviewed and ran the content and is responsible for the submission.
