---
title: UAV radio link budget
---

# UAV radio link budget

Estimate free-space communication range from transmit power, antenna gains, sensitivity, fade margin and losses.

## 1. Prerequisites

- Python 3.10 or newer.
- No third-party runtime dependency for the offline calculation.
- ROS 2 Humble only for the installed launch entry.

## 2. Scope and limitations

This module provides a deterministic preflight calculation. The result must be checked against the actual airframe, mission constraints and measured system data before flight.

![UAV radio link budget](../img/uav_radio_link_budget.gif)

## 3. Run

```bash
PYTHONPATH=src/air/uav_radio_link_budget python3 -m uav_radio_link_budget.main
```

## 4. Test

```bash
PYTHONPATH=src/air/uav_radio_link_budget python3 -m unittest discover \
  -s src/air/uav_radio_link_budget/tests -v
```

## 5. Expected result

The runnable sample passes the radio-link checks, followed by `PASS`.

## AI assistance disclosure

This module, tests and documentation were prepared with large-language-model assistance. The submitter reviewed and ran the content and is responsible for the submission.
