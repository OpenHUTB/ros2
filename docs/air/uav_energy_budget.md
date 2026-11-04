title: UAV energy budget

# UAV mission energy and endurance budget

This module estimates mission energy for hover, climb and cruise segments. It includes payload mass scaling, headwind-adjusted ground speed, battery efficiency and reserve.

## 1. Prerequisites

- Python 3.10 or newer.
- A mission profile JSON with battery, aircraft and segment parameters.
- ROS 2 Humble only when using the installed launch entry.

## 2. Scope and limitations

This is a simplified preflight estimate. It does not model cell voltage sag, thermal effects, nonlinear aerodynamics or wind gusts. Use measured flight data before operational use.

![UAV energy budget](../img/uav_energy_budget.gif)

## 3. Run

```bash
PYTHONPATH=src/air/uav_energy_budget python3 -m uav_energy_budget.main
```

## 4. Expected result

```text
PASS: mission fits within usable battery and reserve
```

## 5. Test

```bash
PYTHONPATH=src/air/uav_energy_budget python3 -m unittest discover \
  -s src/air/uav_energy_budget/tests -v
```

## AI assistance disclosure

This module, tests and documentation were prepared with large-language-model assistance. The submitter reviewed and ran the content and is responsible for the submission.
