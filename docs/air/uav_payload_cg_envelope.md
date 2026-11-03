---
title: UAV payload CG envelope
---

# UAV payload CG envelope

Combine empty-aircraft and payload masses to compute CG, longitudinal margins and lateral offset.

## 1. Prerequisites

- Python 3.10 or newer.
- No third-party runtime dependency for the offline calculation.
- ROS 2 Humble only for the installed launch entry.

## 2. Scope and limitations

This module provides a deterministic preflight calculation. The result must be checked against the actual airframe, mission constraints and measured system data before flight.

![UAV payload CG envelope](../img/uav_payload_cg_envelope.gif)

## 3. Run

```bash
PYTHONPATH=src/air/uav_payload_cg_envelope python3 -m uav_payload_cg_envelope.main
```

## 4. Test

```bash
PYTHONPATH=src/air/uav_payload_cg_envelope python3 -m unittest discover \
  -s src/air/uav_payload_cg_envelope/tests -v
```

## 5. Expected result

The runnable sample passes the CG envelope checks, followed by `PASS`.

## AI assistance disclosure

This module, tests and documentation were prepared with large-language-model assistance. The submitter reviewed and ran the content and is responsible for the submission.
