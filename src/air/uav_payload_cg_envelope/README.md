# UAV payload center-of-gravity envelope

## Prerequisites

- Python 3.10 or newer.
- No third-party runtime dependency.
- ROS 2 Humble only for the installed launch entry.

## Scope

Combine empty-aircraft and payload masses to compute total mass,
three-dimensional CG, longitudinal margins and lateral offset.

The pass/fail decision uses only the longitudinal CG range and lateral offset.
`cg_z_m` is reported for information and does not constrain the current
envelope. Each mass is limited to 10,000 kg and the combined mass is also
limited to 10,000 kg. The result does not replace the airframe manufacturer's
loading chart.

## Run

```bash
PYTHONPATH=src/air/uav_payload_cg_envelope \
  python3 -m uav_payload_cg_envelope.main
```

## Test

Linux or macOS:

```bash
PYTHONPATH=src/air/uav_payload_cg_envelope \
  python3 -m unittest discover \
  -s src/air/uav_payload_cg_envelope/tests -v
```

Windows PowerShell:

```powershell
$env:PYTHONPATH="src/air/uav_payload_cg_envelope"
python -m unittest discover -s src/air/uav_payload_cg_envelope/tests -v
```

## AI assistance disclosure

This module, tests and documentation were prepared with large-language-model
assistance. The submitter reviewed and ran the content and is responsible for
the submission.
