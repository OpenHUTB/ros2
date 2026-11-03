# UAV payload center-of-gravity envelope

## Prerequisites

- Python 3.10 or newer.
- No third-party runtime dependency.

## Scope

Combine empty-aircraft and payload masses to compute total mass, three-dimensional CG, longitudinal margins and lateral offset. The profile defines a simplified longitudinal/lateral envelope and does not replace the airframe manufacturer?s loading chart.

## Run

```bash
PYTHONPATH=src/air/uav_payload_cg_envelope python3 -m uav_payload_cg_envelope.main
```

## Test

```bash
PYTHONPATH=src/air/uav_payload_cg_envelope python3 -m unittest discover -s src/air/uav_payload_cg_envelope/tests -v
```

## AI assistance disclosure

This module, tests and documentation were prepared with large-language-model assistance. The submitter reviewed and ran the content and is responsible for the submission.
