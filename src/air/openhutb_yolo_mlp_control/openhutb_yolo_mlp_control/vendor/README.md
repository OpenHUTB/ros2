# Vendored OpenHUTB AirSim PythonClient

Run `tools/vendor_openhutb_pythonclient.ps1` before `colcon build`.

The script copies the exact `PythonClient/airsim` directory from the installed
OpenHUTB simulator into this directory and redirects its legacy `msgpackrpc`
imports to `msgpackrpc_compat.py`.

This avoids installing the 2018-era `msgpack-rpc-python` / Tornado 4 runtime in
the ROS 2 Jazzy Python 3.12 environment while retaining the OpenHUTB-shipped
AirSim client API source.
