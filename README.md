# 2026-Rebuilt

FRC 7153 robot code for the 2026 season (REBUILT), written in Python with [RobotPy](https://robotpy.readthedocs.io/).

The code lives in `2026-Rebuilt/robot-python`. Previous Java version: see the `main` branch history before the Python port.

## Setup (once per laptop)

```bash
cd 2026-Rebuilt/robot-python
python -m venv .venv
.venv\Scripts\activate        # Windows
pip install robotpy
robotpy sync
```

## Common commands (run inside `2026-Rebuilt/robot-python` with the venv active)

| What | Command |
|---|---|
| Deploy to the robot (connected to robot wifi/USB) | `robotpy deploy` |
| Run the tests | `robotpy test` |
| Run the simulator (connect AdvantageScope with "NetworkTables 4" to localhost) | `robotpy sim` |

The VS Code WPILib "Deploy Robot Code" button does not work for Python; use `robotpy deploy`.
WPILib tools (Driver Station, AdvantageScope, Elastic, PathPlanner, Phoenix Tuner, REV Hardware Client) all still work as before.
