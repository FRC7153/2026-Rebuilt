# SystemCore branch (2027 alpha)

This branch runs the robot code on **SystemCore** with RobotPy **2027.0.0a8**.
For the roboRIO, use the `main` branch – nothing here is merged into it.

## Using it

```
git checkout systemcore
robotpy sync        # installs the 2027 packages into your venv - see note below
robotpy test
robotpy deploy
```

Your venv can only hold one year's packages. Use a separate venv (or clone) for this
branch, otherwise switching back to `main` needs another `robotpy sync`.

## What works

- Teleop driving, intake, shooter, aim-at-hub, every button binding
- Logging, alerts, CAN logging, handoff monitor
- `robotpy test` (13 tests) and `robotpy sim` boot and run cleanly

## Not available yet (waiting on 2027 libraries)

| Missing | Effect | When it's released |
|---|---|---|
| PathPlannerLib 2027 | **No autos** – the chooser only offers "No Auto" | Add `robotpy-pathplannerlib==2027.x` to `pyproject.toml`; the code turns autos back on automatically |
| pyfrc 2027 that installs with WPILib 2027 | No sim physics / fuel sim (`physics.py` is not loaded), so mechanisms don't move in sim | Restore the pyfrc tests from `main` |

## Things to check on the real robot

- **CAN port**: the SPARKs are assumed to be on SystemCore port `CAN_S2`
  (`HardwareConstants.SYSTEMCORE_CAN_PORT` in `constants.py`). Change it if wired elsewhere.
- **Auto selection**: there is no SendableChooser in 2027. The options are published to
  `Auto/Options` and the choice is read from the tunable `Auto/Selected`.
- **"Test" mode is now "Utility" mode** in 2027.

## What changed from `main`

- Every WPILib/REV call renamed to 2027's snake_case (`whileTrue` → `while_true`, `pose.X()` → `pose.x`, …)
- `robotInit` → `__init__`; `DriverStation` mode/alliance calls → `RobotState` / `MatchState`
- `ChassisSpeeds` → `ChassisVelocities`; Phoenix `ApplyRobotSpeeds` → `ApplyRobotVelocity`
- `SmartDashboard.putData` → `telemetry.log`; Alerts need an id; REV getters return signals (`.get()`)
- AprilTag poses are read from `deploy/apriltagfiles/*.fmap` (2027 Python has no `AprilTagFieldLayout`)
