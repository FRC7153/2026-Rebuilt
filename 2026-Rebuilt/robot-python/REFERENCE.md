# 7153 Robot Code – Quick Reference

All commands are run in a terminal **inside the `robot-python` folder**.

## First-time setup (once per laptop)

1. Install **Python 3.14** (64-bit) from python.org. Tick "Add Python to PATH".
2. Get the code:
   ```
   git clone https://github.com/FRC7153/2026-Rebuilt.git
   cd 2026-Rebuilt/2026-Rebuilt/robot-python
   ```
3. Make the virtual environment and install everything:
   ```
   python -m venv .venv
   .venv\Scripts\activate
   pip install robotpy
   robotpy sync
   ```
   `robotpy sync` also downloads the packages the roboRIO needs, so do this **while you still have internet**.

## Every time you open a terminal

```
.venv\Scripts\activate
```
You'll see `(.venv)` at the start of the prompt. If a command says "robotpy not found", you forgot this.

## Deploy code to the robot

1. Connect to the robot (robot wifi, USB, or ethernet).
2. Run:
   ```
   robotpy deploy
   ```
3. Wait for "Deploy was successful". The code restarts on its own.

| Problem | Fix |
|---|---|
| Can't find the roboRIO | Check you're on the robot network; try USB; team number is 7153 |
| Tests fail before deploying | Fix them, or deploy anyway with `robotpy deploy --skip-tests` |
| Missing package on the robot | Get internet, run `robotpy sync`, reconnect, deploy again |
| Want to see robot prints | Driver Station console, or `robotpy deploy --nc` to watch live |

The **VS Code WPILib "Deploy Robot Code" button does not work** for Python – always use `robotpy deploy`.

## Other commands

| What | Command |
|---|---|
| Run all tests (do this before deploying) | `robotpy test` |
| Simulator | `robotpy sim` |
| Update packages after changing `pyproject.toml` | `robotpy sync` |

## Simulator + AdvantageScope

1. `robotpy sim` – a sim window opens.
2. AdvantageScope → File → Connect to Simulator → **NetworkTables 4** (not the AdvantageKit option).
3. In the sim window, drag a keyboard/controller onto Joystick 0 and click **Teleoperated** or **Autonomous**.
4. Robot pose: drag `DriveState/Pose` onto a 3D/2D Field. Fuel: drag `/Sim/FieldFuel` and `/Sim/InFlightFuel` on and choose Game Piece → Fuel.
5. Pick an auto from the "Auto Chooser" in the sim window or Elastic.

## Controls

**Driver (port 0)**

| Button | Does |
|---|---|
| Left stick / right stick | Drive / turn (field-centric) |
| **RT** | Aim at hub + shoot |
| LT | Extend intake + run rollers |
| RB | Stow intake |
| B | Reverse intake (spit out) |
| LB | Pass shot |
| Y | Point at hub (no shooting) |
| X | X-brake the wheels |
| D-pad up | Reset field-centric heading (face away from you, then press) |

**Operator (port 1, optional)**

| Button | Does |
|---|---|
| RT | Shoot |
| LT | Extend intake + run rollers |
| RB | Stow intake |
| LB | Stow while running rollers |
| Y | Partially raise intake |
| B | Reverse intake |
| A + driver RT | Distance-based shot |

## Where things are

| File | What's in it |
|---|---|
| `robotcontainer.py` | Button bindings, auto commands |
| `constants.py` | CAN IDs, PID gains, intake positions/speeds |
| `subsystems/swerve/tuner_constants.py` | Swerve IDs, offsets, gear ratios |
| `subsystems/` | Intake, shooter, Limelight, drivetrain |
| `commands/` | Shoot, intake, point-to-hub commands |
| `deploy/pathplanner/` | Autos and paths – edit these with the PathPlanner app |
| `physics.py`, `fuel_sim.py` | Simulator only, never runs on the robot |

## After a match / when something's wrong

- Driver Station console shows a `[handoff]` line after each auto→teleop switch – it says what (if anything) delayed driving.
- Logs (`.wpilog`) are saved on the robot's USB stick; open them in AdvantageScope.
- Tools that still work like before: Driver Station, AdvantageScope, Elastic, PathPlanner, Phoenix Tuner X, REV Hardware Client.

## Saving your changes to GitHub

```
git add -A
git commit -m "what you changed"
git push
```
Run `git pull` before you start working so you have everyone else's changes.
