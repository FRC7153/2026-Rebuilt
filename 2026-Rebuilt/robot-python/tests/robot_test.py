"""Robot-specific simulation tests: run every auto and press every button."""

import os

import ntcore
import pytest
import wpilib
import wpilib.simulation

AUTOS_DIR = os.path.join(os.path.dirname(__file__), "..", "deploy", "pathplanner", "autos")
AUTO_NAMES = sorted(os.path.splitext(f)[0] for f in os.listdir(AUTOS_DIR) if f.endswith(".auto"))


@pytest.mark.parametrize("auto_name", AUTO_NAMES)
def test_auto_runs(control, auto_name):
    """Selects each PathPlanner auto on the dashboard and runs it for a while."""
    with control.run_robot():
        control.step_timing(seconds=0.5, autonomous=True, enabled=False)

        # Select the auto the same way a dashboard does
        selected = (
            ntcore.NetworkTableInstance.getDefault()
            .getStringTopic("/SmartDashboard/Auto/selected")
            .publish()
        )
        selected.set(auto_name)
        control.step_timing(seconds=1.0, autonomous=True, enabled=False)

        active = ntcore.NetworkTableInstance.getDefault().getStringTopic("/SmartDashboard/Auto/active").subscribe("")
        assert active.get() == auto_name

        control.step_timing(seconds=6.0, autonomous=True, enabled=True)
        control.step_timing(seconds=0.5, autonomous=False, enabled=False)


def test_all_buttons(control):
    """Presses every button/trigger on both controllers during teleop."""
    with control.run_robot():
        control.step_timing(seconds=0.5, autonomous=False, enabled=False)

        ds = wpilib.simulation.DriverStationSim
        controllers = [wpilib.simulation.XboxControllerSim(0), wpilib.simulation.XboxControllerSim(1)]
        for port in (0, 1):
            ds.setJoystickAxisCount(port, 6)
            ds.setJoystickButtonCount(port, 10)
            ds.setJoystickPOVCount(port, 1)
        for sim in controllers:
            sim.setPOV(-1)
        ds.notifyNewData()

        control.step_timing(seconds=0.5, autonomous=False, enabled=True)

        # Drive around with the sticks
        controllers[0].setLeftY(-0.8)
        controllers[0].setLeftX(0.3)
        controllers[0].setRightX(0.5)
        ds.notifyNewData()
        control.step_timing(seconds=0.5, autonomous=False, enabled=True)

        for sim in controllers:
            for button in range(1, 11):
                sim.setRawButton(button, True)
                ds.notifyNewData()
                control.step_timing(seconds=0.2, autonomous=False, enabled=True)
                sim.setRawButton(button, False)

            for set_trigger in (sim.setLeftTriggerAxis, sim.setRightTriggerAxis):
                set_trigger(1.0)
                ds.notifyNewData()
                control.step_timing(seconds=0.2, autonomous=False, enabled=True)
                set_trigger(0.0)

            sim.setPOV(0)
            ds.notifyNewData()
            control.step_timing(seconds=0.2, autonomous=False, enabled=True)
            sim.setPOV(-1)

        # Distance shoot: driver A + right trigger
        controllers[0].setAButton(True)
        controllers[0].setRightTriggerAxis(1.0)
        ds.notifyNewData()
        control.step_timing(seconds=0.5, autonomous=False, enabled=True)

        control.step_timing(seconds=0.2, autonomous=False, enabled=False)


def _select_auto(control, auto_name):
    selected = (
        ntcore.NetworkTableInstance.getDefault().getStringTopic("/SmartDashboard/Auto/selected").publish()
    )
    selected.set(auto_name)
    control.step_timing(seconds=1.0, autonomous=True, enabled=False)
    return selected


def _push_driver_stick_forward():
    ds = wpilib.simulation.DriverStationSim
    driver = wpilib.simulation.XboxControllerSim(0)
    ds.setJoystickAxisCount(0, 6)
    ds.setJoystickButtonCount(0, 10)
    ds.setJoystickPOVCount(0, 1)
    driver.setPOV(-1)
    driver.setLeftY(-1.0)
    ds.notifyNewData()
    return driver


@pytest.mark.parametrize("disabled_gap", [0.0, 0.5])
def test_teleop_handoff(control, robot, disabled_gap):
    """The driver must have control of the drivetrain immediately after autonomous,
    even when the auto routine was still running when the period ended."""
    with control.run_robot():
        control.step_timing(seconds=0.5, autonomous=True, enabled=False)
        publisher = _select_auto(control, "BlueLBAuto")

        container = robot._robot_container
        drivetrain = container.drivetrain

        # Stop partway through the routine, like the end of the autonomous period would
        control.step_timing(seconds=3.0, autonomous=True, enabled=True)
        assert drivetrain.getCurrentCommand().getName() == "BlueLBAuto"

        _push_driver_stick_forward()
        if disabled_gap > 0.0:
            control.step_timing(seconds=disabled_gap, autonomous=False, enabled=False)

        control.step_timing(seconds=0.1, autonomous=False, enabled=True)

        assert drivetrain.getCurrentCommand().getName() == "TeleopDrive"
        fastest_target = max(abs(m.speed) for m in drivetrain.get_state().module_targets)
        assert fastest_target > 1.0, "drivetrain did not respond to the stick right after auto"
        del publisher


def test_sim_mechanisms(control, robot):
    """The simulated shooter and intake actually move (needed for a useful simulator)."""
    with control.run_robot():
        control.step_timing(seconds=0.5, autonomous=False, enabled=False)

        ds = wpilib.simulation.DriverStationSim
        operator = wpilib.simulation.XboxControllerSim(1)
        ds.setJoystickAxisCount(1, 6)
        ds.setJoystickButtonCount(1, 10)
        ds.setJoystickPOVCount(1, 1)
        operator.setPOV(-1)
        operator.setRightTriggerAxis(1.0)  # shoot
        operator.setLeftTriggerAxis(1.0)  # extend intake
        ds.notifyNewData()

        control.step_timing(seconds=3.0, autonomous=False, enabled=True)

        container = robot._robot_container
        assert container._shooter.get_shooter_velocity() == pytest.approx(24.25, abs=3.0)
        assert container._intake.get_intake_pivot_position() == pytest.approx(-0.83, abs=0.1)
        assert abs(container._intake.get_intake_velocity()) > 500


def _sim_int(name):
    return ntcore.NetworkTableInstance.getDefault().getIntegerTopic(f"/Sim/{name}").subscribe(-1)


def _operator(**axes):
    ds = wpilib.simulation.DriverStationSim
    operator = wpilib.simulation.XboxControllerSim(1)
    ds.setJoystickAxisCount(1, 6)
    ds.setJoystickButtonCount(1, 10)
    ds.setJoystickPOVCount(1, 1)
    operator.setPOV(-1)
    for axis, value in axes.items():
        getattr(operator, axis)(value)
    ds.notifyNewData()


def test_sim_fuel_intake(control, robot):
    """Driving the extended, running intake into fuel picks it up."""
    from wpimath.geometry import Pose2d, Rotation2d

    held = _sim_int("HeldFuel")
    with control.run_robot():
        control.step_timing(seconds=0.5, autonomous=False, enabled=False)
        # Facing -X with the (back) intake touching the edge of the neutral-zone fuel
        robot._robot_container.drivetrain.reset_pose(Pose2d(7.0, 4.035, Rotation2d.fromDegrees(180)))
        _operator(setLeftTriggerAxis=1.0)  # extend intake + run rollers
        control.step_timing(seconds=2.5, autonomous=False, enabled=True)
        assert held.get() > 8


def test_sim_fuel_shoot_scores(control, robot):
    """The standard shot from ShootAuto's spot puts preloaded fuel in the red hub."""
    from wpimath.geometry import Pose2d, Rotation2d

    shots = _sim_int("ShotsFired")
    red = _sim_int("Score/Red")
    with control.run_robot():
        control.step_timing(seconds=0.5, autonomous=False, enabled=False)
        robot._robot_container.drivetrain.reset_pose(Pose2d(13.45, 4.035, Rotation2d.fromDegrees(180)))
        _operator(setRightTriggerAxis=1.0)  # shoot
        control.step_timing(seconds=4.0, autonomous=False, enabled=True)
        assert shots.get() > 0
        assert red.get() > 0
