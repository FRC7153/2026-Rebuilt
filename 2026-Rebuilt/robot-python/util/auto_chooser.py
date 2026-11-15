"""Dashboard chooser for the autonomous routine.

Every PathPlanner auto found in deploy/pathplanner/autos is offered. The selected
auto is built ahead of time while the robot is disabled (building parses the path
files, which is too slow to do at the start of autonomous).
"""

import os
import traceback

import wpilib
from commands2 import Command, PrintCommand
from pathplannerlib.auto import AutoBuilder, PathPlannerAuto
from wpilib import Alert, SendableChooser, SmartDashboard

from util import console_logger

NO_AUTO = "No Auto"


class AutoChooser:
    def __init__(self) -> None:
        self._chooser = SendableChooser()
        self._loaded_command: Command | None = None
        self._loaded_name: str | None = None

        self._no_auto_loaded_alert = Alert("No auto loaded yet (run pregame)", Alert.AlertType.kInfo)

        self._chooser.setDefaultOption(NO_AUTO, NO_AUTO)
        for name in self._find_auto_names():
            self._chooser.addOption(name, name)

        # Anything loaded is stale as soon as the selection changes
        self._chooser.onChange(self._on_selection_changed)

        # Add the chooser to the dashboard
        SmartDashboard.putData("Auto", self._chooser)
        self._no_auto_loaded_alert.set(True)

    @staticmethod
    def _find_auto_names() -> list[str]:
        autos_dir = os.path.join(wpilib.getDeployDirectory(), "pathplanner", "autos")
        try:
            files = os.listdir(autos_dir)
        except OSError:
            console_logger.report_error(f"[AutoChooser] Could not list autos in '{autos_dir}'")
            return []
        return sorted(os.path.splitext(f)[0] for f in files if f.endswith(".auto"))

    def _on_selection_changed(self, _new_auto: str) -> None:
        self._loaded_command = None
        self._loaded_name = None
        self._no_auto_loaded_alert.set(True)

    def _build(self, name: str) -> Command:
        if name == NO_AUTO:
            return PrintCommand("No-op auto selected.")

        if not AutoBuilder.isConfigured():
            console_logger.report_error(
                f"[AutoChooser] AutoBuilder is not configured, cannot run '{name}'. Running no auto."
            )
            return PrintCommand(f"Auto '{name}' unavailable: AutoBuilder not configured.")

        try:
            # Autos have "resetOdom" enabled, so PathPlanner resets the pose to the
            # (alliance-flipped) starting pose itself when the auto starts.
            return PathPlannerAuto(name)
        except Exception:
            traceback.print_exc()
            console_logger.report_error(
                f"[AutoChooser] Could not load auto '{name}'. "
                "Check deploy/pathplanner/autos and deploy/pathplanner/paths. Running no auto."
            )
            return PrintCommand(f"Auto '{name}' failed to load.")

    def is_loaded(self) -> bool:
        """If the currently selected auto has been built."""
        selected = self._chooser.getSelected() or NO_AUTO
        return self._loaded_command is not None and self._loaded_name == selected

    def load_auto_command(self) -> None:
        """Builds the currently selected auto command (if it isn't built already)."""
        if self.is_loaded():
            return

        selected = self._chooser.getSelected() or NO_AUTO
        self._loaded_command = self._build(selected)
        self._loaded_name = selected

        print(f"[AutoChooser] Loaded: {selected}")
        self._no_auto_loaded_alert.set(False)

    def get_current_selected_command(self) -> Command:
        """The command for the currently selected auto, building it first if needed."""
        self.load_auto_command()
        assert self._loaded_command is not None
        return self._loaded_command
