"""Command that sends the dashboard a notification and logs it."""

from commands2 import InstantCommand

from libs import elastic
from util import console_logger


class NotificationCommand(InstantCommand):
    def __init__(self, title: str, message: str, level: elastic.NotificationLevel) -> None:
        def _notify() -> None:
            elastic.send_notification(elastic.Notification(level, title, message, 5000))

            if level == elastic.NotificationLevel.ERROR:
                console_logger.report_error(f"{title}: {message}")
            elif level == elastic.NotificationLevel.WARNING:
                console_logger.report_warning(f"{title}: {message}")
            else:
                print(f"{title}: {message}")

        super().__init__(_notify)
