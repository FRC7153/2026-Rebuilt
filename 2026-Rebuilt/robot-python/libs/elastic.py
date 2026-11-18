"""Minimal Python port of the Elastic dashboard helper (notifications + tab selection)."""

import json
from enum import Enum

import ntcore


class NotificationLevel(Enum):
    INFO = "INFO"
    WARNING = "WARNING"
    ERROR = "ERROR"


class Notification:
    def __init__(
        self,
        level: NotificationLevel = NotificationLevel.INFO,
        title: str = "",
        description: str = "",
        display_time_millis: int = 3000,
        width: float = 350,
        height: float = -1,
    ):
        self.level = level
        self.title = title
        self.description = description
        self.display_time_millis = display_time_millis
        self.width = width
        self.height = height

    def to_json(self) -> str:
        return json.dumps(
            {
                "level": self.level.value,
                "title": self.title,
                "description": self.description,
                "displayTime": self.display_time_millis,
                "width": self.width,
                "height": self.height,
            }
        )


_notification_pub: ntcore.StringPublisher | None = None
_selected_tab_pub: ntcore.StringPublisher | None = None


def send_notification(notification: Notification) -> None:
    """Shows a notification popup on the Elastic dashboard."""
    global _notification_pub
    if _notification_pub is None:
        topic = ntcore.NetworkTableInstance.get_default().get_string_topic("/Elastic/RobotNotifications")
        _notification_pub = topic.publish(ntcore.PubSubOptions(send_all=True, keep_duplicates=True))
    _notification_pub.set(notification.to_json())


def select_tab(tab: str | int) -> None:
    """Selects the Elastic tab with the given name (or index)."""
    global _selected_tab_pub
    if _selected_tab_pub is None:
        topic = ntcore.NetworkTableInstance.get_default().get_string_topic("/Elastic/SelectedTab")
        _selected_tab_pub = topic.publish(ntcore.PubSubOptions(keep_duplicates=True))
    _selected_tab_pub.set(str(tab))
