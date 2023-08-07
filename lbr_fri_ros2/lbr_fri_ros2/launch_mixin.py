from typing import Dict

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


class LBRFRIROS2Mixin:
    @staticmethod
    def arg_command_guard_variant() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="command_guard_variant",
            default_value="safe_stop",
            description="Command guard variant.",
            choices=["default", "safe_stop"],
        )

    @staticmethod
    def arg_open_loop() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="open_loop",
            default_value="true",
            description="Open loop control. Works best for LBRs. Should only be set to false by experiences users.",
        )

    @staticmethod
    def param_command_guard_variant() -> Dict[str, LaunchConfiguration]:
        return {
            "command_guard_variant": LaunchConfiguration(
                "command_guard_variant", default="safe_stop"
            )
        }

    @staticmethod
    def param_open_loop() -> Dict[str, LaunchConfiguration]:
        return {"open_loop": LaunchConfiguration("open_loop", default="true")}

    @staticmethod
    def node_lbr_app(**kwargs) -> DeclareLaunchArgument:
        return Node(
            package="lbr_fri_ros2",
            executable="lbr_app",
            emulate_tty=True,
            output="screen",
            **kwargs,
        )