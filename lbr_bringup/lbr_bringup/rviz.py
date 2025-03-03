from typing import Optional, Union

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


class RVizMixin:
    @staticmethod
    def arg_rviz(default_value: str = "false") -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="rviz",
            default_value=default_value,
            description="Whether to launch RViz.",
        )

    @staticmethod
    def arg_rviz_cfg_pkg(
        default_value: str = "lbr_bringup",
    ) -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="rviz_cfg_pkg",
            default_value=default_value,
            description="The package containing the RViz configuration file.",
        )

    @staticmethod
    def arg_rviz_cfg(
        default_value: str = "config/mock.rviz",
    ) -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="rviz_cfg",
            default_value=default_value,
            description="The RViz configuration file relative to rviz_cfg_pkg.",
        )

    @staticmethod
    def node_rviz(
        rviz_cfg_pkg: Optional[Union[LaunchConfiguration, str]] = LaunchConfiguration(
            "rviz_cfg_pkg", default="lbr_bringup"
        ),
        rviz_cfg: Optional[Union[LaunchConfiguration, str]] = LaunchConfiguration(
            "rviz_cfg", default="config/mock.rviz"
        ),
        **kwargs,
    ) -> Node:
        return Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=[
                "-d",
                PathJoinSubstitution(
                    [
                        FindPackageShare(rviz_cfg_pkg),
                        rviz_cfg,
                    ]
                ),
            ],
            **kwargs,
        )
