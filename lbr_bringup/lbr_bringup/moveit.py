import os
from typing import Any, Dict, List, Optional, Union

from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import (
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigs, MoveItConfigsBuilder


class LBRMoveGroupMixin:
    @staticmethod
    def arg_allow_trajectory_execution() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="allow_trajectory_execution",
            default_value="true",
        )

    @staticmethod
    def args_publish_monitored_planning_scene() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="publish_monitored_planning_scene",
            default_value="true",
        )

    @staticmethod
    def arg_capabilities() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="capabilities",
            default_value="",
            description="Non-default space separated non-default list of MoveGroup capabilities.",
        )

    @staticmethod
    def arg_disable_capabilities() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="disable_capabilities",
            default_value="",
            description="Disable MoveGroup capabilities, space separated list.",
        )

    @staticmethod
    def arg_monitor_dynamics() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="monitor_dynamics",
            default_value="false",
            description="Whether to copy robot dynamics into MoveGroup.",
        )

    @staticmethod
    def moveit_configs_builder(
        robot_name: str, package_name: str
    ) -> MoveItConfigsBuilder:
        return (
            MoveItConfigsBuilder(
                robot_name=robot_name,
                package_name=package_name,
            )
            .robot_description(
                os.path.join(
                    get_package_share_directory("lbr_description"),
                    f"urdf/{robot_name}/{robot_name}.xacro",
                ),
            )
            .planning_pipelines(
                default_planning_pipeline="ompl",
                pipelines=["ompl"],
            )
        )

    @staticmethod
    def params_move_group() -> Dict[str, Any]:
        move_group_configuration = {
            "publish_robot_description_semantic": True,
            "allow_trajectory_execution": LaunchConfiguration(
                "allow_trajectory_execution"
            ),
            # Note: Wrapping the following values is necessary so that the parameter value can be the empty string
            "capabilities": ParameterValue(
                LaunchConfiguration("capabilities"), value_type=str
            ),
            "disable_capabilities": ParameterValue(
                LaunchConfiguration("disable_capabilities"), value_type=str
            ),
            # Publish the planning scene of the physical robot so that rviz plugin can know actual robot
            "publish_planning_scene": LaunchConfiguration(
                "publish_monitored_planning_scene"
            ),
            "publish_geometry_updates": LaunchConfiguration(
                "publish_monitored_planning_scene"
            ),
            "publish_state_updates": LaunchConfiguration(
                "publish_monitored_planning_scene"
            ),
            "publish_transforms_updates": LaunchConfiguration(
                "publish_monitored_planning_scene"
            ),
            "monitor_dynamics": False,
        }
        return move_group_configuration

    @staticmethod
    def params_rviz(
        moveit_configs: MoveItConfigs,
    ) -> List[Dict[str, Any]]:
        return [
            moveit_configs.planning_pipelines,
            moveit_configs.robot_description_kinematics,
        ]

    @staticmethod
    def node_move_group(**kwargs) -> Node:
        return Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            **kwargs,
        )


class LBRMoveItServoMixin:
    @staticmethod
    def arg_default_enable_servo() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="default_enable_servo",
            default_value="true",
            description="Whether to enable the servo node by default.",
        )

    @staticmethod
    def node_moveit_servo(
        robot_name: Optional[Union[LaunchConfiguration, str]] = LaunchConfiguration(
            "robot_name", default="lbr"
        ),
        **kwargs,
    ) -> Node:
        return Node(
            package="moveit_servo",
            executable="servo_node_main",
            output="screen",
            namespace=robot_name,
            **kwargs,
        )

    @staticmethod
    def call_start_servo_service(
        robot_name: Optional[Union[LaunchConfiguration, str]] = LaunchConfiguration(
            "robot_name", default="lbr"
        ),
        **kwargs,
    ) -> ExecuteProcess:
        return ExecuteProcess(
            cmd=[
                FindExecutable(name="ros2"),
                "service",
                "call",
                PathJoinSubstitution([robot_name, "servo_node/start_servo"]),
                "std_srvs/srv/Trigger",
            ],
            **kwargs,
        )
