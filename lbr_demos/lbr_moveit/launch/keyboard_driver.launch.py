from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from lbr_bringup.description import LBRDescriptionMixin


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    # launch arguments
    ld.add_action(
        DeclareLaunchArgument(
            name="keyboard_config_pkg",
            default_value="lbr_moveit",
            description="The package containing the keyboard configurations.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            name="keyboard_config",
            default_value="config/forward_keyboard.yaml",
            description="Location of the keyboard configuration file relative to keyboard_config_pkg.",
        )
    )
    ld.add_action(LBRDescriptionMixin.arg_robot_name())

    # forward keyboard node
    keyboard_driver_config = PathJoinSubstitution(
        [
            FindPackageShare(LaunchConfiguration("keyboard_config_pkg")),
            LaunchConfiguration("keyboard_config"),
        ]
    )
    ld.add_action(
        Node(
            package="lbr_moveit",
            executable="forward_keyboard",
            output="screen",
            parameters=[
                keyboard_driver_config,
            ],
            namespace=LaunchConfiguration("robot_name"),
        )
    )
    return ld
