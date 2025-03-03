from typing import List

from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from lbr_bringup.description import LBRDescriptionMixin
from lbr_bringup.moveit import LBRMoveGroupMixin
from lbr_bringup.rviz import RVizMixin


def hidden_setup(context: LaunchContext) -> List[LaunchDescriptionEntity]:
    ld = LaunchDescription()

    ld.add_action(LBRDescriptionMixin.arg_robot_name())
    ld.add_action(LBRMoveGroupMixin.arg_allow_trajectory_execution())
    ld.add_action(LBRMoveGroupMixin.arg_capabilities())
    ld.add_action(LBRMoveGroupMixin.arg_disable_capabilities())
    ld.add_action(LBRMoveGroupMixin.arg_monitor_dynamics())
    ld.add_action(LBRMoveGroupMixin.args_publish_monitored_planning_scene())

    model = LaunchConfiguration("model").perform(context)
    moveit_configs_builder = LBRMoveGroupMixin.moveit_configs_builder(
        robot_name=model,
        package_name=f"{model}_moveit_config",
    )
    move_group_params = LBRMoveGroupMixin.params_move_group()

    mode = LaunchConfiguration("mode").perform(context)
    use_sim_time = False
    if mode == "gazebo":
        use_sim_time = True

    # MoveGroup
    robot_name = LaunchConfiguration("robot_name")
    ld.add_action(
        LBRMoveGroupMixin.node_move_group(
            parameters=[
                moveit_configs_builder.to_dict(),
                move_group_params,
                {"use_sim_time": use_sim_time},
            ],
            namespace=robot_name,
        )
    )

    # RViz if desired
    rviz = RVizMixin.node_rviz(
        rviz_cfg_pkg=f"{model}_moveit_config",
        rviz_cfg="config/moveit.rviz",
        parameters=LBRMoveGroupMixin.params_rviz(
            moveit_configs=moveit_configs_builder.to_moveit_configs()
        )
        + [{"use_sim_time": use_sim_time}],
        remappings=[
            (
                "display_planned_path",
                PathJoinSubstitution([robot_name, "display_planned_path"]),
            ),
            ("joint_states", PathJoinSubstitution([robot_name, "joint_states"])),
            (
                "monitored_planning_scene",
                PathJoinSubstitution([robot_name, "monitored_planning_scene"]),
            ),
            ("planning_scene", PathJoinSubstitution([robot_name, "planning_scene"])),
            (
                "planning_scene_world",
                PathJoinSubstitution([robot_name, "planning_scene_world"]),
            ),
            (
                "robot_description",
                PathJoinSubstitution([robot_name, "robot_description"]),
            ),
            (
                "robot_description_semantic",
                PathJoinSubstitution([robot_name, "robot_description_semantic"]),
            ),
            (
                "recognized_object_array",
                PathJoinSubstitution([robot_name, "recognized_object_array"]),
            ),
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    ld.add_action(rviz)
    return ld.entities


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    ld.add_action(LBRDescriptionMixin.arg_mode())
    ld.add_action(LBRDescriptionMixin.arg_model())
    ld.add_action(RVizMixin.arg_rviz())

    ld.add_action(OpaqueFunction(function=hidden_setup))
    return ld
