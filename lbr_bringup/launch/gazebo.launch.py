from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from lbr_bringup.description import LBRDescriptionMixin
from lbr_bringup.gazebo import GazeboMixin
from lbr_bringup.ros2_control import LBRROS2ControlMixin


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    # launch arguments
    ld.add_action(LBRDescriptionMixin.arg_model())
    ld.add_action(LBRDescriptionMixin.arg_robot_name())
    ld.add_action(
        LBRROS2ControlMixin.arg_ctrl()
    )  # Gazebo loads controller configuration through lbr_description/gazebo/*.xacro from lbr_description/ros2_control/lbr_controllers.yaml

    # static transform world -> <robot_name>_floating_link
    world_robot_tf = [0, 0, 0, 0, 0, 0]  # keep zero
    ld.add_action(
        LBRDescriptionMixin.node_static_tf(
            tf=world_robot_tf,
            parent="world",
            child=PythonExpression(
                ["'", LaunchConfiguration("robot_name"), "' + '_floating_link'"]
            ),
        )
    )

    # robot description
    robot_description = LBRDescriptionMixin.param_robot_description(mode="gazebo")

    # robot state publisher
    robot_state_publisher = LBRROS2ControlMixin.node_robot_state_publisher(
        robot_description=robot_description, use_sim_time=True
    )
    ld.add_action(
        robot_state_publisher
    )  # Do not condition robot state publisher on joint state broadcaster as Gazebo uses robot state publisher to retrieve robot description

    # Gazebo
    ld.add_action(GazeboMixin.include_gazebo())  # Gazebo has its own controller manager
    ld.add_action(GazeboMixin.node_clock_bridge())
    ld.add_action(
        GazeboMixin.node_create(tf=world_robot_tf)
    )  # spawns robot in Gazebo through robot_description topic of robot_state_publisher

    # controllers
    joint_state_broadcaster = LBRROS2ControlMixin.node_controller_spawner(
        controller="joint_state_broadcaster"
    )
    ld.add_action(joint_state_broadcaster)
    ld.add_action(
        LBRROS2ControlMixin.node_controller_spawner(
            controller=LaunchConfiguration("ctrl")
        )
    )
    return ld
