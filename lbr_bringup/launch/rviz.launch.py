from launch import LaunchDescription
from lbr_bringup.rviz import RVizMixin


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    # launch arguments
    ld.add_action(RVizMixin.arg_rviz_cfg())
    ld.add_action(RVizMixin.arg_rviz_cfg_pkg())

    # rviz
    ld.add_action(RVizMixin.node_rviz())
    return ld
