lbr_demos_advanced_py
=====================
.. warning::
    On hardware, do always execute in ``T1`` mode first.

.. contents:: Table of Contents
   :depth: 2
   :local:
   :backlinks: none

Admittance Controller
---------------------
This demo implements a simple admittance controller.

#. Client side configurations:

    #. Configure the ``client_command_mode`` to ``position`` in `lbr_system_config.yaml <https://github.com/lbr-stack/lbr_fri_ros2_stack/blob/humble/lbr_description/ros2_control/lbr_system_config.yaml>`_:octicon:`link-external`
    #. Set the ``update_rate`` to ``100`` in `lbr_controllers.yaml <https://github.com/lbr-stack/lbr_fri_ros2_stack/blob/humble/lbr_description/ros2_control/lbr_controllers.yaml>`_:octicon:`link-external`

#. Remote side configurations:

    #. .. dropdown:: Launch the ``LBRServer`` application on the ``KUKA smartPAD``

        .. thumbnail:: ../../doc/img/applications_lbr_server.png

    #. Select

        - ``FRI send period``: ``10 ms``
        - ``IP address``: ``your configuration``
        - ``FRI control mode``: ``POSITION_CONTROL``
        - ``FRI client command mode``: ``POSITION``

#. Launch the robot driver:

    .. code-block:: bash

        ros2 launch lbr_bringup hardware.launch.py \
            ctrl:=lbr_joint_position_command_controller \
            model:=iiwa7 # [iiwa7, iiwa14, med7, med14]

#. Run the `admittance_control <https://github.com/lbr-stack/lbr_fri_ros2_stack/blob/humble/lbr_demos/lbr_demos_advanced_py/lbr_demos_advanced_py/admittance_control_node.py>`_:octicon:`link-external` with remapping and parameter file:

    .. code-block:: bash

        ros2 run lbr_demos_advanced_py admittance_control --ros-args \
            -r __ns:=/lbr \
            --params-file `ros2 pkg prefix lbr_demos_advanced_py`/share/lbr_demos_advanced_py/config/admittance_control.yaml

#. Now gently move the robot at the end-effector.

Admittance Controller with Remote Center of Motion
--------------------------------------------------
This demo implements an admittance controller with a remote center of motion (RCM).

#. Client side configurations:

    #. Configure the ``client_command_mode`` to ``position`` in `lbr_system_config.yaml <https://github.com/lbr-stack/lbr_fri_ros2_stack/blob/humble/lbr_description/ros2_control/lbr_system_config.yaml>`_:octicon:`link-external`
    #. Set the ``update_rate`` to ``100`` in `lbr_controllers.yaml <https://github.com/lbr-stack/lbr_fri_ros2_stack/blob/humble/lbr_description/ros2_control/lbr_controllers.yaml>`_:octicon:`link-external`

#. Remote side configurations:

    #. .. dropdown:: Launch the ``LBRServer`` application on the ``KUKA smartPAD``

        .. thumbnail:: ../../doc/img/applications_lbr_server.png

    #. Select

        - ``FRI send period``: ``10 ms``
        - ``IP address``: ``your configuration``
        - ``FRI control mode``: ``POSITION_CONTROL``
        - ``FRI client command mode``: ``POSITION``

#. Launch the robot driver:

    .. code-block:: bash

        ros2 launch lbr_bringup hardware.launch.py \
            ctrl:=lbr_joint_position_command_controller \
            model:=iiwa7 # [iiwa7, iiwa14, med7, med14]

#. Run the `admittance_rcm_control <https://github.com/lbr-stack/lbr_fri_ros2_stack/blob/humble/lbr_demos/lbr_demos_advanced_py/lbr_demos_advanced_py/admittance_rcm_control_node.py>`_:octicon:`link-external` with remapping and parameter file:

    .. code-block:: bash

        ros2 run lbr_demos_advanced_py admittance_rcm_control --ros-args \
            -r __ns:=/lbr \
            --params-file `ros2 pkg prefix lbr_demos_advanced_py`/share/lbr_demos_advanced_py/config/admittance_rcm_control.yaml

#. Now gently move the robot at the end-effector.
