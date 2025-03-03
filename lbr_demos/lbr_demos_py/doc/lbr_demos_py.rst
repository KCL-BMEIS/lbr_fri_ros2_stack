lbr_demos_py
============
.. warning::
    On hardware, do always execute in ``T1`` mode first.

.. contents:: Table of Contents
   :depth: 2
   :local:
   :backlinks: none

LBR Joint Position Command Controller (Hardware only)
-----------------------------------------------------
This demo uses the :ref:`lbr_fri_ros2::LBRJointPositionCommandController` and overlays a sinusoidal motion on joint ``A4``.

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

#. Run the robot driver:

    .. code-block:: bash

        ros2 launch lbr_bringup hardware.launch.py \
            ctrl:=lbr_joint_position_command_controller \
            model:=iiwa7 # [iiwa7, iiwa14, med7, med14]

#. Run the `joint_sine_overlay <https://github.com/lbr-stack/lbr_fri_ros2_stack/blob/humble/lbr_demos/lbr_demos_py/lbr_demos_py/joint_sine_overlay.py>`_:octicon:`link-external` node:

    .. code-block:: bash

        ros2 run lbr_demos_py joint_sine_overlay --ros-args -r __ns:=/lbr

    This node overlays a sinusoidal motion on joint ``A4``.

Joint Trajectory Controller
---------------------------
This demos uses the ``joint_trajectory_controller`` of ``ros2_controllers`` and moves the robot to a predefined configuration.

Simulation
~~~~~~~~~~
#. Launch the ``LBRBringup``:

    .. code-block:: bash

        ros2 launch lbr_bringup gazebo.launch.py \
            ctrl:=joint_trajectory_controller \
            model:=iiwa7 # [iiwa7, iiwa14, med7, med14]

#. Run the `joint_trajectory_client <https://github.com/lbr-stack/lbr_fri_ros2_stack/blob/humble/lbr_demos/lbr_demos_py/lbr_demos_py/joint_trajectory_client.py>`_:octicon:`link-external`:

    .. code-block:: bash

        ros2 run lbr_demos_py joint_trajectory_client --ros-args -r __ns:=/lbr

The robot will twist, then move to the zero configuration.

Hardware
~~~~~~~~
#. Client side configurations:

    #. Configure the ``client_command_mode`` to ``position`` in `lbr_system_config.yaml <https://github.com/lbr-stack/lbr_fri_ros2_stack/blob/humble/lbr_description/ros2_control/lbr_system_config.yaml>`_:octicon:`link-external`
    #. Set the ``update_rate`` to ``100`` in `lbr_controllers.yaml <https://github.com/lbr-stack/lbr_fri_ros2_stack/blob/humble/lbr_description/ros2_control/lbr_controllers.yaml>`_:octicon:`link-external`

#. Remote side configurations:

    #. .. dropdown:: Launch the ``LBRServer`` application on the ``KUKA smartPAD``

        .. thumbnail:: ../../doc/img/applications_lbr_server.png

    #. Select

        - ``FRI send period``: ``10 ms``
        - ``IP address``: ``your configuration``
        - ``FRI control mode``: ``POSITION_CONTROL`` or ``JOINT_IMPEDANCE_CONTROL``
        - ``FRI client command mode``: ``POSITION``

#. Proceed with steps 1 and 2 from `Simulation`_ but with ``ros2 launch lbr_bringup hardware.launch.py``.

LBR Torque Command Controller (Hardware only)
---------------------------------------------
This demo uses the :ref:`lbr_fri_ros2::LBRTorqueCommandController` and overlays a sinusoidal torque on joint ``A4``.

#. Client side configurations:

    #. Configure the ``client_command_mode`` to ``torque`` in `lbr_system_config.yaml <https://github.com/lbr-stack/lbr_fri_ros2_stack/blob/humble/lbr_description/ros2_control/lbr_system_config.yaml>`_:octicon:`link-external`
    #. Set the ``update_rate`` to ``500`` in `lbr_controllers.yaml <https://github.com/lbr-stack/lbr_fri_ros2_stack/blob/humble/lbr_description/ros2_control/lbr_controllers.yaml>`_:octicon:`link-external`

#. Remote side configurations:

    #. .. dropdown:: Launch the ``LBRServer`` application on the ``KUKA smartPAD``

        .. thumbnail:: ../../doc/img/applications_lbr_server.png

    #. Select

        - ``FRI send period``: ``2 ms``
        - ``IP address``: ``your configuration``
        - ``FRI control mode``: ``JOINT_IMPEDANCE_CONTROL``
        - ``FRI client command mode``: ``TORQUE``

#. Run the robot driver:

    .. code-block:: bash

        ros2 launch lbr_bringup hardware.launch.py \
            ctrl:=lbr_torque_command_controller \
            model:=iiwa7 # [iiwa7, iiwa14, med7, med14]

#. Run the `torque_sine_overlay <https://github.com/lbr-stack/lbr_fri_ros2_stack/blob/humble/lbr_demos/lbr_demos_py/lbr_demos_py/torque_sine_overlay.py>`_:octicon:`link-external` node:

    .. code-block:: bash

        ros2 run lbr_demos_py torque_sine_overlay --ros-args -r __ns:=/lbr

    This node overlays a sinusoidal torque on joint ``A4``.

LBR Wrench Command Controller (Hardware only)
---------------------------------------------
This demo uses the :ref:`lbr_fri_ros2::LBRWrenchCommandController` and overlays a sinusoidal force on the x- and y-axis.

#. Client side configurations:

    #. Configure the ``client_command_mode`` to ``wrench`` in `lbr_system_config.yaml <https://github.com/lbr-stack/lbr_fri_ros2_stack/blob/humble/lbr_description/ros2_control/lbr_system_config.yaml>`_:octicon:`link-external`
    #. Set the ``update_rate`` to ``500`` in `lbr_controllers.yaml <https://github.com/lbr-stack/lbr_fri_ros2_stack/blob/humble/lbr_description/ros2_control/lbr_controllers.yaml>`_:octicon:`link-external`

#. Remote side configurations:

    #. .. dropdown:: Launch the ``LBRServer`` application on the ``KUKA smartPAD``

        .. thumbnail:: ../../doc/img/applications_lbr_server.png

    #. Select

        - ``FRI send period``: ``2 ms``
        - ``IP address``: ``your configuration``
        - ``FRI control mode``: ``CARTESIAN_IMPEDANCE_CONTROL``
        - ``FRI client command mode``: ``WRENCH``

#. Run the robot driver:

    .. code-block:: bash

        ros2 launch lbr_bringup hardware.launch.py \
            ctrl:=lbr_wrench_command_controller \
            model:=iiwa7 # [iiwa7, iiwa14, med7, med14]

#. Run the `wrench_sine_overlay <https://github.com/lbr-stack/lbr_fri_ros2_stack/blob/humble/lbr_demos/lbr_demos_py/lbr_demos_py/wrench_sine_overlay.py>`_:octicon:`link-external` node:

    .. code-block:: bash

        ros2 run lbr_demos_py wrench_sine_overlay --ros-args -r __ns:=/lbr

    This node overlays a sinusoidal force on the x- and y-axis.