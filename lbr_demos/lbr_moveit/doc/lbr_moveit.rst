lbr_moveit
==========
.. warning::
    On hardware, do always execute in ``T1`` mode first.

.. contents:: Table of Contents
   :depth: 2
   :local:
   :backlinks: none

MoveIt Servo
------------

MoveIt Servo - Simulation
~~~~~~~~~~~~~~~~~~~~~~~~~
#. Run the mock setup (important to run the ``forward_position_controller`` controller for servoing):

    .. code-block:: bash

        ros2 launch lbr_bringup mock.launch.py \
            ctrl:=forward_position_controller \
            model:=iiwa7 # [iiwa7, iiwa14, med7, med14]

   .. hint::
   
       For a physics-based simulation, also try Gazebo (remember to set ``mode:=gazebo`` for the next steps):
   
           .. code-block:: bash
   
               ros2 launch lbr_bringup gazebo.launch.py \
                   ctrl:=forward_position_controller \
                   model:=iiwa7 # [iiwa7, iiwa14, med7, med14]

#. Run MoveIt Servo:
    
    .. code-block:: bash

        ros2 launch lbr_bringup moveit_servo.launch.py \
            mode:=mock \
            model:=iiwa7 # [iiwa7, iiwa14, med7, med14]


#. Optionally run RViZ:

    .. code-block:: bash

        ros2 launch lbr_bringup rviz.launch.py

#. Publish to ``/lbr/servo_node/delta_joint_cmds`` and ``/lbr/servo_node/delta_twist_cmds``. For this demo, we provide a keyboard driver (keyboard layout is printed to terminal):

    .. code-block:: bash

        ros2 launch lbr_moveit keyboard_driver.launch.py

You can now experiment with

- Modifying the MoveIt Servo parameters in `moveit_servo.yaml <https://github.com/lbr-stack/lbr_fri_ros2_stack/blob/humble/lbr_bringup/config/moveit_servo.yaml>`_:octicon:`link-external`. E.g. the ``robot_link_command_frame`` to change the commanding frame.
- Connect a joystick or game controller.
- Or changing the veloctiy scales for this keyboard driver in `forward_keyboard.yaml <https://github.com/lbr-stack/lbr_fri_ros2_stack/blob/humble/lbr_demos/lbr_moveit/config/forward_keyboard.yaml>`_:octicon:`link-external`.

MoveIt Servo - Hardware
~~~~~~~~~~~~~~~~~~~~~~~
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

#. Proceed with steps 1, 2, 3 and 4 from `MoveIt Servo - Simulation`_ but with ``ros2 launch lbr_bringup hardware.launch.py`` in step 1.

MoveIt via RViz
---------------
.. image:: img/iiwa7_moveit_rviz.png
    :align: center
    :alt: MoveIt via RViz
**IIWA 7 R800 in RViz**

To run MoveIt via RViz, simply follow:

MoveIt via RViz - Simulation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#. Run the mock setup:

    .. code-block:: bash

        ros2 launch lbr_bringup mock.launch.py \
            model:=iiwa7 # [iiwa7, iiwa14, med7, med14]

   .. hint::
   
       For a physics-based simulation, also try Gazebo (remember to set ``mode:=gazebo`` for the next steps):
   
           .. code-block:: bash
   
               ros2 launch lbr_bringup gazebo.launch.py \
                   model:=iiwa7 # [iiwa7, iiwa14, med7, med14]

#. Run MoveIt with RViz:

    .. code-block:: bash

        ros2 launch lbr_bringup move_group.launch.py \
            mode:=mock \
            rviz:=true \
            model:=iiwa7 # [iiwa7, iiwa14, med7, med14]

#. You can now move the robot via MoveIt in RViz!

MoveIt via RViz - Hardware
~~~~~~~~~~~~~~~~~~~~~~~~~~
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

#. Proceed with steps 1 and 2 from `MoveIt via RViz - Simulation`_ but with ``ros2 launch lbr_bringup hardware.launch.py`` in step 1.
