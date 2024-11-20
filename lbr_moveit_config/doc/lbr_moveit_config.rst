lbr_moveit_config
=================
Documentation for generating (:ref:`Generate MoveIt Configuration`) and updating (:ref:`Update MoveIt Configuration`) the ``MoveIt 2`` configurations for the LBRs. For full documentation see `MoveIt Documentation <https://moveit.picknik.ai/main/index.html>`_:octicon:`link-external`.

Generate MoveIt Configuration 
-----------------------------
This procedure applies to all LBRs: ``iiwa7``, ``iiwa14``, ``med7``, and ``med14``.

#. Make sure ``MoveIt 2`` is installed.

.. code-block:: bash

    sudo apt install ros-$ROS_DISTRO-moveit*

#. Make sure the ``lbr-stack`` is installed and **sourced**, see :ref:`Installation`.

#. Launch the setup assistant

.. code-block:: bash

    ros2 launch moveit_setup_assistant setup_assistant.launch.py

#. .. dropdown:: ``Load Files``: E.g. ``lbr-stack/install/lbr_description/share/lbr_description/urdf/iiwa7/iiwa7.xacro``

    .. thumbnail:: img/00_start_screen.png

#. .. dropdown:: ``Generate Collision Matrix``: Generate

    .. thumbnail:: img/01_self_collision.png

#. .. dropdown:: ``Virtual Joints``: Skip

    .. thumbnail:: img/02_virtual_joints.png

#. .. dropdown:: ``Planning Groups``: Add 

    .. thumbnail:: img/03_planning_groups.png

  #. .. dropdown:: ``Planning Groups``: Select ``Kinematic Solver`` and add ``Kinematic Chain``

        .. thumbnail:: img/03_define_planning_groups.png

  #. .. dropdown:: ``Kinematic Chain``: Configure

        .. thumbnail:: img/03_define_planning_groups_kinematic_chain.png

#. .. dropdown:: ``Robot Poses``: We add ``zero`` and ``transport``

    .. thumbnail:: img/04_robot_poses.png

#. .. dropdown:: ``End Effectors``: Skip (you might want to add one)

    .. thumbnail:: img/05_end_effectors.png

#. .. dropdown:: ``Passive Joints``: Skip

    .. thumbnail:: img/06_passive_joints.png

#. .. dropdown:: ``ROS 2 Control URDF``: Skip (defined in ``lbr_description``)

    .. thumbnail:: img/07_ros2_control.png

#. .. dropdown:: ``ROS 2 Controllers``: Skip (defined in ``lbr_bringup``)

    .. thumbnail:: img/08_ros2_controllers.png

#. .. dropdown:: ``MoveIt Controllers``: ``Auto Add FollowJointsTrajectory``

    .. thumbnail:: img/09_moveit_controllers.png

#. .. dropdown:: ``Perception``: Select ``None``  (you might want to add one)

    .. thumbnail:: img/10_perception.png

#. .. dropdown:: ``Launch Files``: Only add essential

    .. thumbnail:: img/11_launch_files.png

#. .. dropdown:: ``Author Information``: Add

    .. thumbnail:: img/12_author_information.png

#. .. dropdown:: ``Configuration Files``:

    .. thumbnail:: img/13_configuration_files.png

#. Manual changes:

    #. Manually add acceleration limits in `joint_limits.yaml <https://github.com/lbr-stack/lbr_fri_ros2_stack/blob/humble/lbr_moveit_config/iiwa7_moveit_config/config/joint_limits.yaml>`_:octicon:`link-external` (not supported in ``URDF``)
    
    #. In the `move_group.launch.py <https://github.com/lbr-stack/lbr_fri_ros2_stack/blob/humble/lbr_moveit_config/iiwa7_moveit_config/launch/move_group.launch.py>`_:octicon:`link-external` use the robot descriotion from ``lbr_description``
    
    #. In `moveit_controllers.yaml <https://github.com/lbr-stack/lbr_fri_ros2_stack/blob/humble/lbr_moveit_config/iiwa7_moveit_config/config/moveit_controllers.yaml>`_:octicon:`link-external` change the ``arm_controller`` to ``joint_trajectory_controller``, as in `lbr_controllers.yaml <https://github.com/lbr-stack/lbr_fri_ros2_stack/blob/humble/lbr_ros2_control/config/lbr_controllers.yaml>`_:octicon:`link-external` 

Update MoveIt Configuration 
---------------------------
#. Make sure the ``lbr-stack`` is installed and sourced, see :ref:`Installation`.

#. Run the setup assistant for the existing configuration.

    .. code-block:: bash

        ros2 launch iiwa7_moveit_config  setup_assistant.launch.py # [iiwa7, iiwa14, med7, med14]

#. Update and save the configurations, similar to :ref:`Generate MoveIt Configuration`.
