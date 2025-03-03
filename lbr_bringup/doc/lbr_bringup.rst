lbr_bringup
===========
The ``lbr_bringup`` package hosts some launch files, which can be included via standard procedures:

.. code:: python

    from launch import LaunchDescription
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource

    def generate_launch_description() -> LaunchDescription:
        ld = LaunchDescription()
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("lbr_bringup"),
                            "launch",
                            "mock.launch.py",
                        ]
                    )
                ),
            )
        )
        return ld

The launch files can also be run via the command line, as further described below.

.. contents:: Table of Contents
   :depth: 2
   :local:
   :backlinks: none

Launch Files
------------
Mock Setup
~~~~~~~~~~
Useful for running a physics-free simulation of the system. This launch file will (see `mock.launch.py <https://github.com/lbr-stack/lbr_fri_ros2_stack/blob/humble/lbr_bringup/launch/mock.launch.py>`_:octicon:`link-external`):

#. Run the ``robot_state_publisher``
#. Run the ``ros2_control_node`` with mock components as loaded from ``robot_description``
#. Load ``ros2_controllers``

.. code:: bash

    ros2 launch lbr_bringup mock.launch.py \
        model:=iiwa7 # [iiwa7, iiwa14, med7, med14]

.. note::
    List all arguments for the launch file via ``ros2 launch lbr_bringup mock.launch.py -s``.

Gazebo Simulation
~~~~~~~~~~~~~~~~~
Useful for running a physics simulation the the system. This launch file will will (see `gazebo.launch.py <https://github.com/lbr-stack/lbr_fri_ros2_stack/blob/humble/lbr_bringup/launch/gazebo.launch.py>`_:octicon:`link-external`):

#. Start the ``robot_state_publisher``
#. Start the ``Gazebo`` simulation
#. Spawn the selected robot model (includes the ``ros2_control_node`` within the ``Gazebo`` plugin)
#. Load ``ros2_controllers``

.. code:: bash

    ros2 launch lbr_bringup gazebo.launch.py \
        model:=iiwa7 # [iiwa7, iiwa14, med7, med14]

.. note::
    List all arguments for the launch file via ``ros2 launch lbr_bringup gazebo.launch.py -s``.

Hardware
~~~~~~~~
.. warning::
    Do always execute in ``T1`` mode first.

.. note::
    Make sure you have followed :doc:`Hardware Setup <../../lbr_fri_ros2_stack/doc/hardware_setup>` first.

#. Client side configurations:

    .. dropdown:: Launch the ``LBRServer`` application on the ``KUKA smartPAD``

        .. thumbnail:: ../../lbr_demos/doc/img/applications_lbr_server.png

    Select

    - ``FRI send period``: ``10 ms``
    - ``IP address``: ``your configuration``
    - ``FRI control mode``: ``POSITION_CONTROL`` or ``JOINT_IMPEDANCE_CONTROL``
    - ``FRI client command mode``: ``POSITION``

#. Launch file:

    This launch file will (see `hardware.launch.py <https://github.com/lbr-stack/lbr_fri_ros2_stack/blob/humble/lbr_bringup/launch/hardware.launch.py>`_:octicon:`link-external`):

    #. Run the ``robot_state_publisher``
    #. Run the ``ros2_control_node`` with the ``lbr_fri_ros2::SystemInterface`` plugin from :doc:`lbr_ros2_control <../../lbr_ros2_control/doc/lbr_ros2_control>` as loaded from ``robot_description`` (which will attempt to establish a connection to the real robot).
    #. Load ``ros2_controllers``

    .. code:: bash

        ros2 launch lbr_bringup hardware.launch.py \
            model:=iiwa7 # [iiwa7, iiwa14, med7, med14]

    .. note::
        List all arguments for the launch file via ``ros2 launch lbr_bringup hardware.launch.py -s``.

RViz
~~~~
This launch file will spin up ``RViz`` for visualization. It will (see `rviz.launch.py <https://github.com/lbr-stack/lbr_fri_ros2_stack/blob/humble/lbr_bringup/launch/rviz.launch.py>`_:octicon:`link-external`):

#. Read ``RViz`` configurations.
#. Run ``RViz``.

.. code:: bash

    ros2 launch lbr_bringup rviz.launch.py \
            rviz_cfg_pkg:=lbr_bringup \
            rviz_cfg:=config/mock.rviz # [gazebo.rviz, hardware.rviz, mock.rviz]

.. note::
    List all arguments for the launch file via ``ros2 launch lbr_bringup rviz.launch.py -s``.

.. note::
    Requires the user to run `Mock Setup`_, `Gazebo Simulation`_ or `Hardware`_ first.

MoveIt
~~~~~~
Please note that MoveIt configurations are specific and you as a user will need to create your own for your system (potentially containing multiple robots or an end-effector).

.. code:: bash

    ros2 launch lbr_bringup move_group.launch.py \
        model:=iiwa7 \
        mode:=mock \
        rviz:=true

.. note::
    Requires the user to run `Mock Setup`_, `Gazebo Simulation`_ or `Hardware`_ first.

.. note::
    Runs ``RViz`` with specific MoveIt configurations.

Mixins
------
The ``lbr_bringup`` package makes heavy use of mixins. Mixins are simply state-free classes with static methods. They are a convenient way of writing launch files.

The below shows an example of the `rviz.launch.py <https://github.com/lbr-stack/lbr_fri_ros2_stack/blob/humble/lbr_bringup/launch/rviz.launch.py>`_:octicon:`link-external` file:

.. code:: python

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

Which is quite compact and easy to read.

General Information on the FRI
------------------------------
The ``FRI`` lets the user select a ``FRI control mode`` and a ``FRI client command mode``. When running the ``LBRServer``:

- .. dropdown:: Select ``FRI control mode``

    .. thumbnail:: ../../lbr_fri_ros2_stack/doc/img/controller/raw/lbr_server_control_mode.png

- .. dropdown:: Select ``FRI client command mode``
    
    .. thumbnail:: ../../lbr_fri_ros2_stack/doc/img/controller/raw/lbr_server_client_command_mode.png

The ``FRI control mode`` specifies the mode in which the robot is controlled, and the ``FRI client command mode`` specifies the commands that the user sends.

Troubleshooting
---------------
Noisy Execution
~~~~~~~~~~~~~~~
- Frequency: Make sure the ``ros2_control_node`` frequency and the ``FRI send period`` are compatible, consider changing ``update_rate`` in `lbr_controllers.yaml <https://github.com/lbr-stack/lbr_fri_ros2_stack/tree/humble/lbr_ros2_control/config/lbr_controllers.yaml>`_:octicon:`link-external`. 
- Realtime priority: Set real time priority in ``code /etc/security/limits.conf``, add the line: ``user - rtprio 99``, where user is your username.
