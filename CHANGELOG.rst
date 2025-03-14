^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package LBR FRI ROS 2 Stack
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Humble v2.2.0 (2024-11-20)
--------------------------
This release backports new ``rolling`` features to ``humble``. Following has changed:

* Related PRs:

  * https://github.com/lbr-stack/lbr_fri_ros2_stack/pull/213 and https://github.com/lbr-stack/lbr_fri_ros2_stack/pull/214

    * Joints and links are now prefixed with ``lbr_`` (i.e. the robot name)
    * Robot state publisher has no ``lbr/`` prefix anymore
    * Asynchronous + deactivateable force-torque estimation from external torques (previously synchronous)
    * Issue with setting real-time priority fixed
    * Modifiable source for ``lbr_system_config.yaml`` in launch files

  * https://github.com/lbr-stack/lbr_fri_ros2_stack/pull/220

    * PID on joint position commands replaced by simpler exponential filter (please test robot in T1 mode as this will affect your control)
    * Introduction of twist and admittance controllers
    * Configurations from ``lbr_ros2_control`` now in ``lbr_description`` (for stand alone URDF use)

Humble v2.1.2 (2024-10-18)
--------------------------
* Adds MoveIt Servo demo, related to https://github.com/lbr-stack/lbr_fri_ros2_stack/issues/50 and https://github.com/lbr-stack/lbr_fri_ros2_stack/issues/211

  * ``lbr_bringup``: New launch mixin and launch file for MoveIt Servo
  * ``lbr_moveit``: New keyboard driver to interface with MoveIt Servo

Humble v2.1.1 (2024-09-27)
--------------------------
* Adds support for the new Gazebo and removes support for Gazebo Classic (End-of-Life January 2025, refer https://community.gazebosim.org/t/gazebo-classic-end-of-life/2563).

  * ``lbr_bringup``: Updated launch files and dependencies.
  * ``lbr_description``: Updated ``<gazebo>`` tag to include Gazebo plugin (see https://github.com/ros-controls/gz_ros2_control/tree/humble). 
  * ``lbr_ros2_control``: Changed ``gazebo_ros2_control/GazeboSystem`` -> ``ign_ros2_control/IgnitionSystem```

Humble v2.1.0 (2024-09-10)
--------------------------
* De-couple launch files from ``lbr_bringup`` for easier customization (breaking change):

  * Removed ``sim:=true / false`` argument from launch files in favor of dedicated launch files (since no feature parity between simulation and real robot)
  * MoveIt and RViz need to be launched separately now
  * User can now launch via:

    * ``ros2 launch lbr_bringup mock.launch.py`` (new: mock system)
    * ``ros2 launch lbr_bringup hardware.launch.py`` (real robot)
    * ``ros2 launch lbr_bringup gazebo.launch.py`` (Gazebo simulation)
* Added mock hardware to ``lbr_ros2_control`` (for simple ``ros2_control`` testing without the need for Gazebo, refer https://control.ros.org/humble/doc/ros2_control/hardware_interface/doc/mock_components_userdoc.html)
* Updated documentation

Humble v2.0.0 (2024-07-08)
--------------------------
* Remove ``app.launch.py`` from demos in favor for ``ros2_control`` variant (breaking change)
* ``lbr_ros2_control``:

  * Add Cartesian impedance controller
  * Add ``lbr_system_parameters.yaml`` for system parameter configuration
* ``lbr_demos``:

  * Remove legacy demos
  * Add demo for each available controller, both in C++ and Python
* ``lbr_fri_ros2``: De-couple the async client into ``position`` / ``torque`` / ``wrench``
* Matrix testing against multiple FRIs (https://github.com/lbr-stack/fri): ``1.11``, ``1.14``, ``1.15``, ``1.16``, ``2.5``, ``2.7``
* IDL changes: (breaking change)

  * ``lbr_fri_msgs`` to ``lbr_fri_idl``
  * Moved ``lbr_fri_idl`` into external folder (https://github.com/lbr-stack/lbr_fri_idl) for supporting multiple FRI versions
  * Renamed ``LBRPositionCommand`` to ``LBRJointPositionCommand``
* Update documentation

  * Refer https://lbr-stack.readthedocs.io/en/latest/
  * Deletes branches at https://github.com/lbr-stack/lbr_stack_doc in favor of tags
  * Adds an architecture chart to highlight ``lbr_ros2_control`` relation to ``ros2_control``
* Add log coloring a la https://github.com/ros-controls/ros2_control/blob/e149646d3f2595f269cfa4e1cd0681abde89ee69/controller_manager/controller_manager/spawner.py#L45
* Adds ``black`` linting for Python scripts
* Fixes velocity limit checks in impedance control mode
* Add development tools dependency https://github.com/lbr-stack/lbr_fri_ros2_stack/issues/145
* ``ros2_control_node``: Read robot description from robot state publisher
* Add tests to ``lbr_fri_idl`` and ``lbr_fri_ros2``
* Moves all launch mixins to ``lbr_bringup``
* Update parameter source for ``gazebo_ros2_control`` package in ``lbr.gazebo.xacro``

Humble v1.4.3 (2024-02-03)
--------------------------
* Fixes planning scene namespace https://github.com/lbr-stack/lbr_fri_ros2_stack/issues/153
* Refers to https://github.com/lbr-stack/lbr_fri_ros2_stack/pull/155

Humble v1.4.2 (2023-12-29)
--------------------------
* Fixes cartesian path for move group node with namespace: https://github.com/lbr-stack/lbr_fri_ros2_stack/issues/133
* Solution @josefinemonnet: https://github.com/ros-planning/moveit2/issues/2545#issuecomment-1868480168
* Refers to https://github.com/lbr-stack/lbr_fri_ros2_stack/pull/146

Humble v1.4.1 (2023-12-15)
--------------------------
* Removes the ``base_frame`` parameter from ``lbr_bringup``, ``lbr_description``, ``lbr_fri_ros2``, ``lbr_ros2_control``
* Updates RViz default config in ``lbr_moveit_config``
* Refers to https://github.com/lbr-stack/lbr_fri_ros2_stack/pull/144

Humble v1.4.0 (2023-12-08)
--------------------------
* The general goal of this release is a tighter ``ros2_control`` integration. The ``lbr_bringup``
  will serve as single entry point in the future. For now, ``app_component`` and ``app.launch.py`` are kept
* Changes to ``lbr_fri_ros2``:

  * Removes logging / parameter interfaces from ``lbr_fri_ros2`` (so ``lbr_ros2_control`` serves as single interaction point)
  * Updates legacy ``app_component`` in ``lbr_fri_ros2`` for changes. To be depracted in the future
  * Adds force-torque estimator to ``lbr_fri_ros2``
* Changes to ``lbr_ros2_control``:

  * Removes now redundant node from ``lbr_ros2_control``
  * Adds forward position and forward torque controllers to ``lbr_ros2_control``
  * Removes estimated force-torque broadcaster from ``lbr_ros2_control`` in favor of ``ros2_control`` default implementation
    
    * Force-torque now available under ``/lbr/force_torque_broadcaster/wrench`` 
    * Namespace issues since ``lbr_controllers.yaml`` includes namespace in ``frame_id`` parameter
  * Adds ``lbr_fri_ros2`` force-torque estimator to ``lbr_ros2_control`` as sensor
  * Adds configurations to ``lbr_system_interface.xacro``
  * Simplifies ``lbr_ros2_control`` class names
* ``/lbr/command/position`` topic now under ``/lbr/command/joint_position``
* Adds this changelog with release notes
* Refers to https://github.com/lbr-stack/lbr_fri_ros2_stack/pull/142

Humble v1.3.1 (2023-11-21)
--------------------------
* v1.3.0 Gazebo namespace fixes in https://github.com/lbr-stack/lbr_fri_ros2_stack/pull/123
* Fix iiwa ee link in https://github.com/lbr-stack/lbr_fri_ros2_stack/pull/126
* Humble v.1.3.1 in https://github.com/lbr-stack/lbr_fri_ros2_stack/pull/130
* Full log: https://github.com/lbr-stack/lbr_fri_ros2_stack/compare/humble-v1.3.0-beta...humble-v1.3.1

Humble v1.3.0 beta (2023-10-03)
-------------------------------
* Namespaced robot_description and joint_states
* De-coupled commands, user will interact through LBRPositionCommand, LBRTorqueCommand, LBRWrenchCommand
* Multi-robot support
* New command / state interfaces in lbr_fri_ros2
* Topic free ros2_control support through command / state interfaces in lbr_fri_ros2
* Intraprocess cpp admittance demo
* New app component based on command / state interfaces in lbr_fri_ros2
* Refers to https://github.com/lbr-stack/lbr_fri_ros2_stack/pull/121

Humble v1.2.5 (2023-09-08)
--------------------------
* Updated visualization (STL -> DAE files with materials, might occur dark in Gazebo, caused by lack of light)
* Fixes joint bug in Gazebo
* Improved logging in command guard

Humble v1.2.4 (2023-08-09)
--------------------------
* Remove robot name from configs and use frame_prefix from robot state publisher instead
* Removed robot name from joint names, e.g. lbr_A1 -> A1
* Added PID for asynchronous control rate
* Simplified class names, e.g. LBRApp -> App
* Add utils.hpp for PID and exponential filter

Humble v1.2.3 (2023-08-07)
--------------------------
* Utilizes FRI through vendor package for common fri source in https://github.com/lbr-stack/
* Addresses some of https://github.com/lbr-stack/lbr_fri_ros2_stack/pull/85
* Give command guard only logger interface
* Fix open loop bug
* Adds real-time priority via rt_prio parameter

Humble v1.2.2 (2023-08-05)
--------------------------
* Adds base frame parameter to URDF and launch
* Adds an open loop option to control the robot, which works extremely well
* Updates logo in readme
* Updates joint names to KUKA convention, i.e. A1,...

Humble v1.2.1 (2023-08-04)
--------------------------
* Stack's new home at: https://github.com/lbr-stack

Humble v1.2.0 (2023-08-03)
--------------------------
* Re-introduces MoveIt, refer to https://github.com/lbr-stack/lbr_fri_ros2_stack/issues/52
* Moves demo prefix to front for improved package overview
* Single node for hardware interface
* Static executors where possible
* Adds plenty documentation
* Introduce /lbr, i.e. robot name, namespace to LBRClient for better multi-robot support. Commands / states now e.g. published to /lbr/command / /lbr/state
* Hardware interface exact limits (stand-alone use has safety-limits)
* Gives command guard a node handle
