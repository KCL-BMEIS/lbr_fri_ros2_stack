lbr_ros2_control
================
.. note::

    - Refer to :ref:`lbr_demos` for exemplary usage
    - Also refer to the official `ros2_control <https://control.ros.org/humble/index.html>`_:octicon:`link-external` documentation

.. contents:: Table of Contents
   :depth: 2
   :local:
   :backlinks: none

General
-------
This package implements:

#. A ``hardware_interface::SystemInterface`` for ``ros2_control`` integration: `Hardware Plugin`_
#. Dedicated ``ros2_controllers`` for various command modes and the state: `Controller Plugins`_

A simplified overview of ``lbr_ros2_control`` is shown :ref:`below <lbr_ros2_control simplified software architecture figure>` (click to expand):

.. _lbr_ros2_control simplified software architecture figure:
.. thumbnail:: img/lbr_ros2_control_v2.0.0.svg
    :alt: lbr_ros2_control

New starters are often confused by ``ros2_control``. Everything in ``ros2_control`` is a plugin, refer `pluginlib <https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Pluginlib.html>`_:octicon:`link-external`.

Hardware components and controllers are loaded as plugins (components) by the ``controller_manager::ControllerManager`` during runtime. Therefore, hardware and controllers communicate over shared memory in the same process, **not** topics! 

The ``controller_manager::ControllerManager`` has a node, the `controller_manager <https://github.com/ros-controls/ros2_control/blob/humble/controller_manager/src/ros2_control_node.cpp>`_:octicon:`link-external`. 

- Hardware plugins are read from the ``robot_descritption`` parameter of the ``robot_state_publisher`` node and loaded at runtime.
- Parameters, such as ``update_rate``, the configured controllers, are simply set as node parameters, see e.g. `lbr_controllers.yaml <https://github.com/lbr-stack/lbr_fri_ros2_stack/blob/humble/lbr_description/ros2_control/lbr_controllers.yaml>`_:octicon:`link-external`.

Hardware Plugin
---------------
lbr_ros2_control::SystemInterface
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
The ``lbr_ros2_control::SystemInterface`` plugin implements a ``hardware_interface::SystemInterface`` and utilizes the :ref:`lbr_fri_ros2` package for communication with the robot. Overview :ref:`below <lbr_ros2_control detailed software architecture figure>` (click to expand):

.. _lbr_ros2_control detailed software architecture figure:
.. thumbnail:: img/lbr_ros2_control_detailed_v2.0.0.svg
    :alt: lbr_ros2_control_detailed

- The ``controller_manager::ControllerManager`` loads the correct plugin from the ``<ros2_control>`` tag in the ``robot_description``: `lbr_system_interface.xacro <https://github.com/lbr-stack/lbr_fri_ros2_stack/blob/humble/lbr_description/ros2_control/lbr_system_interface.xacro>`_:octicon:`link-external`
- FRI relevant parameters are forwarded to the ``lbr_ros2_control::SystemInterface`` plugin from `lbr_system_config.yaml <https://github.com/lbr-stack/lbr_fri_ros2_stack/blob/humble/lbr_description/ros2_control/lbr_system_config.yaml>`_:octicon:`link-external`

The ``lbr_ros2_control::SystemInterface`` is spun with the ``controller_manager`` node at a rate set by the ``update_rate`` parameter. The communication to the robot is run **asynchronously** at a rate set by the robot's sample time.

**Why asynchronously**? KUKA designed the FRI that way, by adhering to this design choice, we can support multiple FRI versions, see :ref:`fri`!


Controller Plugins
------------------
Simple controller plugins for exposing the robot commands and states as topics. Utilizes :ref:`lbr_fri_idl` message definitions.

lbr_fri_ros2::LBRJointPositionCommandController
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Exposes the robot command in ``POSITION`` client command mode as ``LBRJointPositionCommand`` message.

- Supported control modes:

  - ``POSITION_CONTROL``
  - ``JOINT_IMPEDANCE_CONTROL``
  - ``CARTESIAN_IMPEDANCE_CONTROL``
- Topic: ``command/joint_position``

lbr_fri_ros2::LBRTorqueCommandController
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Expose the robot command in ``TORQUE`` client command mode as ``LBRTorqueCommand`` message.

- Supported control modes: ``TORQUE_CONTROL`` 
- Topic: ``command/torque``


lbr_fri_ros2::LBRWrenchCommandController
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Expose the robot command in ``WRENCH`` client command mode as ``LBRWrenchCommand`` message.

- Supported control modes: ``CARTESIAN_IMPEDANCE_CONTROL`` 
- Topic: ``command/wrench``

lbr_fri_ros2::LBRStateBroadcaster
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Exposes the robot state as ``LBRState`` message.

- Any client command mode
- Any control mode
- Topic: ``state``
