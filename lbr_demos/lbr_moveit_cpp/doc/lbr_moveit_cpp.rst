lbr_moveit_cpp
==============
.. note::

    Also refer to the official `MoveIt <https://moveit.picknik.ai/humble/doc/tutorials/your_first_project/your_first_project.html>`_:octicon:`link-external` documentation.

.. contents:: Table of Contents
   :depth: 2
   :local:
   :backlinks: none

MoveIt via C++ API
------------------
Simulation
~~~~~~~~~~
#. Run the robot driver:

    .. code-block:: bash

        ros2 launch lbr_bringup mock.launch.py \
            moveit:=true \
            model:=iiwa7 # [iiwa7, iiwa14, med7, med14]

#. Run MoveIt:

    .. code-block:: bash

        ros2 launch lbr_moveit_cpp move_group.launch.py \
            mode:=mock \
            model:=iiwa7 # [iiwa7, iiwa14, med7, med14]

#. Run the `hello_moveit <https://github.com/lbr-stack/lbr_fri_ros2_stack/blob/humble/lbr_demos/lbr_moveit_cpp/src/hello_moveit.cpp>`_:octicon:`link-external` node:

    .. code-block:: bash

        ros2 launch lbr_moveit_cpp hello_moveit.launch.py \
            mode:=mock \
            model:=iiwa7 # [iiwa7, iiwa14, med7, med14]

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

#. Proceed with steps 1, 2 and 3 from `Simulation`_ but with ``ros2 launch lbr_bringup hardware.launch.py``.

Examining the Code
~~~~~~~~~~~~~~~~~~
The source code for this demo is available on `GitHub <https://github.com/lbr-stack/lbr_fri_ros2_stack/tree/humble/lbr_demos/lbr_moveit_cpp>`_:octicon:`link-external`. The demo vastly follows the official `MoveIt <https://moveit.picknik.ai/humble/doc/tutorials/your_first_project/your_first_project.html>`_:octicon:`link-external` demo.

Differently, this repository puts the ``MoveGroup`` under a namespace. The ``MoveGroup`` is thus created as follows:

.. code-block:: cpp

    // Create MoveGroupInterface (lives inside robot_name namespace)
    auto move_group_interface = moveit::planning_interface::MoveGroupInterface(
        node_ptr, moveit::planning_interface::MoveGroupInterface::Options("arm", "robot_description",
                                                                        robot_name));

The ``MoveGroup`` configurations are parsed conveniently through a mixin:

.. code-block:: python

    from lbr_bringup.moveit import LBRMoveGroupMixin

    ...

    model = LaunchConfiguration("model").perform(context)

    # generate moveit configs
    moveit_configs = LBRMoveGroupMixin.moveit_configs_builder(
        robot_name=model,
        package_name=f"{model}_moveit_config",
    )

.. note::

    The MoveIt configurations might vary depending the user's configurations.
