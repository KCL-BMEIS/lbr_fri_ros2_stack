lbr_dual_arm_description
========================
This demo gives a walk through on using the ``lbr_description`` to generate a custom robot description. In this case, we will be investigating a dual-arm setup.

.. contents:: Table of Contents
   :depth: 2
   :local:
   :backlinks: none

Creating the Package
--------------------
#. Inside your workspace, create a package:

.. code-block:: bash

    ros2 pkg create lbr_dual_arm_description \
        --build-type ament_cmake \
        --license Apache-2.0 \
        --maintainer-name mhubii \
        --maintainer-email "m.huber_1994@hotmail.de" \
        --description "A dual LBR description package using lbr_description." \
        --dependencies lbr_description

#. Remove unused folders and create a Universal Robot Description File (URDF) folder:

.. code-block:: bash

    cd lbr_dual_arm_description && \
    rm -r lbr_dual_arm_description/include && \
    rm -r lbr_dual_arm_description/src && \
    mkdir urdf

Creating the Dual-arm Robot Description
---------------------------------------
#. Create a dual-arm robot description:

.. code-block:: bash

    touch urdf/lbr_dual_arm.xacro

#. Fill it with the following contents (note that `xacro <https://github.com/ros/xacro/tree/ros2>`_:octicon:`link-external` is used):

.. code-block:: XML

    <?xml version="1.0"?>

    <robot name="lbr_dual_arm" xmlns:xacro="http://www.ros.org/wiki/xacro">
        <!-- include the lbr iiwa macro -->
        <xacro:include filename="$(find lbr_description)/urdf/iiwa7/iiwa7_description.xacro" />

        <!-- add an arguent to allow for mock / hardware / gazebo -->
        <xacro:arg name="mode" default="mock" />

        <!-- add a base / floating link -->
        <link name="base_link" />

        <!-- add robot 1 via macro, note that different lbr_one_system_config.yaml are used (to
        configure port id) -->
        <xacro:iiwa7
            robot_name="lbr_one"
            mode="$(arg mode)"
            system_config_path="$(find lbr_dual_arm_description)/ros2_control/lbr_one_system_config.yaml" />

        <!-- add robot 2 via macro -->
        <xacro:iiwa7
            robot_name="lbr_two"
            mode="$(arg mode)"
            system_config_path="$(find lbr_dual_arm_description)/ros2_control/lbr_two_system_config.yaml" />
    </robot>

#. Create the ROS 2 Control configuration files:

    #. Create a configurations file for the controller manager and respective controllers:

        

    #. FRI system configurations (only necessary when hardware is used):

        .. code-block:: bash

            cp `ros2 pkg prefix lbr_description`/share/lbr_description/ros2_control/lbr_system_config.yaml ros2_control/lbr_one_system_config.yaml && \
            cp `ros2 pkg prefix lbr_description`/share/lbr_description/ros2_control/lbr_system_config.yaml ros2_control/lbr_two_system_config.yaml

#. Open ``lbr_two_system_config.yaml``:

    #. Change ``port_id`` to ``30201`` (or as desired, but different from ``lbr_one_system_config.yaml``)
    #. Update ``estimated_ft_sensor`` frames:

        .. code-black:: yaml

            chain_root: lbr_one_link_0 # and lbr_two_link_0 respectively
            chain_tip: lbr_one_link_ee # and lbr_two_link_0 respectively

#. Add the following to the ``CMakeLists.txt``:

    .. code-block:: cmake

        install(DIRECTORY ros2_control urdf
            DESTINATION share/${PROJECT_NAME}
        )



.. #. Build the package in your workspace:

..     .. code-block:: bash

..         colcon build --packages-select lbr_dual_arm_description --symlink-install
