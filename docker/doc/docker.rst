docker
======
To run the ``lbr_fri_ros2_stack`` in a Docker container, follow the instructions below.

#. Install ``vcstool``

   .. code-block:: bash

    pip3 install vcstool

#. Create a workspace and clone

   .. code-block:: bash

    export FRI_CLIENT_VERSION=1.15 # replace by your FRI client version
    mkdir -p lbr-stack/src && cd lbr-stack
    vcs import src --input https://raw.githubusercontent.com/lbr-stack/lbr_fri_ros2_stack/humble/lbr_fri_ros2_stack/repos-fri-${FRI_CLIENT_VERSION}.yaml

#. Install `Docker <https://docs.docker.com/engine/install/>`_:octicon:`link-external`.

#. Copy the Dockerfile and the container scripts to the ``lbr-stack`` directory. Build and start the container

   .. code-block:: bash

    cp -r src/lbr_fri_ros2_stack/docker/* .
    sudo ./container_build.sh # this will run the container once finished

#. Once inside the container, launch e.g. the mock setup via

   .. code-block:: bash

    ros2 launch lbr_bringup mock.launch.py model:=iiwa7

   .. hint::

    List all arguments for the launch file via

    .. code-block:: bash

        ros2 launch lbr_bringup mock.launch.py

   .. hint::

    Refer to :ref:`lbr_bringup` for more launch options.

#. Connect another shell to the running container

   .. code-block:: bash
    
    ./container_new_console.sh

#. Run e.g. MoveIt with RViz

   .. code-block:: bash
    
    ros2 launch lbr_bringup move_group.launch.py rviz:=true
