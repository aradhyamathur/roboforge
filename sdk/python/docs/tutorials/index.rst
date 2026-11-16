Tutorials
=========

Nineteen small tutorials, one idea each. Tutorials 14 to 17 build your own
policy, renderer, physics solver and SLAM method, and show how little code
each takes; 18 writes a controller in C++; 19 does imitation learning with
Hugging Face's LeRobot. They are toys on purpose: CPU only,
no GPU, about 150 MB of memory, seconds to a minute to run. On a desktop with
8 cores the simulator steps 17 to 30 times faster than real time
(8 to 13 times with a 320×240 camera), so a laptop is enough.
``examples/debugging/health_check.py`` measures yours.

The files
---------

Every tutorial runs files that ship with the SDK:

- with the IDE: ``~/.roboforge/sdk/python/examples/``
- in the repository: ``sdk/python/examples/``

Copy them somewhere you can edit:

.. code-block:: console

   $ cp -r ~/.roboforge/sdk/python/examples ~/rf-tutorials && cd ~/rf-tutorials

Run the commands in a RoboForge terminal (where ``python3`` already has the
SDK), or in the environment from :doc:`../install`. Tutorials marked *ROS 2*
need ``source /opt/ros/<distro>/setup.bash`` first. Add ``--headless`` to any
``run`` command to skip the viewer window; everything works without it.

.. list-table::
   :header-rows: 1
   :widths: 6 34 44 16

   * - #
     - Tutorial
     - Teaches
     - Needs
   * - 1
     - :doc:`first_app`
     - An app from ``app.yaml``; writing a controller; reading the bus
     -
   * - 2
     - :doc:`arm_ros`
     - The simulator on ROS 2 topics; commanding it from ROS
     - ROS 2
   * - 3
     - :doc:`wheeled_base`
     - ``cmd_vel``, odometry, command timeouts
     - ROS 2
   * - 4
     - :doc:`quad_mavlink`
     - The simulator as a MAVLink vehicle; a mission script
     -
   * - 5
     - :doc:`all_three`
     - MuJoCo, MAVLink and ROS 2 in one app
     - ROS 2
   * - 6
     - :doc:`bridge`
     - No simulator: MAVLink telemetry to ROS and back
     - ROS 2
   * - 7
     - :doc:`pid_to_policy`
     - A slow model above a fast PID loop, with a fallback
     -
   * - 8
     - :doc:`record_export`
     - Recording a run; exporting for NeRF, splatting and SLAM
     -
   * - 9
     - :doc:`px4`
     - A real PX4 autopilot flying the MuJoCo vehicle
     - PX4 build
   * - 10
     - :doc:`custom_robot`
     - Your own URDF or MJCF; floating bases
     -
   * - 11
     - :doc:`training`
     - Any app as a Gymnasium environment
     -
   * - 12
     - :doc:`render_pass`
     - Changing what the camera sees
     -
   * - 13
     - :doc:`debug_broken_app`
     - Finding five mistakes with the SDK's own tools
     -
   * - 14
     - :doc:`custom_policy`
     - Your own neural-network policy: train, save, deploy; PyTorch and stable-baselines3
     -
   * - 15
     - :doc:`custom_renderer`
     - Your own renderer: Gaussian splats in numpy, no OpenGL
     -
   * - 16
     - :doc:`custom_solver`
     - Your own physics solver, checked against MuJoCo
     -
   * - 17
     - :doc:`custom_slam`
     - Your own sensors and SLAM method, scored with ATE and RPE
     -
   * - 18
     - :doc:`cpp_controller`
     - A controller in C++, under a Python one; built for you or with CMake
     - C++ compiler
   * - 19
     - :doc:`lerobot`
     - Record demonstrations as a LeRobot dataset, train ACT, deploy it; hub policies; GPU
     - LeRobot

.. toctree::
   :hidden:
   :maxdepth: 1

   first_app
   arm_ros
   wheeled_base
   quad_mavlink
   all_three
   bridge
   pid_to_policy
   record_export
   px4
   custom_robot
   training
   render_pass
   debug_broken_app
   custom_policy
   custom_renderer
   custom_solver
   custom_slam
   cpp_controller
   lerobot
