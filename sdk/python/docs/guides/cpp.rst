C++
===

There are three ways to bring C++ into a RoboForge app. Pick by where the
C++ code lives.

.. list-table::
   :header-rows: 1
   :widths: 30 35 35

   * - Your C++ is
     - Use
     - Data goes through
   * - A control law, filter, planner or model inside the app
     - a **C++ controller** (``type: cpp``), below
     - the bus, in the app's own process
   * - A ROS 2 node (``rclcpp``)
     - the **ROS 2 link**: the app publishes ``/joint_states``, ``/odom``,
       ``/imu``, ``/clock`` and listens on ``/joint_command``, ``/cmd_vel``,
       ``/setpoint_pose``
     - ROS 2 topics (:doc:`ros2`)
   * - A MAVSDK program or other MAVLink client
     - the **MAVLink link** in ``vehicle`` mode, which speaks the messages
       MAVSDK uses on ``udp://:14540``. MAVSDK itself has not been connected
       yet (:doc:`../testing`); plain MAVLink clients and PX4 have
     - MAVLink over UDP (:doc:`mavlink`)

C++ controllers
---------------

:doc:`../tutorials/cpp_controller` walks through one. The reference:

.. code-block:: yaml

   - type: cpp
     source: controller.cpp          # compiled for you; or library: build/libx.so
     rate: 500                       # Hz, as for any controller
     inputs: [cmd/joint, task/goal]  # extra signals the controller reads
     cxxflags: "-O3 -march=native"   # added to the compiler's flags (source: only)
     kp: 60                          # anything else: parameters for the constructor

``source:`` and ``library:`` are relative to ``app.yaml``. The library is
cached in ``build/`` next to the source under a name that includes a hash of
the source, the header, the compiler and the flags, so it is rebuilt exactly
when one of them changes.

The C++ side
------------

One header, ``roboforge/controller.hpp``: C++17, no dependencies.
``python3 -m roboforge cpp include`` prints its folder.

.. list-table::
   :header-rows: 1
   :widths: 34 66

   * - API
     - Meaning
   * - ``rf::Params``
     - ``number(name, default)``, ``numbers(name, default)``,
       ``text(name, default)``, ``has(name)``. A parameter the constructor
       never reads is reported as a likely typo
   * - ``setup(const rf::Info&)``
     - once: ``dt``, ``joint_names``, ``actuator_names``, ``ctrl_low``,
       ``ctrl_high``
   * - ``rf::Obs``
     - ``t()``; ``joints()``, ``q(i)``, ``qd(i)``, ``effort(i)``, ``q()``;
       ``has_pose()``, ``position()``, ``orientation()`` (w x y z),
       ``linear()``, ``angular()``, ``yaw()``; ``has_imu()``, ``accel()``,
       ``gyro()``; ``has_status()``, ``armed()``; ``has(name)``,
       ``signal(name)`` for ``inputs``
   * - ``rf::Action``
     - ``actuators(u)``, ``joints(q)``, ``velocity(forward, yaw_rate)`` or
       ``velocity(lin, ang)``, ``setpoint(x, y, z, yaw)``, ``none()``
   * - ``reset()``
     - on ``app.reset()``
   * - ``rf::PID``, ``rf::clamp``
     - the same PID as ``rf.PID``, and a clamp

Frames and units are the SDK's: world ENU, body FLU, quaternions (w, x, y,
z), SI units, radians.

How it runs
-----------

The library is loaded into the app's process; each step the observations
are copied into a plain C struct and the command is copied back. That
hand-over costs about ten microseconds; the C++ itself runs at native speed
(``controller.native_time`` and ``controller.calls`` measure it). Exceptions
are caught at the boundary and reported with the controller's name; a crash
in C++ (a bad pointer) ends the process, as it would in any native code.

Wrap a slow C++ controller in ``rf.Async`` like any other. Several instances
of one library each keep their own state.

Building it yourself
--------------------

.. code-block:: console

   $ python3 -m roboforge cpp build controller.cpp          # what source: does, printing the library's path
   $ g++ -std=c++17 -O2 -shared -fPIC -I"$(python3 -m roboforge cpp include)" controller.cpp -o libcontroller.so

With CMake, see the end of :doc:`../tutorials/cpp_controller`. The library
must export the functions ``RF_EXPORT_CONTROLLER`` defines; the interface is
versioned, and a library built against another version is refused with a
message to rebuild it.
