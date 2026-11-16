Signals and messages
====================

A signal is a name on the bus; a message is the value it carries. Messages
are plain dataclasses with numpy arrays. All of them are in ``roboforge``
(``rf.Pose``…) and in ``roboforge.msgs``.

State
-----

.. list-table::
   :header-rows: 1
   :widths: 22 18 60

   * - Signal
     - Message
     - Fields
   * - ``clock``
     - ``Clock``
     - ``t``: app time, seconds
   * - ``state/joint``
     - ``JointState``
     - ``names``; ``position`` (rad or m), ``velocity``, ``effort`` (N·m or
       N), one entry per hinge or slide joint; ``t``
   * - ``state/pose``
     - ``Pose``
     - ``position`` (3, world ENU, m); ``orientation`` (4, quaternion w x y
       z, body to world); ``t``
   * - ``state/twist``
     - ``Twist``
     - ``linear`` (3, world frame, m/s); ``angular`` (3, **body** frame,
       rad/s); ``t``
   * - ``state/status``
     - ``VehicleStatus``
     - ``armed``; ``mode`` (``HOLD``, ``TAKEOFF``, ``OFFBOARD``, ``LAND``);
       ``landed``

Sensors
-------

.. list-table::
   :header-rows: 1
   :widths: 22 18 60

   * - Signal
     - Message
     - Fields
   * - ``sensor/imu``
     - ``Imu``
     - ``accel`` (3, body FLU, m/s², specific force: reads +9.81 on z at
       rest); ``gyro`` (3, body, rad/s); ``orientation``; ``t``
   * - ``sensor/camera/<name>``
     - ``Image``
     - ``data`` (H×W×3 ``uint8``); ``camera``; ``encoding`` (``rgb8``);
       ``fovy`` (degrees); ``pose`` (4×4 camera to world, OpenGL axes);
       ``t``; ``intrinsics`` (``w``, ``h``, ``fl_x``, ``fl_y``, ``cx``,
       ``cy``); ``extras``
   * - ``sensor/depth/<name>``
     - ``Image``
     - ``data`` (H×W ``float32``, metres); ``encoding`` ``depth32f``

Commands
--------

.. list-table::
   :header-rows: 1
   :widths: 22 18 60

   * - Signal
     - Message
     - Fields
   * - ``cmd/actuators``
     - ``ActuatorCommand``
     - ``values``: one per actuator, in ``physics.actuator_names`` order,
       clipped to each actuator's range
   * - ``cmd/joint``
     - ``JointCommand``
     - ``position`` (and optionally ``velocity``, ``effort``); ``names`` to
       address a subset
   * - ``cmd/velocity``
     - ``VelocityCommand``
     - ``linear`` (3) and ``angular`` (3), both in the **body** frame. Make
       one with ``rf.velocity_command(linear, angular)``
   * - ``cmd/setpoint``
     - ``Setpoint``
     - ``position`` (3, world) *or* ``velocity`` (3, world); ``yaw`` (rad,
       0 faces east, counter-clockwise positive)
   * - ``cmd/vehicle``
     - ``VehicleCommand``
     - ``name``: ``arm``, ``disarm``, ``takeoff``, ``land``; ``params``
       (``{"altitude": 2.0}``)
   * - ``cmd/instruction``
     - ``Instruction``
     - ``text``: a natural-language goal, for policies that take one

Your own signals
----------------

Any name works: ``bus.publish("planner/path", my_object)`` or return
``{"planner/path": my_object}`` from a controller. Read it with
``obs["planner/path"]``. A message class with a ``signal`` attribute can be
returned from a controller directly.

Frames at a glance
------------------

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Where
     - Convention
   * - On the bus
     - World ENU, body FLU, quaternion (w, x, y, z)
   * - ROS 2 topics
     - The same frames; quaternion (x, y, z, w)
   * - MAVLink
     - World NED, body FRD; yaw 0 faces north, clockwise positive
   * - Camera ``pose``
     - OpenGL: x right, y up, looks along −z. The TUM export converts to
       z-forward

``roboforge.frames`` has the conversions (``enu_to_ned``, ``flu_to_frd``,
``quat_to_mat``, ``yaw_of``…).
