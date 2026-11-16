Concepts
========

The whole design is one idea, a few parts, and some conventions.

The idea: a signal bus instead of bridges
-----------------------------------------

Usually every pairing needs its own glue: a bridge from this simulator to
ROS, another from that simulator to PX4, a third from ROS to MAVLink. With
*N* simulators and *M* transports that is *N × M* bridges, and most of them do
not exist.

Here nothing talks to anything else directly. Every part publishes and reads
named, typed **signals** on one in-process **bus**:

.. code-block:: text

      physics ──► state/pose, state/joint, sensor/imu, clock ──┐
                                                               ▼
   controllers ◄── reads the latest values ────────────── the bus ◄──► links (ROS 2, MAVLink)
        │                                                      ▲
        └──► cmd/joint, cmd/setpoint, cmd/actuators ───────────┘

A simulator is one adapter. A transport is one adapter. Any of them combine,
because each only knows the bus. That is why adding ``links: [{type: ros2}]``
to an app is the whole of "ROS support" for that app.

The parts
---------

.. list-table::
   :header-rows: 1
   :widths: 16 44 40

   * - Part
     - What it does
     - Provided
   * - ``Robot``
     - A model file plus what kind of machine it is
     - ``quadrotor``, ``arm``, ``diffdrive``; your own MJCF or URDF
   * - ``Physics``
     - Steps the world, publishes state, applies ``cmd/actuators``
     - ``mujoco``; ``none`` (a real robot, or state arriving through a link)
   * - ``Renderer``
     - Turns the world and a camera into images
     - ``mujoco`` (colour, depth, your own passes)
   * - ``Link``
     - Connects the bus to the outside
     - ``ros2``; ``mavlink`` in ``vehicle``, ``client`` and ``hil`` modes
   * - ``Controller``
     - Your code: ``step(obs, t)`` returns commands
     - ``joint_pd``, ``multirotor``, ``diffdrive``, ``pid``; ``Async`` for slow models
   * - ``Callback``
     - Hooks around the loop
     - ``recorder``, ``viewer``, ``rate_monitor``, ``tap``, ``checks``
   * - ``App``
     - Owns the clock and the loop. Optional
     - ``run()``, ``step()``, ``reset()``, ``as_env()``

One step of the loop
--------------------

``app.step()`` does this, in this order, every physics step (2 ms for the
bundled robots):

#. Links read what arrived from outside (``before_step``). A link may block
   here: that is how lockstep with PX4 works.
#. Each controller whose turn has come runs, and what it returns is published.
#. Physics advances one step and publishes the new state.
#. The renderer publishes camera frames that are due.
#. Links send (``after_step``); callbacks run.
#. In real-time mode the loop sleeps until the wall clock catches up.

Every controller has **its own rate**. A 500 Hz PID loop, a 50 Hz planner and
a 2 Hz learned model live in one app, each called when due.

Three levels of control
-----------------------

Start at the top; drop down only when you need to.

**1. Configuration only.** An ``app.yaml`` and the command line. No Python.
See :doc:`guides/config`.

**2. Your own classes.** Subclass ``rf.Controller`` (or ``Callback``,
``Renderer``, ``Link``, ``Physics``) and name it in the config as
``module:Class``. See :doc:`guides/controllers` and :doc:`guides/extending`.

**3. Everything exposed.** Build the app in Python, write the loop yourself
with ``app.step()``, and reach the native objects: ``app.physics.raw``
(MuJoCo's ``MjModel`` and ``MjData``), ``link.raw`` (the rclpy node, the
pymavlink connection). Nothing is hidden behind the abstraction.

Conventions
-----------

These hold everywhere inside the SDK. Links convert at the edge.

.. list-table::
   :widths: 24 76

   * - World frame
     - ENU: x east, y north, z up (as in ROS and MuJoCo)
   * - Body frame
     - FLU: x forward, y left, z up
   * - Quaternions
     - ``(w, x, y, z)``, as in MuJoCo. ROS uses ``(x, y, z, w)``; the ROS link converts
   * - Units
     - SI: metres, seconds, **radians**, newtons, newton-metres
   * - MAVLink
     - NED world and FRD body on the wire; the MAVLink link converts both ways
   * - Time
     - ``t`` is app time: simulation time with physics, wall time with ``physics: none``

.. warning::

   Angles are radians. Joint targets written in degrees are the most common
   reason an arm slams into its limits; the :doc:`debugging tutorial
   <tutorials/debug_broken_app>` shows what that looks like.

Standard signals
----------------

A signal is just a name and a message type. These names are the contract
between parts; the full list with every field is in :doc:`reference/signals`.

.. list-table::
   :header-rows: 1
   :widths: 26 22 52

   * - Signal
     - Message
     - Meaning
   * - ``clock``
     - ``Clock``
     - app time
   * - ``state/joint``
     - ``JointState``
     - joint positions, velocities, efforts
   * - ``state/pose``, ``state/twist``
     - ``Pose``, ``Twist``
     - the base in the world; linear velocity in the world, angular in the body
   * - ``state/status``
     - ``VehicleStatus``
     - armed, mode, landed (vehicles)
   * - ``sensor/imu``
     - ``Imu``
     - specific force and angular rate in the body frame
   * - ``sensor/camera/<name>``
     - ``Image``
     - a colour frame with its camera pose and field of view
   * - ``cmd/actuators``
     - ``ActuatorCommand``
     - one value per actuator: what physics applies
   * - ``cmd/joint``
     - ``JointCommand``
     - joint targets, for a joint controller
   * - ``cmd/velocity``
     - ``VelocityCommand``
     - body-frame velocity (ROS ``cmd_vel``)
   * - ``cmd/setpoint``
     - ``Setpoint``
     - a world position or velocity, with yaw (multirotors)
   * - ``cmd/vehicle``
     - ``VehicleCommand``
     - arm, disarm, takeoff, land

Commands flow **down**: a high-level controller publishes ``cmd/setpoint``,
the multirotor controller turns it into ``cmd/actuators``, physics applies
it. Swapping the top layer (a joystick, a mission script, a learned policy)
changes nothing below it.

What it does not do
-------------------

Honest limits, so you do not find them the hard way:

- One physics backend so far (MuJoCo). Gazebo and Isaac backends are planned,
  not built. The interfaces are designed for them, and
  :doc:`guides/extending` shows what a backend must provide.
- Simulation runs on the CPU, one world at a time. There is no GPU-parallel
  training.
- No lidar model yet; cameras and an IMU are the sensors.
- PX4-in-the-loop has been flown with the bundled quadrotor and PX4's generic
  quadrotor airframe only.

:doc:`testing` lists exactly what has been verified and how.
