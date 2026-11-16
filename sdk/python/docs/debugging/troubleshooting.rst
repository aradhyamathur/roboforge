Troubleshooting
===============

Organised by what you see. Each entry: the likely cause, how to confirm it,
the fix. If the SDK printed an error, look the message up in :doc:`errors`
instead.

Start here
----------

#. ``python3 -m roboforge doctor``: is everything this app needs installed
   *in this terminal*?
#. ``python3 -m roboforge info app.yaml``: is the app what you think it is?
#. Run with ``--check --tap``.

Nothing moves
-------------

.. list-table::
   :header-rows: 1
   :widths: 30 34 36

   * - Cause
     - Confirm
     - Fix
   * - No low-level controller
     - ``--check`` says nothing publishes ``cmd/actuators``; ``--tap`` has no
       such row
     - Add ``joint_pd``, ``multirotor`` or ``diffdrive`` after your controller
   * - The vehicle is disarmed
     - ``--check`` says so; ``state/status`` shows ``disarmed``
     - Arm it: ``link.arm()``, a MAVLink arm command, or
       ``rf.VehicleCommand("arm")``
   * - The controller returns ``None``
     - ``--tap``: its output signal has no row
     - Return a message. A value is ``None`` until first published: guard
       with ``if obs.joint is None: return``
   * - Controllers in the wrong order
     - The high-level one is listed after the low-level one
     - List high-level controllers first
   * - A command timed out
     - ``diffdrive`` stops 0.5 s after the last ``cmd/velocity``; a MAVLink
       velocity target lapses after 1 s
     - Keep publishing, or raise ``timeout`` / ``target_timeout``

It moves, but wrongly
---------------------

.. list-table::
   :header-rows: 1
   :widths: 30 34 36

   * - Symptom
     - Cause
     - Fix
   * - An arm slams into its limits
     - Targets in degrees. ``--check`` reports saturated actuators; ``--tap``
       shows targets like ``40.0``
     - Radians: ``np.radians(...)``
   * - A joint stops short of its target
     - Two links collide at the joint, or the motor is too weak for the load
     - Separate the shapes (:doc:`../tutorials/custom_robot`); raise the
       motor's limit
   * - A multirotor flips at takeoff
     - Rotor order or spin direction does not match the controller
     - For PX4 set ``motor_map``; for your model check each motor's ``gear``
   * - It flies sideways when told to go north
     - Frames. Inside the SDK the world is ENU (x east, y north); MAVLink is
       NED
     - Use ENU on the bus. The links convert
   * - It drifts or oscillates
     - Gains
     - Tune with :ref:`debugging/toolbox:Compare two runs`
   * - Two controllers fight
     - Both publish the same signal; ``--tap`` shows the source changing
     - Remove one, or make one consume the other's output

Errors when starting
--------------------

Configuration mistakes stop the app before its first step with a one-line
``roboforge: error: ...``. They are all in :doc:`errors`. The common ones:
a misspelt key or argument (the message suggests the right one), a file path
(relative paths are relative to the app file), a robot and controller that do
not fit together.

Too slow, or not real time
--------------------------

.. list-table::
   :header-rows: 1
   :widths: 30 34 36

   * - Cause
     - Confirm
     - Fix
   * - Camera too large or too fast
     - ``debugging/health_check.py``; ``--check`` reports x0.x of real time
     - Lower ``width``, ``height`` or ``fps``
   * - A slow controller in the loop
     - ``- {type: rate_monitor, every: 2}`` shows the rates dropping
     - Wrap it in ``rf.Async``
   * - Printing every step
     - ``--tap`` far faster than needed, or ``print`` in ``step``
     - Print less often
   * - Training runs at real time
     - ``realtime`` left on
     - ``realtime: false`` / ``--no-realtime`` (``as_env()`` does this itself)

The simulation blows up
-----------------------

``the simulation became unstable near t=...``: an actuator or contact is too
stiff for the timestep. In order of preference: use an implicit integrator
(``<option integrator="implicitfast"/>`` in the model), add ``armature`` and
damping to the joints, lower the gains, lower the timestep
(``physics.timestep=0.001``). The run stops at the first sign, so the
recording up to that moment is intact.

ROS 2
-----

.. list-table::
   :header-rows: 1
   :widths: 30 34 36

   * - Symptom
     - Cause
     - Fix
   * - ``No module named 'rclpy'``
     - ROS not sourced in this terminal, or Python is not the system one
       (conda ``base`` first on the path is the usual culprit)
     - ``source /opt/ros/<distro>/setup.bash``; ``which python3`` should be
       ``/usr/bin/python3`` or an environment built on it
       (:doc:`../install`)
   * - The app runs but ``ros2 topic list`` is empty
     - Different ``ROS_DOMAIN_ID`` or ``ROS_LOCALHOST_ONLY`` in the two
       terminals
     - Make them equal
   * - Other nodes' time stands still or jumps
     - They are on wall time while the app publishes ``/clock``
     - Start them with ``use_sim_time:=true``
   * - A command on a topic is ignored
     - The link dropped it (wrong size, ``nan``, unknown joint) and said so
       once
     - Read the warning; ``link.dropped`` counts them
   * - Two apps overwrite each other's topics
     - Same topic names
     - Give each link a ``namespace`` and a ``node_name``
   * - Topic names differ from what my stack expects
     -
     - ``topics: {...}`` or ``profile: gazebo`` / ``isaac``

MAVLink
-------

.. list-table::
   :header-rows: 1
   :widths: 30 34 36

   * - Symptom
     - Cause
     - Fix
   * - A client never sees the vehicle
     - Mismatched ends: both ``udpout``, both ``udpin``, or different ports
     - One ``udpout:host:port``, one ``udpin:0.0.0.0:port``. Confirm with
       the sniffer
   * - ``another program ... is already using that port``
     - An earlier client, a ground station, or a second copy of the app
     - Stop it: ``ss -lunp | grep 14540`` shows who
   * - Arm or takeoff returns ``False``
     - Refused: takeoff while disarmed, disarm in the air, takeoff altitude
       not positive
     - Arm first; land before disarming
   * - A command "sometimes" gets no answer
     - UDP loss, usually a client that stopped reading and overflowed its
       receive buffer
     - Keep reading while waiting; resend until acknowledged
       (:doc:`../guides/mavlink`)
   * - The vehicle stops after following velocity targets for a second
     - By design: a velocity target must be refreshed
     - Send it at 5 Hz or more, or use position targets
   * - PX4 will not arm: ``Preflight Fail``
     - Too early (give it 10 s), or PX4 is attached to a stale simulator
     - ``ss -ltn | grep 4560``; stop the old simulator
   * - PX4 ignores position targets
     - Not in OFFBOARD mode
     - The SDK's client requests it; your own client must, after targets
       are already flowing
   * - PX4 binary: ``GLIBC_2.4x not found``
     - Built on another system
     - Build PX4 on this machine (:doc:`../tutorials/px4`)

Rendering and cameras
---------------------

.. list-table::
   :header-rows: 1
   :widths: 30 34 36

   * - Symptom
     - Cause
     - Fix
   * - No ``sensor/camera/...`` signal
     - The app has no ``renderer``
     - Add one
   * - ``could not create an OpenGL context``
     - No display and no EGL (a container without GPU access, a minimal
       server)
     - ``MUJOCO_GL=egl`` with GPU drivers and ``libegl1``; or
       ``MUJOCO_GL=osmesa`` with ``libosmesa6`` for software rendering
   * - Frames are black
     - The camera looks at nothing, or the scene has no light
     - Check the camera in MuJoCo's viewer; add a ``<light>`` (a ground
       added for a URDF brings one)
   * - A URDF robot is invisible
     - MuJoCo dropped its ``<visual>`` shapes
     - ``<mujoco><compiler discardvisual="false"/></mujoco>`` in the URDF
   * - The viewer does not open
     - No display, or ``--headless``
     - Run on a desktop session; it is skipped, with a message, otherwise

Custom models
-------------

.. list-table::
   :header-rows: 1
   :widths: 30 34 36

   * - Symptom
     - Cause
     - Fix
   * - The robot falls through the floor
     - No ``<collision>`` shapes (the app warns at start)
     - Add them
   * - It falls for ever
     - No ground: an MJCF without a floor, or ``ground: false``
     - Add a plane, or ``ground: true``
   * - A humanoid hangs in the air
     - Its base is fixed to the world
     - ``floating_base: true`` and a ``position``
   * - ``MuJoCo could not load ...``
     - The model is malformed or a mesh is missing; MuJoCo's reason follows
     - Open it with ``python3 -m mujoco.viewer --mjcf=file`` to iterate

Recording and datasets
----------------------

.. list-table::
   :header-rows: 1
   :widths: 30 34 36

   * - Symptom
     - Cause
     - Fix
   * - ``... is not a recorded dataset``
     - The folder has no ``dataset.yaml``: the run never closed, or the path
       is wrong
     - Let the app close (Ctrl+C is fine; ``kill -9`` is not)
   * - No images in the recording
     - No renderer in the app
     - Add one
   * - An earlier recording disappeared
     - A folder holds one run
     - Record each run to its own ``dir``

Exiting and threads
-------------------

.. list-table::
   :header-rows: 1
   :widths: 30 34 36

   * - Symptom
     - Cause
     - Fix
   * - ``this app was closed; build a new App``
     - ``step()`` or ``run()`` after ``close()``
     - Use ``reset()`` to start over; build a new app after closing
   * - The process hangs at exit
     - A non-daemon thread of yours is still running an app
     - ``app.stop()`` from another thread ends ``run()`` after the current
       step
   * - Memory grows when creating many apps
     - It settles after the first hundred or so; it is not a leak
     - Call ``app.close()`` on each; reuse one app with ``reset()`` where
       you can

Still stuck
-----------

Reduce it: the smallest ``app.yaml`` that shows the problem, run with
``--check --tap``, plus the output of ``python3 -m roboforge doctor``. That
is what to bring to whoever helps, and making it usually finds the cause.
