The debugging toolbox
=====================

Seven tools, from quickest to most thorough. :doc:`../tutorials/debug_broken_app`
uses the first four on a real example.

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - When
     - Reach for
   * - Before anything else
     - ``python3 -m roboforge doctor``
   * - "What is actually in this app?"
     - ``python3 -m roboforge info app.yaml``
   * - It runs, but does the wrong thing
     - ``--check``
   * - "Is that signal even being published?"
     - ``--tap``
   * - A controller misbehaves
     - Step through the run by hand
   * - "Did my change help?"
     - Record two runs and compare
   * - A MAVLink peer "does not connect"
     - Sniff the port

Check the environment
---------------------

.. code-block:: console

   $ python3 -m roboforge doctor

Which optional parts import in *this* terminal, and the fix for each that
does not. Most "it worked yesterday" problems are a terminal where ROS is
not sourced or a different Python is first on the path.

See what is in an app
---------------------

.. code-block:: console

   $ python3 -m roboforge info app.yaml

The robot, the physics step, every controller with its rate, every link with
the signals it takes and gives, and every signal with its type. It runs the
app briefly with the links off, so it is safe to use anywhere. If a signal
you expect is not in the list, nothing publishes it.

Let the app tell you: ``--check``
---------------------------------

.. code-block:: console

   $ python3 -m roboforge run app.yaml --check

A watchdog that knows the usual reasons a robot does not do what was
expected and reports each once, in plain words:

.. list-table::
   :header-rows: 1
   :widths: 34 66

   * - It says
     - Meaning
   * - nothing has published ``cmd/actuators``
     - No controller drives the motors. A controller that returns joint
       targets, setpoints or velocities needs a low-level controller after it
   * - the actuators have been at their limits
     - Gains too high, or a target that cannot be reached: wrong units
       (degrees), wrong frame, or a collision in the way
   * - a setpoint is waiting but the vehicle is disarmed
     - Multirotor motors stay off until armed
   * - the app runs at x0.6 of real time
     - A step takes too long: camera too large or too fast, or a slow
       controller that should be wrapped in ``rf.Async``
   * - controller X is declared at 100 Hz but runs at 40 Hz
     - Same cause, seen from the controller
   * - controller X asks for 5000 Hz, above the physics rate
     - It runs once per step. Lower its rate or the physics timestep
   * - the robot is in free fall
     - No ground, or no collision geometry
   * - the link has dropped N incoming messages
     - Bad values, or messages for another robot; the link's own warnings
       say which

In Python: ``callbacks=[rf.Checks()]``; afterwards ``checks.findings`` holds
``(code, text)`` pairs, which makes it usable in tests.

Watch the bus: ``--tap``
------------------------

.. code-block:: console

   $ python3 -m roboforge run app.yaml --tap          # once a second
   $ python3 -m roboforge run app.yaml --tap 0.2      # five times a second

.. code-block:: text

   roboforge tap  t=2.00s
     clock            500.0 Hz  mujoco     t 2.004
     cmd/actuators    500.0 Hz  JointPD    u [-0.19 -0.68 0.20]
     cmd/joint         49.9 Hz  Wave       q* [0.73 0.50 -0.90]
     state/joint      500.0 Hz  mujoco     q [0.74 0.79 -0.92]

Every signal: how often it arrives, who publishes it, its latest value. Read
it like this:

- **A row is missing:** nothing publishes that signal.
- **The rate is wrong:** the publisher runs slower than you think.
- **The source is not who you expected:** two things publish the same
  signal and the latest wins.
- **The value is absurd:** units or frames.

In Python, ``rf.Tap(every=1.0, signals=["cmd/*"])`` narrows it, and
``tap.rows(app)`` returns the table as data.
``examples/debugging/signal_tap.py``:

.. literalinclude:: ../../examples/debugging/signal_tap.py
   :language: python
   :pyobject: main

Step through a run
------------------

``app.step()`` advances exactly one physics step, so you can own the loop:
print anything, stop on a condition, open the debugger.

.. literalinclude:: ../../examples/debugging/step_through.py
   :language: python
   :pyobject: main

.. code-block:: console

   $ python3 step_through.py
   model: 3 positions, 3 actuators (yaw, shoulder, elbow), dt 2 ms

        t  joint positions            error  torques
     0.20  [ 0.518  0.444 -0.776]     0.124  [ 0.26 -0.75 -0.34]
     0.40  [ 0.604  0.516 -0.909]     0.016  [ 0.03 -0.7   0.12]
     ...
   largest overshoot: 0.030 rad at t=0.74s
   settled to within 0.0203 rad after 2.0s

   replayed to t=0.742s: joints [ 0.618  0.525 -0.93 ], velocities [ 0. -0. -0.]

Three things make this work:

- ``realtime=False``: nothing waits for the wall clock. (In real-time mode a
  pause such as a breakpoint is also safe: the loop does not sprint to catch
  up afterwards.)
- **Runs are deterministic.** ``app.reset()`` and the same steps reproduce
  the same state exactly, so you can find the bad moment in one run and stop
  at it in the next.
- ``app.physics.raw`` is MuJoCo's own ``(model, data)``, for anything the bus
  does not carry: contact forces, actuator forces, constraint states.

``breakpoint()`` inside a controller's ``step`` works too.

Compare two runs
----------------

Record each variant, then load both and compute what you care about.

.. literalinclude:: ../../examples/debugging/compare_runs.py
   :language: python
   :lines: 14-

.. code-block:: console

   $ python3 compare_runs.py
   run       rms error    worst   (radians)
   soft         0.0638   0.1371
   stiff        0.0282   0.0588

   the stiff run repeated: identical, sample for sample

The last line is the important one: because a repeated run is identical, a
difference between two runs is caused by what you changed and nothing else.

Sniff a MAVLink port
--------------------

.. code-block:: console

   $ python3 debugging/mavlink_sniffer.py udpin:0.0.0.0:14540 5
   listening on udpin:0.0.0.0:14540 for 5 s ...

   from system/component: 1/1
   message                          rate
   ATTITUDE                       50.2 Hz
   LOCAL_POSITION_NED             50.0 Hz
   GLOBAL_POSITION_INT            50.0 Hz
   HEARTBEAT                       1.0 Hz
   SYS_STATUS                      1.0 Hz
   ...

   heartbeat: autopilot 0 (12 is PX4, 0 is generic), disarmed
   position (north, east, down): 0.00, -0.00, 0.00 m

or ``nothing arrived``, which answers the question: the vehicle is not
sending to that port. Stop the program that normally listens there first;
a UDP port has one listener.

For ROS 2 the equivalents already exist: ``ros2 topic list``, ``ros2 topic
hz``, ``ros2 topic echo --once``, ``ros2 node info /roboforge``.

Is this machine fast enough?
----------------------------

.. code-block:: console

   $ python3 debugging/health_check.py
   robot        physics only   + 320x240 camera at 30 fps
   quadrotor   x17 real time                 x8 real time
   arm         x30 real time                x13 real time
   diffdrive   x32 real time                x13 real time

   verdict: fine. The slowest case runs at x8, so real-time apps have room to spare.

In an app, ``- {type: rate_monitor, every: 2}`` prints the real-time factor
and each controller's measured rate while it runs.

In your own tests
-----------------

Everything above is usable from pytest: build the app with
``realtime=False``, run it, assert on ``app.bus.latest(...)``,
``checks.findings`` or a recorded ``Dataset``. The SDK's own tests
(:doc:`../testing`) are written this way.
