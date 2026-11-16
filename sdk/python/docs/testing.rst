How it is tested
================

What has been verified, how, and what has not. The commands are run from
``sdk/python`` in the repository.

Unit and integration tests
--------------------------

.. code-block:: console

   $ source /opt/ros/jazzy/setup.bash          # the ROS tests are skipped without it
   $ .venv/bin/python -m pytest tests -q

About a minute. They cover the bus, configuration, the three bundled robots,
every controller, the renderer and its passes, recording and both exports,
the Gymnasium wrapper, the command line, MAVLink vehicle and client over
real UDP, PX4-in-the-loop against a stand-in autopilot, the ROS 2 link
against ROS 2 Jazzy, and **every example and tutorial**: each app is started,
each script is run and its output checked, and the broken-app tutorial is
replayed step by step.

Battle tests
------------

.. code-block:: console

   $ .venv/bin/python tests/battle/harness.py            # all, about six minutes
   $ .venv/bin/python tests/battle/harness.py mavlink    # those whose name contains "mavlink"

38 scenarios that try to break it, each in its own process with a time
limit, so a hang or a crash in native code is caught as such.

.. list-table::
   :header-rows: 1
   :widths: 24 76

   * - Area
     - What is thrown at it
   * - Mistakes
     - Misspelt keys, types and arguments; broken YAML and model files;
       values out of range; parts that do not fit; controllers returning the
       wrong size, ``nan`` or nonsense; exceptions in user code; use after
       close. Every one must give a clear message, never a raw
       ``TypeError`` or ``KeyError``
   * - Endurance
     - Ten simulated minutes per robot with flat memory; 300 apps opened and
       closed; 2000 resets; bit-for-bit repeatability; real-time pacing at
       three speeds and after a stall; eight controller rates held exactly;
       a slow policy that crashes, stalls and returns nothing
   * - MAVLink
     - 3500 junk packets and 2000 random commands, then a normal flight;
       ``nan``, infinite and 5 km targets; every command out of order; two
       ground stations, one vanishing; a client with no vehicle; twenty
       restarts on the same port
   * - ROS 2
     - ``nan`` and absurd ``cmd_vel``; joint commands of the wrong size or
       for unknown joints; thirty restarts beside the user's own node; two
       namespaced robots in one graph
   * - Data and models
     - Full-HD and depth frames; rendering from a second thread; failing
       render passes; recording 3000 frames with flat memory; a crash
       mid-recording; a read-only folder; Gymnasium's ``check_env`` and 120
       random episodes; URDFs with and without motors, collision shapes and
       free bases; a 24-joint free-base robot; an unstable simulation
   * - Command line and threads
     - Thirteen kinds of mistake, each one line and a non-zero exit; a script
       that exits while an app runs in a background thread; 160 000 messages
       from eight threads

Running it found, among others: mistakes surfacing as raw Python errors or
not at all; a segfault at interpreter exit; a crash when rendering from two
threads; ``nan`` from ROS or MAVLink stopping the app; a MAVLink receiver
that went deaf for seconds after junk on its port; a velocity target flown
for ever; one failing bus subscriber starving the rest. All are fixed, and
the scenarios stay as regression tests.

A real PX4
----------

.. code-block:: console

   $ tests/px4/run.sh <px4 build dir> <work dir>        # pymavlink: takeoff, hover, land
   $ tests/px4/client.sh <px4 build dir> <work dir>     # the SDK's mission script: a square in OFFBOARD

Against PX4 v1.18 (beta), built from source: PX4 passes its own preflight
checks on the simulated sensors, takes off to 2.5 m, hovers within 1 cm
horizontally, lands and disarms; the SDK's client then flies it through a
square. These need a PX4 build, so they are not part of the automated
suite.

The IDE
-------

``tests/api/stacks.py`` creates a project for each of the 14 combinations
the New Project picker accepts, through the IDE's own API on an empty
RoboForge home, sets up the environment, reads the Stack view and runs each
one; the 5 combinations it refuses are checked too.

Not tested
----------

Stated plainly, so you know where the edges are:

- **No physical robot** has been driven. Simulation, and a real autopilot
  in simulation, only.
- **QGroundControl and MAVSDK** have not been connected. Plain pymavlink
  clients and PX4 have.
- **PX4-in-the-loop** was flown with the bundled quadrotor and PX4's generic
  quadrotor airframe only, on one PX4 version.
- **ROS 2 Jazzy** on Ubuntu 24.04 is the only ROS setup tried. No ROS 1.
- **The MuJoCo viewer window** and the IDE's desktop window were not checked
  by eye: every run here was headless.
- **Linux only.** Nothing was run on macOS or Windows.
- **OSMesa** (software rendering) was not available on the test machine;
  EGL and GLFW were used.
- Training on exported datasets (nerfstudio, Gaussian splatting) and SLAM
  systems were not run: the exports are checked for layout and for poses
  against the simulator's ground truth.

Writing your own tests
----------------------

.. code-block:: python

   import numpy as np
   import roboforge as rf

   def test_arm_reaches_its_target():
       checks = rf.Checks()
       app = rf.App(robot="arm", controllers=[rf.JointPD(rate=500, target=[0.5, 0.4, -0.8])],
                    callbacks=[checks], realtime=False)
       app.run(duration=4)
       q = app.bus.latest("state/joint").position
       app.close()
       assert np.abs(q - [0.5, 0.4, -0.8]).max() < 0.02
       assert not checks.findings

``realtime=False`` makes it fast, determinism makes it stable, and
``checks.findings`` turns "it looked wrong" into an assertion.
