The app file
============

An ``app.yaml`` describes an app completely. It is flat on purpose: no
inheritance, no includes, no variable interpolation. What you read is what
runs.

.. code-block:: yaml

   roboforge: 1                       # file format version
   name: my_app

   robot: quadrotor                   # a bundled robot, or {urdf: ..} / {mjcf: ..}
   physics: {type: mujoco}
   renderer: {type: mujoco, width: 320, height: 240, fps: 10}

   links:
     - {type: mavlink, mode: vehicle}
     - {type: ros2}

   controllers:
     - {type: "controller:Patrol", rate: 2}     # class Patrol in controller.py next to this file
     - {type: multirotor, rate: 250}

   callbacks:
     - {type: recorder, dir: runs/latest}

   realtime: true
   speed: 1.0
   run: {duration: 30}

Two rules cover everything under a component:

- ``type`` is a registered name (``mujoco``, ``ros2``, ``joint_pd``…) or
  ``module:Class`` for your own code, looked up next to the file first.
- **Every other key is an argument of that class.** The arguments of each
  built-in are in :doc:`../reference/api`; a wrong one is reported with the
  list of right ones.

A component with no arguments can be written as just its name:
``physics: mujoco``.

Top-level keys
--------------

.. list-table::
   :header-rows: 1
   :widths: 18 82

   * - Key
     - Meaning
   * - ``roboforge``
     - Format version, ``1``
   * - ``name``
     - Shown in logs and in the IDE
   * - ``robot``
     - A zoo name, or a mapping (:doc:`robots`). Leave out for apps with no
       simulator
   * - ``physics``
     - ``mujoco`` (default when there is a robot) or ``none``
   * - ``renderer``
     - Optional. Without it there are no camera signals
   * - ``links``
     - A list. Order does not matter
   * - ``controllers``
     - A list, **run in this order** each step: put high-level controllers
       before the low-level ones that consume their output
   * - ``callbacks``
     - A list: ``recorder``, ``viewer``, ``rate_monitor``, ``tap``, ``checks``
   * - ``realtime``
     - ``true``: keep pace with the wall clock. ``false``: as fast as possible
   * - ``speed``
     - Real-time factor when ``realtime`` is true (``2.0`` is twice as fast)
   * - ``run``
     - ``{duration: seconds}``: how long ``roboforge run`` runs by default

Overrides
---------

Any value can be changed from the command line without editing the file:

.. code-block:: console

   $ python3 -m roboforge run app.yaml physics.timestep=0.001 --headless
   $ python3 -m roboforge run app.yaml controllers.0.rate=5 renderer.fps=30
   $ python3 -m roboforge run app.yaml "links.0.urls=[udpout:10.0.0.7:14550]"

A path is keys and list indexes joined by dots; the value is YAML. A
component written as a bare name is expanded as needed, so
``physics.substeps=2`` works on ``physics: mujoco``.

See what will run
-----------------

.. code-block:: console

   $ python3 -m roboforge info app.yaml
   my_app: robot quadrotor (multirotor), physics mujoco (dt 0.0020s)
     renderer   mujoco  cameras: front
     controller Patrol  2 Hz
     controller MultirotorController  250 Hz
     link       mavlink (vehicle)  in: state/pose, state/twist, cmd/vehicle  out: state/status, cmd/setpoint
     link       ros2 (sim)  in: clock, state/joint, ...  out: cmd/velocity, cmd/joint, ...
     signals    clock [Clock], cmd/actuators [ActuatorCommand], ...

``info`` runs the app for a moment with its links switched off, so it opens
no ports and needs no ROS. ``--json`` gives the same as data; the IDE's
Stack view is drawn from it.

From Python
-----------

.. code-block:: python

   app = rf.App.from_config("app.yaml")
   app = rf.App.from_config("app.yaml", ["physics.timestep=0.001"])           # with overrides
   app = rf.App.from_config("app.yaml", controllers=[MyController()])          # replace a whole section
   app = rf.App.from_config({"robot": "arm", "controllers": ["joint_pd"]})     # a dict works too

Mistakes
--------

Every mistake in an app file stops the app before its first step, as a
``ConfigError`` with a message that says what was found, what is allowed and
what was probably meant. :doc:`../debugging/errors` lists them.
