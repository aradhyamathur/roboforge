Error messages
==============

Every message the SDK can raise or print, what it means, and what to do.
Search this page for a few words of the message.

How errors are reported
-----------------------

.. list-table::
   :header-rows: 1
   :widths: 24 76

   * - Kind
     - When
   * - ``rf.ConfigError``
     - Something is wrong in an app file or in the arguments an app was built
       with. Raised before the first step. It is a ``ValueError``
   * - ``rf.SimulationError``
     - The physics could not go on. It is a ``RuntimeError``
   * - ``ValueError``
     - A controller produced something unusable (wrong size, ``nan``, not a
       message). Carries a note naming the controller
   * - ``ConnectionError``, ``TimeoutError``
     - A port is taken; a peer never connected or went away
   * - ``roboforge: ... ignored ...``
     - Printed, not raised: a link dropped a bad incoming message
   * - ``roboforge check: ...``
     - Printed by ``--check``: something looks wrong while running

On the command line the first four are printed as one line,
``roboforge: error: ...``, with exit code 2. An exception from your own code
keeps its full traceback, because the line number is what you need, with a
note such as ``(in controller Patrol at t=1.234s)`` appended.

The app file
------------

``unknown config keys: X. Did you mean 'Y'? Allowed: ...``
   A top-level key that does not exist. The allowed ones are listed.

``X is not valid YAML: ...``
   The file does not parse; the line and column follow.

``X: expected `key: value` settings at the top level, found a list``
   The file starts with list items (lines beginning with a dash). It must be
   a mapping.

``'links' must be a list (lines starting with '- ')``
   ``links``, ``controllers`` and ``callbacks`` are lists even with one
   entry.

``no such file: X``
   The app file itself was not found.

``override 'X' is not of the form path=value``
   For example ``physics.substeps=2``.

``override 'controllers.4.rate=3': controllers is a list of 2, so the next part must be an index from 0 to 1``
   The path goes through a list; use the entry's position.

Names and arguments
-------------------

``unknown controller 'X'. Did you mean 'Y'? Known: ... Use 'module:Class' for your own.``
   Also for ``physics backend``, ``renderer``, ``link`` and ``callback``. The
   known names are listed.

``controller 'X' has no argument 'A'. Did you mean 'B'? Its arguments: ...``
   A key under a component that its class does not accept. For your own
   class these are the arguments of its ``__init__``; a class with no
   ``__init__`` accepts ``rate`` and ``name``.

``controller 'mod:Cls': there is no module 'mod'. Put mod.py next to the config file``
   The module was not found next to the app file or on the Python path.

``controller 'mod:Cls': mod has no class 'Cls'. Did you mean 'Cla'? It defines: ...``
   The module was found; the class was not.

``link 'ros2' needs a package that is not installed: No module named 'rclpy'``
   An optional dependency is missing in this environment. ``python3 -m
   roboforge doctor`` gives the fix.

``a controller needs a `type```
   A list entry is a mapping without ``type``.

Values
------

``the rate of controller X must be a number greater than 0, got -5``
   Likewise for ``speed``, ``renderer width`` / ``height`` / ``fps``,
   ``physics timestep`` / ``substeps``, ``wheel_radius``, and the MAVLink
   link's rates and timeouts.

``mavlink mode must be vehicle, client or hil, got 'X'``

``mavlink url 'X' is not understood. Use udpout:host:port (send to), udpin:host:port (listen), ...``

``mavlink url 'X' needs a host and a port from 1 to 65535``

``robot position must be [x, y, z]``

The robot and its model
-----------------------

``no robot named 'X' in the zoo. Available: quadrotor, arm, diffdrive. For your own model use {mjcf: file.xml} or {urdf: file.urdf}.``

``robot model not found: X``
   Paths in an app file are relative to that file.

``robot has no setting 'X'. Its settings: ...``

``unknown robot profile 'X'. Choose one of: multirotor, arm, diffdrive, articulated``
   A humanoid is ``articulated``.

``MuJoCo could not load X: ...``
   MuJoCo's own reason follows: malformed XML, an unknown element, a mesh
   file that was not found.

``the model has no body named 'X'. Its bodies: ...``
   ``base`` must be one of them.

``the model has no camera named 'X'. Its cameras: ...``

``X has no collision geometry, so it will be invisible and pass through the ground.``
   Printed for a URDF with only ``<visual>`` shapes
   (:doc:`../tutorials/custom_robot`).

Parts that do not fit together
------------------------------

``the mujoco physics backend needs a robot (a model to load). Set `robot:`, or use `physics: none`.``

``the mujoco renderer needs the mujoco physics backend and a robot``

``the multirotor controller flies a multirotor, and arm is not one``
   It needs a model whose actuators are rotors attached to sites.

``the diffdrive controller drives two wheels (left, right), but X has 4 actuators``

``the joint_pd controller drives joints, and quadrotor has none``

``the joint_pd controller needs one actuator per joint, but X has 12 joints and 6 actuators``
   Set ``motors: true`` on the robot to add the missing ones.

``mavlink vehicle mode flies a multirotor, and arm is an arm robot``
   Use the quadrotor, give your model ``profile: multirotor``, or use
   ``mode: client``.

What a controller returned
--------------------------

Each of these carries ``(in controller X at t=...)``.

``X returned a str ('...'). Return a message (rf.JointCommand, rf.Setpoint, ...), a list of messages, a dict of {signal: message}, or None.``

``cmd/actuators has 2 values but arm has 3 actuators (yaw, shoulder, elbow)``
   One value per motor, in that order.

``cmd/actuators contains nan or inf: [...]. A controller upstream produced it``
   Usually a division by a zero ``dt`` on the first step, or a ``nan``
   already in the controller's input.

``the cmd/joint target has 7 positions but the robot has 3 joints (...)``
   Give every joint, or fill in ``names`` to address some of them.

``the cmd/joint target names unknown joints: wrist. This robot has ...``

``cmd/setpoint has nan or inf in its position``

``cmd/velocity contains nan or inf``

``no signal 'X' yet. Seen so far: ...``
   ``obs["X"]`` before anything published it. Use ``obs.get("X")`` and
   handle ``None``.

Policies and training
---------------------

``with your own `act`, also give `action_space```
   ``app.as_env(act=...)`` turns actions into something other than motor
   commands, so their range cannot be guessed. Pass
   ``action_space=roboforge.env.box(ranges)``, or use ``policy.make_env``.

``policy output must be one of joint, actuators, velocity, setpoint``

``policy X produced nan or inf``
   The network's output, refused before it reaches the robot.

``(policy X could not build its observation for 1000 calls in a row ...)``
   A note on the error from ``observe``: a signal it reads is never
   published, or ``observe`` has a bug.

``no signal 'X' in this dataset`` / ``'X' is not a pose signal``
   From ``Dataset.trajectory``.

``expected (N, 3) positions or (N, 7) poses`` / ``trajectories differ in length; use associate() first``
   From ``rf.eval``.

``the X physics backend has no cameras``
   A renderer asked ``physics.camera(name)`` of a backend that does not
   provide it.

Running
-------

``the simulation became unstable near t=0.004s (MuJoCo reported a nan or huge acceleration)``
   See :ref:`debugging/troubleshooting:The simulation blows up`.

``this app was closed; build a new App to run again (use reset() to start over without closing)``

``the renderer is not set up yet: call app.setup() (or app.step()) before render()``

``render pass 0 (name) returned a str; a pass returns an rf.Image, or None``

``could not create an OpenGL context for off-screen rendering``
   See :ref:`debugging/troubleshooting:Rendering and cameras`.

``the recorder cannot write to X: Permission denied``

Connections
-----------

``mavlink could not open udpin:0.0.0.0:14540: another program (or another copy of this app) is already using that port``

``no autopilot connected on tcpin:0.0.0.0:4560 within 120s. Start PX4 SITL so it connects to this simulator.``

``the autopilot stopped answering on tcpin:0.0.0.0:4560 (no motor command for 5s). Did PX4 exit?``

``this model's rotors spin the other way round from PX4's quadrotor X; yaw will be reversed``
   Printed in ``hil`` mode. Flip the sign of the yaw term in each motor's
   ``gear``, or set ``motor_map``.

Dropped input
-------------

Printed at most once every five seconds per kind, with a count of the rest.
The app keeps running.

``ros2 link ignored a cmd_vel message: it contains nan or inf``

``ros2 link ignored a joint_command message: it has 7 positions but this robot has 3 joints (...)``

``ros2 link ignored a joint_command message: unknown joint X; this robot has ...``

``ros2 link ignored a actuators message: it has 50 values but this robot has 3 actuators``

``mavlink link ignored a position target: it contains nan or inf``

``mavlink link ignored a position target: its type_mask ignores both position and velocity``
   Acceleration-only targets are not supported.

``async(X) raised RuntimeError: ... (error 1; the fallback applies)``
   A controller wrapped in ``rf.Async`` failed. The loop continues with the
   fallback; the exception is in ``.last_error``.

Datasets
--------

``X is not a recorded dataset (no dataset.yaml)``

``X has no camera frames: it was recorded without a renderer``

``no camera 'X' in this dataset. Recorded: front``

``no signal 'X' in this dataset. Recorded: ...``

Checks
------

The ``roboforge check: ...`` lines are explained one by one in
:ref:`debugging/toolbox:Let the app tell you: \`\`--check\`\``.
