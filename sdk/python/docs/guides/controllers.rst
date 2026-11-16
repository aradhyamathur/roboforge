Controllers
===========

A controller is anything with ``step(obs, t)`` that returns commands. The
same interface holds a ten-line PID loop and a multi-gigabyte learned model.

Three ways to write one
-----------------------

A function:

.. code-block:: python

   @rf.controller(rate=100)
   def hold(obs, t):
       return rf.JointCommand(position=[0.5, 0.4, -0.8])

A class, when it has state or settings:

.. code-block:: python

   class Patrol(rf.Controller):
       def __init__(self, rate=None, altitude=2.0):       # arguments come from app.yaml
           super().__init__(rate=rate)
           self.altitude = altitude

       def setup(self, app):                              # once, before the first step
           self.start = app.bus.latest("state/pose").position.copy()

       def step(self, obs, t):                            # at `rate` Hz of app time
           return rf.Setpoint(position=self.start + [0, 0, self.altitude])

       def reset(self): ...                               # on app.reset()
       def close(self): ...                               # on app.close()

.. code-block:: yaml

   controllers:
     - {type: "controller:Patrol", rate: 2, altitude: 3.0}

Or any object with a ``step`` method.

What ``step`` receives
----------------------

``obs`` is the latest value of every signal:

.. list-table::
   :widths: 34 66

   * - ``obs.joint``, ``obs.pose``, ``obs.twist``, ``obs.imu``
     - the standard state signals
   * - ``obs.status``, ``obs.setpoint``
     - vehicle status; the current ``cmd/setpoint``
   * - ``obs.camera("front")``
     - the latest frame of that camera
   * - ``obs["cmd/instruction"]``
     - any signal by name; raises a ``KeyError`` that lists what exists
   * - ``obs.get(name, default)``
     - the same, without raising
   * - ``"state/joint" in obs``
     - whether it has been published yet

A value is ``None`` until something has published it. ``t`` is app time in
seconds.

What ``step`` may return
------------------------

.. list-table::
   :widths: 34 66

   * - a message
     - published on that message's signal (``rf.JointCommand`` → ``cmd/joint``)
   * - a list of messages
     - each is published
   * - ``{signal: message}``
     - to publish on a signal of your own naming
   * - ``None``
     - nothing this time; lower layers keep their last target

Anything else is refused with an error naming the controller.

.. list-table::
   :header-rows: 1
   :widths: 40 22 38

   * - Message
     - Signal
     - Consumed by
   * - ``rf.ActuatorCommand(values=)``
     - ``cmd/actuators``
     - physics: one value per motor
   * - ``rf.JointCommand(position=, names=)``
     - ``cmd/joint``
     - ``joint_pd``
   * - ``rf.velocity_command(linear, angular)``
     - ``cmd/velocity``
     - ``diffdrive``
   * - ``rf.Setpoint(position=, velocity=, yaw=)``
     - ``cmd/setpoint``
     - ``multirotor``; the MAVLink client link
   * - ``rf.VehicleCommand("arm")``
     - ``cmd/vehicle``
     - the MAVLink links

Rates and order
---------------

Each controller runs at its own ``rate`` (Hz of app time); ``None`` means
every physics step. A rate above the physics rate runs once per step.
Controllers due in the same step run **in the order listed**, so a
high-level controller listed first has its output consumed by the low-level
one in the same step.

The built-in controllers
------------------------

.. list-table::
   :header-rows: 1
   :widths: 16 30 54

   * - Name
     - Turns
     - Notes
   * - ``joint_pd``
     - joint targets → torques
     - A PID loop per joint (``kp``, ``ki``, ``kd`` scalar or per joint).
       Holds the starting pose until a target arrives. Needs one motor per
       joint
   * - ``multirotor``
     - a position or velocity setpoint → motor commands
     - Cascaded position and attitude control. Reads the rotor layout from
       the model, so it flies any multirotor MuJoCo describes. Motors stay
       off while ``state/status`` says disarmed
   * - ``diffdrive``
     - forward speed and turn rate → wheel speeds
     - Wheel geometry from the robot's ``params``. Stops when commands stop
       (``timeout``)
   * - ``pid``
     - an error → a correction
     - ``rf.PID``: the building block, for use inside your own controller

Learned policies
----------------

``rf.Policy`` is a controller split into ``observe(obs) -> array`` and
``act(x) -> array``, the two halves a trainer needs, plus
``make_env(app, reward=...)`` for training against it.
:doc:`../tutorials/custom_policy` trains one and deploys it.

Slow controllers
----------------

Wrap anything slow in ``rf.Async``. It runs in a worker thread on a frozen
copy of the observations and never blocks the loop; see
:doc:`../tutorials/pid_to_policy`.

.. code-block:: python

   policy = rf.Async(MyModel(), rate=50, timeout=1.0, fallback="hold")

``fallback`` is what to do when the model is late: ``"hold"`` (repeat the
last action), a message, a function ``(obs, t) -> action``, or ``None``.

When a controller fails
-----------------------

An exception raised in ``step`` stops the run and carries a note naming the
controller and the time: ``(in controller Patrol at t=1.234s)``. The app is
still closed properly, so a recording up to that point is saved.
