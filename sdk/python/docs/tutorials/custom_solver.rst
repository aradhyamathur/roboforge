16. Write your own physics solver
=================================

**You will:** replace MuJoCo with a solver written from scratch, run the same
controllers on it, and check it against MuJoCo. **Time:** 10 minutes.
**Needs:** numpy (MuJoCo for the comparison).

What it takes
-------------

A class with ``dt``, ``time``, ``setup``, ``step`` and ``reset`` that
applies ``cmd/actuators`` and publishes the state. ``solver.py`` solves a
two-link arm's equations of motion with a Runge-Kutta step; its whole
contact with the SDK is this:

.. code-block:: python

   class TwoLinkArm(rf.Physics):
       def setup(self, app):
           self.bus = app.bus
           self.bus.subscribe("cmd/actuators", self._on_command)    # torques in
           self.reset()

       def step(self):
           ...                                                     # integrate one step
           self.time += self.dt
           self.publish_state(joint=rf.JointState(names=..., position=self.q, velocity=self.qd))

``publish_state`` publishes the clock and stamps every message with the
current time. Declaring ``joint_names``, ``actuator_names``, ``ctrl_range``
and ``joint_range`` buys size checks, saturation checks and Gymnasium action
spaces; declaring nothing more still works.

The physics
-----------

.. literalinclude:: ../../examples/custom_solver/solver.py
   :language: python
   :pyobject: TwoLinkArm.acceleration

Use it
------

.. literalinclude:: ../../examples/custom_solver/app.yaml
   :language: yaml
   :lines: 4-

No robot: the solver knows its own model. The controllers are the ones from
the URDF tutorial, unchanged.

.. code-block:: console

   $ cd custom_solver
   $ python3 -m roboforge run app.yaml --headless --duration 3 --tap --check
     state/joint      500.5 Hz  twolink    q [0.13 0.40]

Check it against MuJoCo
-----------------------

``compare.py`` runs the app twice, changing only the physics: once on this
solver, once on MuJoCo loading ``custom_urdf/twolink.urdf``, the same arm.

.. code-block:: console

   $ python3 compare.py
   solver                      speed   final joints (rad)
   numpy RK4           x13 real time   [0.366 0.537]
   MuJoCo              x29 real time   [0.366 0.524]

   over 500 samples: largest difference 0.0245 rad, mean 0.0046 rad
   agree

They agree to a few hundredths of a radian; the difference is the
integrators (RK4 here, Euler with implicit damping in MuJoCo). This is how to
validate a new backend: same app, same inputs, compare.

Other solvers
-------------

The same class wraps any simulator:

- **A simulator in its own process** (Gazebo, Isaac Sim, a game engine):
  ``step`` sends the commands, advances it one step and waits for the new
  state.
- **A learned world model:** ``step`` runs the model on the last state and
  command. Controllers, links and recordings work on its "imagined" rollouts
  as on real ones.
- **A real robot:** ``physics: none`` and a link, as in
  :doc:`bridge`.

Provide ``camera(name)`` too and any renderer, the splat one included, can
draw your solver's world.

If something goes wrong
-----------------------

``the two-link solver diverged``
   An explicit integrator with too large a timestep, or gains too stiff.
   ``rf.SimulationError`` stops the run cleanly.

Controllers complain about sizes
   ``joint_names`` and ``actuator_names`` must match what ``state/joint``
   carries and what ``cmd/actuators`` expects.
