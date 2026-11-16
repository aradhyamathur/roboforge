1. Your first app
=================

**You will:** run a robot arm from Python, write its controller, and read
what happened from the bus. **Time:** 5 minutes. **Needs:** MuJoCo.

The whole program
-----------------

.. literalinclude:: ../../examples/first_app.py
   :language: python
   :lines: 5-

.. code-block:: console

   $ python3 first_app.py
   joint names:  ['yaw', 'shoulder', 'elbow']
   target:       [ 0.5  0.4 -0.8]
   reached:      [ 0.5    0.413 -0.805]
   steps:        1500 (2 ms each)
   signals:      clock, cmd/actuators, sensor/imu, state/joint, state/pose, state/twist

What each part did
------------------

``rf.App(robot="arm", ...)``
   Loads the bundled arm and, because there is a robot, MuJoCo to simulate
   it. ``realtime=False`` runs as fast as the machine allows; leave it out and
   three simulated seconds take three real ones.

``@rf.controller(rate=500)``
   Turns a function into a controller called 500 times per simulated second.
   It receives ``obs``, the latest value of every signal: ``obs.joint`` is
   ``state/joint``, ``obs.pose`` is ``state/pose``, and ``obs["any/name"]``
   reads anything else.

``return rf.ActuatorCommand(...)``
   What a controller returns is published on the bus. An ``ActuatorCommand``
   goes to ``cmd/actuators``, which physics applies to the motors.

``app.bus.latest("state/joint")``
   After (or during) a run, the bus holds the latest value of every signal.

The shoulder stopped at 0.413, not 0.4: a PD loop without an integral term
cannot quite hold a joint against gravity. That small error is the cue for
the next step.

Use the built-in controller instead
-----------------------------------

Writing torques by hand is rarely what you want. Say *where* the joints
should be and let ``JointPD`` (a PID loop per joint, with an integral term)
make the torques:

.. code-block:: python

   @rf.controller(rate=50)
   def wave(obs, t):
       return rf.JointCommand(position=[0.8 * np.sin(t), 0.5, -0.9])

   app = rf.App(robot="arm", controllers=[wave, rf.JointPD(rate=500)])

Two controllers, two rates, one line each. ``wave`` publishes ``cmd/joint``;
``JointPD`` reads it and publishes ``cmd/actuators``. This layering, a
high-level controller above a low-level one, is how everything in the SDK is
built, from this to a learned policy (:doc:`pid_to_policy`).

The same app as a file
----------------------

Anything you can build in Python you can also write down:

.. code-block:: yaml

   roboforge: 1
   robot: arm
   controllers:
     - {type: "controller:Wave", rate: 50}      # class Wave in controller.py, next to this file
     - {type: joint_pd, rate: 500}

.. code-block:: console

   $ python3 -m roboforge info app.yaml        # what is in it, and who produces each signal
   $ python3 -m roboforge run app.yaml --tap   # run it, printing the bus once a second

``examples/mujoco_ros_arm`` is exactly this app, plus a ROS 2 link, which is
the next tutorial.

Try next
--------

- Change ``TARGET`` and the gains. Push ``40.0`` to ``400.0`` and watch
  ``reached`` overshoot.
- Print ``obs.joint.velocity`` inside the controller to see it change.
- Remove ``realtime=False`` and add ``callbacks=[rf.Viewer()]`` to watch.

If something goes wrong
-----------------------

``cmd/actuators has 2 values but arm has 3 actuators``
   The controller must return one value per motor.
   ``app.physics.actuator_names`` lists them.

Nothing moves and no error
   Run with ``--check`` (or add ``rf.Checks()`` to ``callbacks``): it says
   when nothing is publishing ``cmd/actuators``.
