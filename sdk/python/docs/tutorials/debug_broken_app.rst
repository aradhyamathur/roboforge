13. Debug a broken app
======================

**You will:** fix an app with five mistakes in it, using only what the SDK
tells you. **Time:** 10 minutes. **Needs:** MuJoCo.

This is the tutorial to come back to. The mistakes are the ones people
actually make, and the method works on your own apps: **run, read, fix one
thing, run again.**

The broken app
--------------

.. literalinclude:: ../../examples/debugging/broken_arm/app.yaml
   :language: yaml
   :lines: 7-

.. literalinclude:: ../../examples/debugging/broken_arm/controller.py
   :language: python
   :lines: 7-

Copy the folder and work in the copy:

.. code-block:: console

   $ cp -r debugging/broken_arm /tmp/broken && cd /tmp/broken

Every run uses the same command. ``--check`` turns on the watchdog that
reports problems *while* the app runs:

.. code-block:: console

   $ python3 -m roboforge run app.yaml --headless --duration 5 --check

Mistake 1: a misspelt section
-----------------------------

.. code-block:: text

   roboforge: error: unknown config keys: controlers. Did you mean 'controllers'? Allowed: callbacks, controllers, links, ...

The message names what was found, what was probably meant, and everything
that is allowed. Fix: ``controlers:`` → ``controllers:``.

Mistake 2: a misspelt argument
------------------------------

.. code-block:: text

   roboforge: error: controller 'controller:Reach' has no argument 'rte'. Did you mean 'rate'? Its arguments: rate, degrees.

This works for your own classes too: the arguments listed are those of
``Reach.__init__``. Fix: ``rte: 20`` → ``rate: 20``.

Mistake 3: a camera that does not exist
---------------------------------------

.. code-block:: text

   roboforge: error: the model has no camera named 'overveiw'. Its cameras: overview

Fix: ``overveiw`` → ``overview``. Up to here nothing has run: configuration
mistakes are caught before the first step.

Mistake 4: nothing drives the motors
------------------------------------

Now the app runs, for five seconds, and exits without an error. The arm has
not moved. This is the hard kind of bug, and what ``--check`` is for:

.. code-block:: text

   roboforge check: nothing has published cmd/actuators after 1.0s, so the 3 actuators get no input.
   Controllers in this app: Reach. A controller that returns joint targets, setpoints or velocities
   needs a low-level controller after it (joint_pd, multirotor, diffdrive).

``Reach`` says where the joints should go. Nothing turns that into torques.
Fix: add the low-level controller.

.. code-block:: yaml

   controllers:
     - {type: "controller:Reach", rate: 20}
     - {type: joint_pd, rate: 500}

To see it for yourself, run with ``--tap``: the table has a ``cmd/joint`` row
and no ``cmd/actuators`` row.

Mistake 5: degrees
------------------

.. code-block:: text

   roboforge check: the actuators have been at their limits 100% of the time. The gains are too high,
   or the target cannot be reached (check its units and frame).

The motors are flat out and the arm is jammed against its joint limits.
``--tap`` shows why:

.. code-block:: text

   cmd/actuators   500.0 Hz  JointPD   u [8.00 12.00 -8.00]
   cmd/joint        20.9 Hz  Reach     q* [40.00 35.00 -60.00]
   state/joint     500.0 Hz  mujoco    q [2.98 1.75 -2.46]

The torques ``u`` are exactly the motors' limits (8, 12 and 8 N·m), the
joints are stuck near their end stops, and the target is ``[40, 35, -60]``.
Those are degrees; the SDK works in radians, and 40 radians is more than six
turns. Fix: ``degrees: false`` on the
``Reach`` line (the controller then converts).

Fixed
-----

.. code-block:: console

   $ python3 -m roboforge run app.yaml --headless --duration 5 --check
   roboforge: running 'broken_arm' (robot: arm, links: none) for 5s
   roboforge: stopped at t=5.00s after 2500 steps

No complaints, and the joints are at 40.3°, 35.4° and −60.5°.
``app_fixed.yaml`` in the same folder is the result:

.. literalinclude:: ../../examples/debugging/broken_arm/app_fixed.yaml
   :language: yaml
   :lines: 3-

The method
----------

#. **Read the one-line error.** Configuration mistakes stop the app before it
   starts and say what is allowed.
#. **No error but wrong behaviour: add** ``--check``. It knows the usual
   causes.
#. **Still unclear: add** ``--tap`` and look for the signal that is missing,
   has no publisher, arrives at the wrong rate, or has an absurd value.
#. **Then step through it** (:ref:`debugging/toolbox:Step through a run`).

:doc:`../debugging/toolbox` covers each tool;
:doc:`../debugging/troubleshooting` lists symptoms and causes.
