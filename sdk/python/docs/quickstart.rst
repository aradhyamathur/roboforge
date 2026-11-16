Quickstart
==========

Five minutes, three steps. You need MuJoCo (``python3 -m roboforge doctor``
shows whether you have it).

1. Run a robot
--------------

.. code-block:: console

   $ python3 -m roboforge zoo
     quadrotor    multirotor   X-layout quadrotor, 1 kg, with an IMU and a forward camera.
     arm          arm          Three-joint torque-controlled arm on a fixed base, with an overview camera.
     diffdrive    diffdrive    Differential-drive base with a caster, IMU and forward camera.

Save this as ``app.yaml``:

.. code-block:: yaml

   roboforge: 1
   robot: arm
   controllers:
     - {type: joint_pd, rate: 500, target: [0.5, 0.4, -0.8]}

and run it. ``--tap`` prints what is happening once a second:

.. code-block:: console

   $ python3 -m roboforge run app.yaml --headless --duration 3 --tap
   roboforge: running 'app' (robot: arm, links: none) for 3s
   roboforge tap  t=1.00s
     clock            501.0 Hz  mujoco      t 1.002
     cmd/actuators    500.0 Hz  JointPD     u [-0.00 -0.53 0.20]
     sensor/imu       501.0 Hz  mujoco      acc [0.00 0.00 9.81]
     state/joint      501.0 Hz  mujoco      q [0.51 0.42 -0.83]
     state/pose       501.0 Hz  mujoco      pos [0.00 0.00 0.10]
     state/twist      501.0 Hz  mujoco      lin [0.00 0.00 0.00] ang [0.00 0.00 -0.00]
   ...
   roboforge: stopped at t=3.00s after 1500 steps

The arm moved to the target: ``state/joint`` reads ``[0.51 0.42 -0.83]``. Drop
``--headless`` to watch it in MuJoCo's viewer.

2. Write the controller yourself
--------------------------------

Next to ``app.yaml``, save ``controller.py``:

.. code-block:: python

   import numpy as np
   import roboforge as rf

   class Wave(rf.Controller):
       def step(self, obs, t):                       # called 50 times a second
           return rf.JointCommand(position=[0.8 * np.sin(t), 0.5, -0.9])

and point the app at it. Your controller says *where* the joints should be;
the built-in one after it turns that into motor torques:

.. code-block:: yaml

   roboforge: 1
   robot: arm
   controllers:
     - {type: "controller:Wave", rate: 50}
     - {type: joint_pd, rate: 500}

Run it again. ``cmd/joint`` now appears in the table, published by ``Wave``.

3. Connect it to something
--------------------------

Add two lines and the same app is on ROS 2:

.. code-block:: yaml

   links:
     - {type: ros2}

.. code-block:: console

   $ ros2 topic echo --once /joint_states

Nothing else changed, and nothing else will when you swap the robot, add a
camera, or put a MAVLink autopilot in front of it. That is the idea:
:doc:`concepts` explains why it works, and the
:doc:`tutorials <tutorials/index>` go through each combination.
