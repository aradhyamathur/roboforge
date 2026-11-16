3. A wheeled base and ``cmd_vel``
=================================

**You will:** drive a differential-drive base with velocity commands and read
its odometry. **Time:** 10 minutes. **Needs:** MuJoCo, ROS 2.

The app
-------

.. literalinclude:: ../../examples/diffdrive_ros/app.yaml
   :language: yaml
   :lines: 7-

.. literalinclude:: ../../examples/diffdrive_ros/controller.py
   :language: python
   :lines: 7-

``Circle`` publishes a velocity command (forward speed and turn rate, in the
robot's own frame). The built-in ``diffdrive`` controller converts it into
left and right wheel speeds using the robot's wheel radius and separation.

Run it
------

.. code-block:: console

   $ cd diffdrive_ros
   $ python3 -m roboforge run app.yaml --headless --tap

The ``state/pose`` row of the table moves around a circle. From ROS:

.. code-block:: console

   $ ros2 topic echo --once /odom --field pose.pose.position

Drive it yourself
-----------------

Delete the ``controller:Circle`` line and run again. Five seconds at
0.2 m/s moves the base one metre:

.. code-block:: console

   $ ros2 topic pub -r 10 -t 50 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
   $ ros2 topic echo --once /odom --field pose.pose.position
   x: 1.0351
   y: -1.77e-10
   z: 0.0697

Or with the keyboard:

.. code-block:: console

   $ ros2 run teleop_twist_keyboard teleop_twist_keyboard

When the commands stop, so does the robot: the ``diffdrive`` controller
treats a command older than half a second as gone (``timeout: 0.5``). A
runaway robot because a teleop node crashed is not possible.

Try next
--------

- Give the app to Nav2 or slam_toolbox: it publishes ``/odom``, ``/tf`` and
  ``/clock`` like any simulated base.
- Use the topic names another simulator's bridge uses:
  ``- {type: ros2, profile: gazebo}``.

If something goes wrong
-----------------------

The base does not move when ``cmd_vel`` is published
   Check the table from ``--tap``: is ``cmd/velocity`` there, and who
   publishes it? If ``Circle`` is still in the app it publishes too, and the
   latest command wins.

``the diffdrive controller drives two wheels (left, right), but ... has 4 actuators``
   This controller is for two-wheel robots. For another drive, write the
   mixing yourself (:doc:`../guides/controllers`).
