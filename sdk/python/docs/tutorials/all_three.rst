5. MuJoCo, MAVLink and ROS 2 together
=====================================

**You will:** fly the quadrotor over MAVLink while ROS 2 watches. **Time:**
5 minutes. **Needs:** MuJoCo, pymavlink, ROS 2.

This is tutorial 4 with one more link:

.. literalinclude:: ../../examples/mujoco_ros_mavlink_quad/app.yaml
   :language: yaml
   :lines: 7-

.. code-block:: console

   $ cd mujoco_ros_mavlink_quad
   $ python3 -m roboforge run app.yaml --headless                 # terminal 1
   $ python3 mission.py                                           # terminal 2
   $ ros2 topic echo /odom --field pose.pose.position             # terminal 3

The position in terminal 3 traces the square that the mission in terminal 2
commands over MAVLink. ``ros2 topic list`` shows ``/clock``, ``/odom``,
``/imu``, ``/tf`` and ``/camera/front/image_raw``.

Why this is one line
--------------------

The MAVLink link and the ROS link do not know about each other. Both read
``state/pose`` from the bus; MAVLink turns it into ``LOCAL_POSITION_NED``
(north-east-down), ROS into ``nav_msgs/Odometry`` (east-north-up). A third
transport would be a third adapter, not three more bridges.

``python3 -m roboforge info app.yaml`` shows the wiring: which signals each
link takes from the bus and which it puts on it.

Try next
--------

- Send the vehicle a goal from ROS instead of MAVLink:

  .. code-block:: console

     $ ros2 topic pub -r 5 /setpoint_pose geometry_msgs/msg/PoseStamped "{pose: {position: {x: 2.0, y: 0.0, z: 2.0}}}"

  (arm and take off over MAVLink first: the motors stay off while disarmed.)
