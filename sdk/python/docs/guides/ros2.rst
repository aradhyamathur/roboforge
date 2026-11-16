The ROS 2 link
==============

``- {type: ros2}`` mirrors the bus on ROS 2 topics. It needs ``rclpy``, which
means ROS 2 sourced in the terminal and the system Python
(:doc:`../install`).

Topics
------

In the default ``mode: sim`` the app is the robot: state goes out, commands
come in.

.. list-table::
   :header-rows: 1
   :widths: 26 32 10 32

   * - Topic
     - Type
     - Dir.
     - Signal
   * - ``/clock``
     - ``rosgraph_msgs/Clock``
     - out
     - ``clock``, every step
   * - ``joint_states``
     - ``sensor_msgs/JointState``
     - out
     - ``state/joint``
   * - ``odom``, ``/tf``
     - ``nav_msgs/Odometry``, ``tf2_msgs/TFMessage``
     - out
     - ``state/pose``, ``state/twist`` (``odom`` → ``base_link``)
   * - ``imu``
     - ``sensor_msgs/Imu``
     - out
     - ``sensor/imu``
   * - ``camera/<name>/image_raw``
     - ``sensor_msgs/Image``
     - out
     - ``sensor/camera/<name>``
   * - ``cmd_vel``
     - ``geometry_msgs/Twist``
     - in
     - ``cmd/velocity``
   * - ``joint_command``
     - ``sensor_msgs/JointState``
     - in
     - ``cmd/joint``
   * - ``actuators``
     - ``std_msgs/Float64MultiArray``
     - in
     - ``cmd/actuators``
   * - ``setpoint_pose``
     - ``geometry_msgs/PoseStamped``
     - in
     - ``cmd/setpoint``

Messages are stamped in app time. ``/clock`` carries it, so start other
nodes with ``use_sim_time:=true``.

Options
-------

.. list-table::
   :header-rows: 1
   :widths: 22 78

   * - Option
     - Meaning
   * - ``mode``
     - ``sim`` (above) or ``robot``: the directions reverse, to drive a real
       or separately simulated robot from an app with ``physics: none``
   * - ``namespace``
     - Prefix for relative topics: ``namespace: r1`` gives ``/r1/cmd_vel``
   * - ``node_name``
     - Default ``roboforge``. Give each app its own when several share a graph
   * - ``topics``
     - Rename any: ``{cmd_vel: /robot/cmd_vel, joint: /joint_states}``
   * - ``profile``
     - ``gazebo`` or ``isaac``: the names those simulators' ROS bridges use,
       so a stack written against them needs no remapping
   * - ``enable``
     - Only these mappings: ``[odom, setpoint]``
   * - ``rate``
     - State publishing rate in Hz (``/clock`` is every step)
   * - ``tf``, ``frame_id``, ``child_frame_id``
     - Whether to publish ``/tf``, and the frame names

Two robots in one graph:

.. code-block:: yaml

   links:
     - {type: ros2, namespace: r1, node_name: sim1}

Bad input is dropped, not obeyed
--------------------------------

Whatever arrives from ROS is checked before it reaches a controller. A
message that cannot be right is dropped with one warning (repeated at most
every five seconds, with a count):

.. code-block:: text

   roboforge: ros2 link ignored a cmd_vel message: it contains nan or inf
   roboforge: ros2 link ignored a joint_command message: it has 7 positions but this robot has 3 joints (yaw, shoulder, elbow); send all of them in that order, or fill in `name`
   roboforge: ros2 link ignored a joint_command message: unknown joint wrist; this robot has yaw, shoulder, elbow

The app keeps running and keeps obeying good messages. ``link.dropped``
counts what was dropped, and ``--check`` mentions it.

A ``joint_command`` with ``name`` filled in may address any subset of the
joints in any order.

Your own topics
---------------

``link.node`` is the rclpy node. Add anything to it from a controller's or a
callback's ``setup``:

.. code-block:: python

   class Publisher(rf.Callback):
       def setup(self, app):
           ros = next(l for l in app.links if l.name == "ros2")
           self.pub = ros.node.create_publisher(String, "status", 10)

The link does not take over ROS: if your program has already called
``rclpy.init()``, the link uses that context and leaves it running when the
app closes.

Ctrl+C
------

The link starts ROS without rclpy's own signal handlers, so Ctrl+C and the
IDE's Stop reach the app, which closes its links in order.
