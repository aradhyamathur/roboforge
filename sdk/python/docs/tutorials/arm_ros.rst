2. An arm on ROS 2 topics
=========================

**You will:** put the simulated arm on ROS 2, watch it with the ``ros2``
tools, and command it from another node. **Time:** 10 minutes.
**Needs:** MuJoCo, ROS 2 (sourced).

The app
-------

.. literalinclude:: ../../examples/mujoco_ros_arm/app.yaml
   :language: yaml
   :lines: 7-

.. literalinclude:: ../../examples/mujoco_ros_arm/controller.py
   :language: python
   :lines: 6-

The only line about ROS is ``- {type: ros2}``.

Run it
------

.. code-block:: console

   $ cd mujoco_ros_arm
   $ python3 -m roboforge info app.yaml
   $ python3 -m roboforge run app.yaml --headless

In a second terminal (with ROS sourced):

.. code-block:: console

   $ ros2 topic list
   /actuators
   /camera/overview/image_raw
   /clock
   /cmd_vel
   /imu
   /joint_command
   /joint_states
   /odom
   /setpoint_pose
   /tf
   $ ros2 topic hz /clock
   average rate: 500.008
   $ ros2 topic echo --once /joint_states --field position
   array('d', [0.4656, 0.5156, -0.9002])

State goes out (``/clock``, ``/joint_states``, ``/odom``, ``/tf``, ``/imu``,
camera images); commands come in (``/joint_command``, ``/cmd_vel``,
``/actuators``, ``/setpoint_pose``). ``/clock`` carries simulation time, so
nodes started with ``use_sim_time:=true`` follow the simulator.

Command it from ROS
-------------------

Delete the ``controller:Wave`` line from ``app.yaml`` so nothing inside the
app is choosing targets, run it again, and publish them from outside:

.. code-block:: console

   $ ros2 topic pub -r 10 /joint_command sensor_msgs/msg/JointState "{position: [0.5, 0.2, -1.0]}"
   $ ros2 topic echo --once /joint_states --field position
   array('d', [0.5033, 0.2013, -1.0078])

Joints can also be addressed by name, any subset in any order; the others
keep their targets:

.. code-block:: console

   $ ros2 topic pub -r 10 /joint_command sensor_msgs/msg/JointState "{name: [elbow], position: [-0.4]}"

Try next
--------

- ``ros2 run rqt_image_view rqt_image_view /camera/overview/image_raw``
- Rename a topic: ``- {type: ros2, topics: {joint_command: /arm/command}}``
- Two robots in one graph: give each link a ``namespace`` (:doc:`../guides/ros2`).

If something goes wrong
-----------------------

``link 'ros2' needs a package that is not installed: No module named 'rclpy'``
   ROS is not sourced in the terminal that runs the app, or the Python in use
   is not the system one. See :ref:`debugging/troubleshooting:ROS 2`.

``ros2 topic list`` shows nothing
   The two terminals are on different ``ROS_DOMAIN_ID`` values, or
   ``ROS_LOCALHOST_ONLY`` differs between them.

``roboforge: ros2 link ignored a joint_command message: it has 7 positions but this robot has 3 joints``
   A command that does not fit is dropped and reported, never applied. Send
   one position per joint, or fill in ``name``.
