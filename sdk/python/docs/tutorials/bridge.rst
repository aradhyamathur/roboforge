6. ROS 2 and MAVLink with no simulator
======================================

**You will:** bridge an autopilot to ROS 2: telemetry in, setpoints out.
**Time:** 10 minutes. **Needs:** pymavlink, ROS 2 (and MuJoCo for the
stand-in autopilot).

The app
-------

.. literalinclude:: ../../examples/ros_mavlink_bridge/app.yaml
   :language: yaml
   :lines: 7-

``physics: none`` means there is nothing to simulate: state arrives through
the MAVLink link and time is the wall clock. This is the app you run next to
a real drone, or next to an autopilot simulated elsewhere.

Run it
------

With no autopilot at hand, the vehicle from tutorial 4 stands in for one:

.. code-block:: console

   $ (cd mujoco_mavlink_quad && python3 -m roboforge run app.yaml --headless)    # terminal 1: the "autopilot"
   $ (cd ros_mavlink_bridge && python3 -m roboforge run app.yaml)                # terminal 2: the bridge
   $ ros2 topic echo --once /odom --field pose.pose.position                     # terminal 3

``/odom`` and ``/tf`` now carry the vehicle's pose, received over MAVLink.
A pose published on ``/setpoint_pose`` goes the other way, as a MAVLink
position target.

With a real autopilot
---------------------

Point ``url`` at it:

.. code-block:: yaml

   - {type: mavlink, mode: client, url: "udpin:0.0.0.0:14540"}      # PX4 SITL sends here by default
   # - {type: mavlink, mode: client, url: "/dev/ttyUSB0"}           # a telemetry radio

With PX4 on the other end the link asks for OFFBOARD mode once setpoints are
flowing, which PX4 requires before it follows them.

If something goes wrong
-----------------------

``/odom`` never appears
   Nothing is arriving over MAVLink. Stop the bridge and run
   ``python3 debugging/mavlink_sniffer.py udpin:0.0.0.0:14540``: it prints
   what arrives on that port, or says that nothing does.
