4. A quadrotor flown over MAVLink
=================================

**You will:** make the simulator behave as a MAVLink vehicle and fly it with
a mission script, with no autopilot installed. **Time:** 10 minutes.
**Needs:** MuJoCo, pymavlink.

The app
-------

.. literalinclude:: ../../examples/mujoco_mavlink_quad/app.yaml
   :language: yaml
   :lines: 7-

``mode: vehicle`` makes the app answer MAVLink the way an autopilot does:
heartbeat, attitude, position, and the arm, takeoff, land and position-target
commands. Ground stations expect a vehicle on UDP 14550 and API clients on
14540, so it sends to both.

Fly it
------

.. code-block:: console

   $ cd mujoco_mavlink_quad
   $ python3 -m roboforge run app.yaml --headless      # terminal 1
   $ python3 mission.py                                # terminal 2
   waiting for the vehicle...
   armed: True
   takeoff: True
   at [1.78 0.   2.19]
   at [2.15 1.79 2.13]
   at [0.22 2.14 2.1 ]
   at [-0.04  0.27  2.08]
   landing: True
   landed and disarmed

The mission script
------------------

.. literalinclude:: ../../examples/mujoco_mavlink_quad/mission.py
   :language: python
   :lines: 6-

It is a second app with no physics (``type: none``) and a MAVLink link in
``client`` mode. Telemetry from the vehicle arrives as ``state/pose``; a
``Setpoint`` published on ``cmd/setpoint`` is streamed to the vehicle as
position targets. **The same script flies a real PX4** (:doc:`px4`): it is an
ordinary MAVLink client.

Try next
--------

- Change the corners; add a yaw: ``rf.Setpoint(position=..., yaw=1.57)``.
- Watch the traffic: ``python3 ../debugging/mavlink_sniffer.py`` *instead of*
  the mission (only one program can listen on a port).
- Uncomment the ``recorder`` line and export the flight (:doc:`record_export`).

If something goes wrong
-----------------------

``no vehicle heard on udp 14540. Is the app running?``
   Start the app first. If it is running, something else may be listening on
   14540; see :ref:`debugging/troubleshooting:MAVLink`.

``takeoff: False``
   The vehicle refuses takeoff while disarmed, and disarming in the air.
   ``armed:`` must print ``True`` first.

``mavlink could not open udpin:...: another program ... is already using that port``
   A previous mission script or a ground station still holds the port.
