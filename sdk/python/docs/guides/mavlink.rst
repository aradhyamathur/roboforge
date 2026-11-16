The MAVLink link
================

``- {type: mavlink, mode: ...}`` speaks MAVLink through pymavlink. One link,
three roles.

.. list-table::
   :header-rows: 1
   :widths: 12 22 66

   * - Mode
     - The app is
     - Use it to
   * - ``vehicle``
     - the vehicle
     - fly the simulated multirotor from a mission script, MAVSDK or a
       ground station, with no autopilot installed
   * - ``client``
     - a MAVLink client
     - talk to an existing autopilot: a real drone, PX4 SITL, or an app in
       ``vehicle`` mode
   * - ``hil``
     - PX4's simulator
     - put the real PX4 autopilot in the loop (:doc:`../tutorials/px4`)

Addresses
---------

.. list-table::
   :header-rows: 1
   :widths: 34 66

   * - ``url``
     - Meaning
   * - ``udpout:127.0.0.1:14540``
     - send to that address (and receive the replies)
   * - ``udpin:0.0.0.0:14540``
     - listen on that port
   * - ``tcp:host:port``, ``tcpin:0.0.0.0:4560``
     - TCP client, TCP server
   * - ``/dev/ttyUSB0``
     - a serial device

``urls`` takes a list. Defaults: ``vehicle`` sends to 14550 (ground station)
and 14540 (API); ``client`` listens on 14540; ``hil`` listens on TCP 4560.
The two ends must be a matching pair: one ``udpout``, one ``udpin``, same
port.

Vehicle mode
------------

Sends ``HEARTBEAT``, ``ATTITUDE``, ``LOCAL_POSITION_NED``,
``GLOBAL_POSITION_INT``, ``GPS_RAW_INT``, ``SYS_STATUS``,
``EXTENDED_SYS_STATE`` and ``HOME_POSITION``. Handles arm and disarm,
``NAV_TAKEOFF``, ``NAV_LAND`` and ``SET_POSITION_TARGET_LOCAL_NED`` (position
or velocity, with yaw). It needs a multirotor and the ``multirotor``
controller, which flies the setpoints the link publishes.

It behaves like an autopilot where it matters:

- takeoff is refused while disarmed, and a takeoff altitude must be positive
- disarming is refused in the air (unless forced)
- landing ends with an automatic disarm
- a **velocity target must keep coming**: if it is not refreshed for
  ``target_timeout`` (1 s), the vehicle stops and holds position
- targets containing ``nan`` or ``inf`` are dropped with a warning
- speed is limited by the ``multirotor`` controller's ``max_speed``

Options: ``home`` (latitude, longitude, altitude of the local origin),
``takeoff_altitude``, ``land_speed``, ``rate`` (telemetry, Hz), ``system_id``.

Client mode
-----------

Telemetry becomes ``state/pose``, ``state/twist`` and ``state/status``. A
``Setpoint`` on ``cmd/setpoint`` is streamed as position targets.

.. code-block:: python

   mav = rf.MavlinkLink(mode="client", url="udpin:0.0.0.0:14540")
   app = rf.App(physics={"type": "none", "rate": 50}, links=[mav])
   app.setup()
   mav.wait_connected(30)             # True once a heartbeat has arrived
   mav.arm(); mav.takeoff(2.0)        # each returns whether the autopilot accepted
   app.bus.publish("cmd/setpoint", rf.Setpoint(position=[2, 0, 2]))
   app.run(duration=10)
   mav.land()

Commands are **resent until acknowledged** (UDP loses packets), for up to
three seconds. With PX4 on the other end the link also requests OFFBOARD
mode once setpoints are flowing and converts the takeoff height to PX4's
above-sea-level convention.

``examples/mujoco_mavlink_quad/mission.py`` is a complete client.

PX4 in the loop
---------------

``mode: hil`` is covered in :doc:`../tutorials/px4`. Its options:
``sensor_rate`` (250 Hz), ``sensor_noise`` (1.0; 0 turns it off),
``motor_map``, ``lockstep_timeout``, ``connect_timeout``.

Robustness
----------

The link is meant to face the open network of a lab:

- Bytes that are not MAVLink are skipped at once. A burst of junk on the port
  does not make the link deaf (each UDP datagram is parsed on its own).
- Opening a ``udpin`` or ``tcpin`` port that another program holds is an
  error at start, not a silent second listener.
- Several clients can command one vehicle; a client that disappears does not
  disturb the others.

.. warning::

   A UDP client that stops reading for a second or two overflows its receive
   buffer, and the packets dropped then may include the acknowledgement it is
   waiting for. If you write your own client: keep reading while you wait,
   and resend commands until acknowledged. The SDK's client does both.

Seeing the traffic
------------------

``examples/debugging/mavlink_sniffer.py`` listens on a port and prints which
messages arrive, how often and from whom
(:ref:`debugging/toolbox:Sniff a MAVLink port`).

``link.raw`` is the list of pymavlink connections, for anything else.
