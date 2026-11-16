9. A real PX4 flying the MuJoCo vehicle
=======================================

**You will:** run the real PX4 autopilot with MuJoCo as its simulator, and
fly it. **Time:** 20 minutes, half of it PX4 compiling. **Needs:** MuJoCo,
pymavlink, the PX4 source and a C++ toolchain. Gazebo is not needed.

How it fits together
--------------------

.. code-block:: text

   MuJoCo app  ── HIL_SENSOR, HIL_GPS ──►  PX4 SITL  ◄── MAVLink, UDP 14540 ──►  your client
   (mode: hil) ◄── HIL_ACTUATOR_CONTROLS ─  (the real autopilot)                 (mission.py, QGC, MAVSDK)
        TCP 4560, lockstep

PX4 is the controller; the app only simulates. The two run in lockstep: the
simulator advances only when PX4 has answered the last sensor sample, so the
result does not depend on how fast either one runs.

1. Build PX4 once
-----------------

.. code-block:: console

   $ cmake -S ~/PX4-Autopilot -B ~/px4-build -G Ninja -DCONFIG=px4_sitl_default
   $ ninja -C ~/px4-build px4

About ten minutes. Building into a separate folder leaves any ``build/``
inside the source tree alone.

.. warning::

   A PX4 binary built on another machine or in a container may not start
   (``GLIBC_2.4x not found``). Build it on the machine that runs it.

2. The app
----------

.. literalinclude:: ../../../../tests/px4/app.yaml
   :language: yaml

In the IDE: **New ▸ Build on the RoboForge SDK ▸ Autopilot link: PX4 in the
loop** generates the same thing.

3. Start both
-------------

.. code-block:: console

   $ python3 -m roboforge run app.yaml --headless          # terminal 1: waits for PX4 on TCP 4560
   $ mkdir -p ~/px4-run && cd ~/px4-run                    # terminal 2: an empty working folder
   $ PX4_SYS_AUTOSTART=10016 PX4_SIM_MODEL=none_iris ~/px4-build/bin/px4 -d ~/px4-build/etc -w . -s etc/init.d-posix/rcS

``none_iris`` is PX4's quadrotor airframe for "some other simulator". After
a few seconds of sensor data PX4 prints ``Ready for takeoff!``.

4. Fly
------

Anything that flies PX4 now flies the MuJoCo vehicle. The mission script from
tutorial 4, unchanged:

.. code-block:: console

   $ cd mujoco_mavlink_quad && python3 mission.py
   armed: True
   takeoff: True
   at [1.72 0.06 1.97]
   at [2.16 1.78 1.98]
   at [0.21 2.16 1.99]
   at [-0.22  0.16  1.99]
   landing: True
   landed and disarmed

``tests/px4/run.sh`` and ``tests/px4/client.sh`` in the repository do steps 3
and 4 in one command each.

What the link takes care of
---------------------------

Four things real PX4 needs that a naive simulator bridge gets wrong. Each
was found by running it:

- PX4 needs a **stream of sensor data to start its clock** before it sends
  its first motor command, so the simulator does not wait for one at first.
- PX4 **rejects readings that never change** as a stuck sensor, and will not
  fuse a noise-free magnetometer or barometer. A little noise is added
  (``sensor_noise``).
- PX4 **numbers a quadrotor's motors** front-right, rear-left, front-left,
  rear-right. The mapping to your model's rotors is worked out from their
  positions; ``motor_map`` overrides it.
- When PX4 exits, the app **stops with an error** instead of waiting on it.

If something goes wrong
-----------------------

``no autopilot connected on tcpin:0.0.0.0:4560 within 120s``
   PX4 did not start, or was started with a different simulator target.

PX4 says ``Preflight Fail: ...`` and refuses to arm
   Give it ten seconds after start. If it persists, another simulator may
   still be holding port 4560 from an earlier run, and PX4 attached to that
   one: ``ss -ltn | grep 4560``.

``the autopilot stopped answering ... Did PX4 exit?``
   PX4 exited or crashed; its own terminal says why.

The vehicle flips at takeoff
   Your model's rotor layout is not a quadrotor X, or its rotors spin the
   other way round (the app prints a warning). Set ``motor_map``.
