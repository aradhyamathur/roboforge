Robots
======

A robot is a model file plus the few facts the rest of the stack needs.

.. code-block:: yaml

   robot: quadrotor                        # from the zoo

   robot:                                  # your own
     urdf: my_robot.urdf                   # or mjcf: my_robot.xml; paths are relative to app.yaml
     profile: articulated
     base: torso
     cameras: [head_cam]
     floating_base: true
     position: [0, 0, 1.0]
     params: {wheel_radius: 0.05}

.. list-table::
   :header-rows: 1
   :widths: 20 80

   * - Setting
     - Meaning
   * - ``zoo`` / ``urdf`` / ``mjcf``
     - Where the model comes from. Exactly one
   * - ``profile``
     - ``multirotor``, ``arm``, ``diffdrive``, or ``articulated`` (anything
       with joints, humanoids included). Tells links and the IDE what kind of
       machine it is
   * - ``base``
     - The body whose pose is ``state/pose``. Default: the first body
   * - ``cameras``
     - Cameras the renderer publishes by default
   * - ``motors``
     - Add a torque motor per joint. Default: when the model has joints and
       no actuators (every URDF)
   * - ``ground``
     - Add a ground plane and a light. Default: for URDFs
   * - ``floating_base``
     - Give the base a free joint, for robots that are not bolted down
   * - ``position``
     - Where the base starts, ``[x, y, z]``
   * - ``params``
     - Numbers a controller needs that the model does not state

The bundled robots
------------------

.. list-table::
   :header-rows: 1
   :widths: 14 14 72

   * - Name
     - Profile
     - What it is
   * - ``quadrotor``
     - multirotor
     - X-layout quadrotor, 1 kg, four rotors of 6.5 N, an IMU and a forward
       camera ``front``
   * - ``arm``
     - arm
     - Three joints (``yaw``, ``shoulder``, ``elbow``) with torque motors of
       8, 12 and 8 N·m, on a fixed base, with a camera ``overview``
   * - ``diffdrive``
     - diffdrive
     - Two driven wheels and a caster, an IMU, a camera ``front``; wheel
       radius 0.05 m, separation 0.30 m

``python3 -m roboforge zoo`` lists them. The model files are in
``roboforge/zoo/`` and are short enough to read and copy.

Your own model
--------------

:doc:`../tutorials/custom_robot` walks through a URDF. In brief:

- **MJCF** is loaded as written. For a multirotor, attach each rotor's motor
  to a ``<site>`` with the thrust and reaction torque in its ``gear``; the
  ``multirotor`` controller and the PX4 link read the layout from there.
- **URDF** gets motors and a ground plane added. Keep the visuals with
  ``<mujoco><compiler discardvisual="false"/></mujoco>``.

In Python:

.. code-block:: python

   robot = rf.Robot.from_urdf("humanoid.urdf", floating_base=True, position=[0, 0, 1.0])
   robot = rf.Robot.from_mjcf("my_quad.xml", profile="multirotor", base="frame", cameras=["down"])
   app = rf.App(robot=robot, ...)

Sensors
-------

Physics publishes ``sensor/imu`` from the model's accelerometer and gyro
sensors if it has them; otherwise it derives one from the base's motion, so
there is always an IMU. Cameras are the model's ``<camera>`` elements,
rendered by the renderer.
