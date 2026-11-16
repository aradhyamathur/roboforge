10. Your own robot
==================

**You will:** load a URDF, drive its joints, and let its base move freely as
a humanoid's would. **Time:** 10 minutes. **Needs:** MuJoCo.

The app
-------

.. literalinclude:: ../../examples/custom_urdf/app.yaml
   :language: yaml
   :lines: 8-

.. literalinclude:: ../../examples/custom_urdf/controller.py
   :language: python

.. code-block:: console

   $ cd custom_urdf
   $ python3 -m roboforge info app.yaml
   custom_urdf: robot twolink (arm), physics mujoco (dt 0.0020s)
     controller Swing  50 Hz
     controller JointPD  500 Hz
   $ python3 -m roboforge run app.yaml --headless --duration 4 --tap 2 --check
     cmd/joint         50.4 Hz  Swing     q* [0.41 0.70]
     state/joint      500.5 Hz  mujoco    q [0.49 0.79]

What a URDF lacks, and what is added
------------------------------------

A URDF describes links and joints. To simulate it three more things are
needed, and the SDK supplies each unless told not to:

.. list-table::
   :header-rows: 1
   :widths: 22 46 32

   * - Missing
     - What is added
     - Setting
   * - Motors
     - A torque motor per joint, limited to the URDF's ``<limit effort=...>``,
       when the model has joints and no actuators
     - ``motors: true`` / ``false``
   * - Ground
     - A ground plane and a light, for URDFs
     - ``ground: true`` / ``false``
   * - A free base
     - Nothing by default: the root link is fixed to the world
     - ``floating_base: true``, ``position: [x, y, z]``

An MJCF file that already has actuators and a floor is loaded exactly as
written.

Humanoids and legged robots
---------------------------

.. code-block:: yaml

   robot:
     urdf: my_humanoid.urdf
     profile: articulated
     floating_base: true
     position: [0, 0, 1.0]

``state/pose`` is then the pose of the base link, ``state/joint`` has every
joint, and one ``joint_pd`` controller drives them all. A ``JointCommand``
with ``names`` may address any subset.

Three things that catch everyone
--------------------------------

**The robot is invisible.** MuJoCo ignores a URDF's ``<visual>`` shapes
unless the URDF says otherwise. Add this inside ``<robot>``:

.. code-block:: xml

   <mujoco><compiler discardvisual="false"/></mujoco>

**The robot falls through the floor.** No ``<collision>`` shapes. The app
says so at start, and ``--check`` reports the free fall.

**A joint stops short of its target.** Two links overlap at the joint and
collide. The example's shoulder sits 5 cm above the base box for this
reason. ``--check`` reports actuators stuck at their limits.

Meshes
------

Mesh paths in a URDF must be findable from the URDF's folder. For
``package://`` paths, add a mesh directory:

.. code-block:: xml

   <mujoco><compiler meshdir="../meshes" strippath="true" discardvisual="false"/></mujoco>

If something goes wrong
-----------------------

``MuJoCo could not load .../robot.urdf: ... link1.stl``
   A collision mesh was not found; see Meshes above.

``the joint_pd controller needs one actuator per joint, but ... has 12 joints and 6 actuators``
   The model has motors on only some joints. Set ``motors: true`` to add the
   rest, or write a controller for the motors it has.

``the model has no body named 'torso'. Its bodies: ...``
   ``base`` must be one of the listed bodies.
