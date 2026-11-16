Python API
==========

Everything below is importable from ``roboforge`` (``import roboforge as
rf``). The arguments of each class are also the keys it accepts in an app
file.

App
---

.. autoclass:: roboforge.App
   :members: from_config, setup, step, run, stop, reset, close, publish, as_env, describe, controller_rate, time

Robot
-----

.. autoclass:: roboforge.Robot
   :members: from_zoo, from_mjcf, from_urdf, from_spec

Controllers
-----------

.. autoclass:: roboforge.Controller
.. autofunction:: roboforge.controller
.. autoclass:: roboforge.Obs
.. autoclass:: roboforge.PID
.. autoclass:: roboforge.control.joint.JointPD
.. autoclass:: roboforge.control.multirotor.MultirotorController
.. autoclass:: roboforge.control.diffdrive.DiffDriveController
.. autoclass:: roboforge.Async
.. autoclass:: roboforge.Chunk

Policies
--------

.. autoclass:: roboforge.Policy
   :members: observe, act, command, step, action_space, make_env

Physics
-------

.. autoclass:: roboforge.physics.mujoco.MujocoPhysics
   :members: allocation, raw
.. autoclass:: roboforge.physics.none.NoPhysics

Rendering
---------

.. autoclass:: roboforge.render.mujoco.MujocoRenderer
   :members: render, add_pass

Links
-----

.. autoclass:: roboforge.links.ros2.Ros2Link
   :members: raw
.. autoclass:: roboforge.links.mavlink.MavlinkLink
   :members: command, arm, disarm, takeoff, land, wait_connected

Callbacks
---------

.. autoclass:: roboforge.Recorder
.. autoclass:: roboforge.Tap
   :members: rows
.. autoclass:: roboforge.Checks
.. autoclass:: roboforge.RateMonitor
   :members: report
.. autoclass:: roboforge.Viewer
.. autofunction:: roboforge.summarize

Data
----

.. autoclass:: roboforge.Dataset
   :members: cameras, signals, times, trajectory, frames, export_nerfstudio, export_tum
.. autofunction:: roboforge.callbacks.as_row

Evaluation
----------

.. automodule:: roboforge.eval
   :members: ate, rpe, align, associate, write_tum, read_tum

Training
--------

.. autoclass:: roboforge.env.GymEnv
   :members: reset, step, close
.. autofunction:: roboforge.env.to_numpy
.. autofunction:: roboforge.env.box

The bus
-------

.. autoclass:: roboforge.Bus

Base classes
------------

.. autoclass:: roboforge.Physics
   :members: publish_state, camera
.. autoclass:: roboforge.Renderer
   :members: setup, render, finish, step, add_pass
.. autoclass:: roboforge.Link
.. autoclass:: roboforge.Callback

Errors
------

.. autoclass:: roboforge.ConfigError
.. autoclass:: roboforge.SimulationError

Messages
--------

.. automodule:: roboforge.msgs
   :members:
   :exclude-members: signal

Frames
------

.. automodule:: roboforge.frames
   :members:
