RoboForge SDK
=============

A small Python library in which the simulator, the renderer, ROS 2 and MAVLink
are interchangeable parts. You write the controller; the same code runs
against MuJoCo, behind ROS topics, as a MAVLink vehicle, under a real PX4
autopilot, or as a training environment.

.. code-block:: python

   import roboforge as rf

   app = rf.App(robot="quadrotor",
                controllers=[rf.MultirotorController(rate=250, hover=[0, 0, 1.5])],
                links=[rf.MavlinkLink(mode="vehicle")])      # MAVLink clients can now fly it
   app.run(duration=10)

Or with no code at all, from a file:

.. code-block:: console

   $ python3 -m roboforge run app.yaml

**New here?** :doc:`install`, then :doc:`quickstart` (five minutes), then the
:doc:`tutorials <tutorials/index>` in order.

**Something does not work?** Go straight to :doc:`debugging/troubleshooting`:
it is organised by what you see, and every error message the SDK can print is
explained in :doc:`debugging/errors`.

.. toctree::
   :maxdepth: 2
   :caption: Start

   install
   quickstart
   concepts

.. toctree::
   :maxdepth: 2
   :caption: Tutorials

   tutorials/index

.. toctree::
   :maxdepth: 2
   :caption: Guides

   guides/config
   guides/controllers
   guides/robots
   guides/ros2
   guides/mavlink
   guides/rendering
   guides/cpp
   guides/extending
   guides/ide

.. toctree::
   :maxdepth: 2
   :caption: Help and debugging

   debugging/toolbox
   debugging/troubleshooting
   debugging/errors
   debugging/faq

.. toctree::
   :maxdepth: 2
   :caption: Reference

   reference/cli
   reference/signals
   reference/api
   testing
