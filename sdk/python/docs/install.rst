Install
=======

The SDK is plain Python (3.10 or newer). Its core needs only ``numpy`` and
``pyyaml``; each backend imports its own dependency the first time it is used,
so you install only what you run.

.. list-table::
   :header-rows: 1
   :widths: 22 30 48

   * - Part
     - Needs
     - How to get it
   * - MuJoCo physics and renderer
     - ``mujoco``
     - ``pip install mujoco``
   * - MAVLink link
     - ``pymavlink``
     - ``pip install pymavlink``
   * - ROS 2 link
     - ``rclpy`` and the standard message packages
     - Install ROS 2 and ``source /opt/ros/<distro>/setup.bash``
   * - Training environments
     - ``gymnasium``
     - ``pip install gymnasium``

With the RoboForge IDE
----------------------

The IDE carries the SDK inside it. Nothing to download:

#. Create a project with **New ▸ Build on the RoboForge SDK**.
#. Press **Set up SDK environment** when the Stack view asks for it. This makes
   one small Python environment, ``~/.roboforge/envs/sdk-venv``, with MuJoCo,
   pymavlink and Gymnasium on top of the system Python, without ``sudo``.
#. Every RoboForge terminal in that project now has ``python3 -m roboforge``.

The library itself is unpacked to ``~/.roboforge/sdk/python``, with these
tutorials' files in ``~/.roboforge/sdk/python/examples``.

From the repository
-------------------

.. code-block:: console

   $ cd sdk/python
   $ uv venv --python /usr/bin/python3 --system-site-packages .venv
   $ uv pip install --python .venv/bin/python -e ".[all]" pytest
   $ source .venv/bin/activate

Two details matter, and both are about ROS 2:

``--python /usr/bin/python3``
   ROS 2 installs ``rclpy`` for the **system** Python. An environment built on
   another interpreter (conda's ``base``, pyenv) cannot import it.

``--system-site-packages``
   Lets the environment see the ROS packages once you have sourced ROS.

``python3 -m venv`` works too if your system has ``python3-venv`` installed.
Without ROS 2 any Python 3.10+ environment will do.

Check the installation
----------------------

.. code-block:: console

   $ python3 -m roboforge doctor
   roboforge 0.1.0 on Python 3.12.3  (display: yes)
     ok   mujoco   MuJoCo physics and renderer  [4.0.1]
     ok   mavlink  MAVLink link (vehicle, client, hil)  [2.4.50]
     ok   ros2     ROS 2 link
     ok   gym      Gymnasium environment adapter  [1.4.0]

A line that says ``MISSING`` comes with the command that fixes it. ``ros2`` is
missing in any terminal where ROS has not been sourced; that only matters for
apps with a ROS 2 link.

To see whether the machine is fast enough, run
``examples/debugging/health_check.py`` (:ref:`debugging/toolbox:Is this machine fast enough?`).

Without a display
-----------------

Everything runs headless: over SSH, in a container, in CI. Rendering then
uses EGL, which the SDK selects by itself when there is no ``DISPLAY``. If
camera frames fail on such a machine see
:ref:`debugging/troubleshooting:Rendering and cameras`.
