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
   * - C++ controllers
     - a C++17 compiler
     - ``sudo apt install g++`` (CMake only if you want it)

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

With pip and the requirements files
-----------------------------------

The repository's ``requirements.txt`` pins every Python package to the
version the SDK is tested with; ``requirements-ml.txt`` adds PyTorch and
stable-baselines3 for the PyTorch examples:

.. code-block:: console

   $ pip install -r requirements.txt
   $ pip install -e sdk/python
   $ uv pip install --torch-backend cpu -r requirements-ml.txt      # optional; the CPU build is enough

In Docker
---------

The repository's ``Dockerfile`` builds one image with everything: ROS 2
Jazzy, MuJoCo, pymavlink, Gymnasium, a C++ toolchain for C++ controllers,
the documentation tools, PyTorch (CPU) and stable-baselines3, LeRobot in a
second environment (``source /opt/roboforge/venv-lerobot/bin/activate``),
and the IDE.

.. code-block:: console

   $ docker build -t roboforge .                        # --build-arg WITH_ML=0 leaves out PyTorch
   $ docker run --rm -it --network host roboforge       # the IDE, at http://127.0.0.1:7878
   $ docker run --rm -it --network host roboforge bash  # a shell with ROS sourced and the SDK ready

``--network host`` because the IDE listens on 127.0.0.1 only, and so ROS 2
discovery and MAVLink's UDP reach the host as they would natively. Mount a
folder at ``/home/ubuntu/roboforge-projects`` to keep projects; files are
written as uid 1000. Rendering runs on the CPU (Mesa); for MuJoCo's viewer
window add ``-e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix``. The whole test
suite passes inside the image.

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

LeRobot
-------

LeRobot needs ``numpy < 2.3``, older than the version pinned here, so give
it an environment of its own: ``requirements-lerobot.txt`` and
:doc:`tutorials/lerobot` have the commands. The
SDK and its whole test suite work in both.

What uses a GPU
---------------

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Part
     - On a GPU?
   * - Physics (MuJoCo)
     - No: one world, stepped on the CPU
   * - Camera rendering
     - Yes, when there is one: MuJoCo renders through the GPU's EGL driver
       (0.4 ms per 640×480 frame on a GTX 1070), and through Mesa on the CPU
       otherwise
   * - Training and running policies (PyTorch, LeRobot)
     - Yes, when PyTorch can use the GPU: ``device="auto"`` picks it.
       PyTorch must be built for a CUDA version the driver supports
       (``uv pip install --torch-backend cu126 ...`` for a CUDA 12.6 driver)
   * - Your own code
     - Anything; ``rf.Policy`` and the C++ controllers convert GPU tensors
       for you

A GPU pays off for large models and camera inputs: ACT with a camera trained
8.5 times faster on a GTX 1070 than on an 8-core CPU. The small state-only
policies of the tutorials train as fast on a CPU.

Without a display
-----------------

Everything runs headless: over SSH, in a container, in CI. Rendering then
uses EGL, which the SDK selects by itself when there is no ``DISPLAY``. If
camera frames fail on such a machine see
:ref:`debugging/troubleshooting:Rendering and cameras`.
