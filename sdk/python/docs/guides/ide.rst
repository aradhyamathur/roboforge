With the RoboForge IDE
======================

The IDE creates, understands and launches SDK apps. None of it is required:
an app is a folder of text files and runs from any terminal.

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - In the IDE
     - What it does
   * - **New ▸ Build on the RoboForge SDK**
     - The stack picker: simulator × robot × autopilot link × ROS 2 ×
       controller language (Python or C++), with four quick picks.
       Combinations that cannot work are refused with the reason. It writes
       ``app/app.yaml``, a starter ``controller.py`` or ``controller.cpp`` (with
       a ``.clangd`` for VS Code), ``mission.py`` for a MAVLink vehicle, and
       run configurations
   * - **Stack**
     - The app's parts, each controller's rate, each link's signals, and who
       publishes every signal (from ``roboforge info --json``)
   * - **Set up SDK environment**
     - Creates ``~/.roboforge/envs/sdk-venv`` and points the project at it
   * - **▶ Run**
     - *App: run*, *App: show wiring*, *SDK: check environment*, and for a
       MAVLink vehicle *Mission: fly a square*
   * - **Open** on any folder
     - An ``app.yaml`` is recognised and offered in the Run menu; an MJCF
       model gets a "view in MuJoCo" entry
   * - Terminals (Ctrl+`)
     - ``python3`` already has the SDK; ROS is sourced if the project uses it

The SDK and these tutorials' files are unpacked from the IDE to
``~/.roboforge/sdk/python`` at start, so a fresh machine needs nothing else.

Stopping an app from the IDE sends it the same signal as Ctrl+C: links are
closed and a recording in progress is saved.
