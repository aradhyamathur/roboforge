Questions
=========

Do I need ROS 2?
   No. ROS 2 is one link among several. Apps without a ``ros2`` link never
   import it.

Do I need a GPU?
   No. Physics runs on the CPU. Camera rendering uses the GPU when there is
   one and Mesa otherwise. Training and running neural policies use the GPU
   when PyTorch can (``device="auto"``); see :ref:`install:What uses a GPU`.

Do I need the IDE?
   No. An app is a folder of text files; ``python3 -m roboforge run
   app.yaml`` runs it anywhere.

Can the same controller run on the real robot?
   That is the intent of ``physics: none``: state then arrives through a
   link (ROS topics in ``mode: robot``, or a MAVLink autopilot) and the
   controller cannot tell the difference. What has been verified is the
   MAVLink side against a real PX4 in simulation; no physical robot has been
   driven with it yet.

How fast does it run?
   On an 8-core desktop: ×17 real time for the quadrotor, ×30 for the arm,
   about half that with a 320×240 camera at 30 fps. Measure yours with
   ``examples/debugging/health_check.py``.

Is a run reproducible?
   Yes, exactly. The same app and the same inputs give the same states bit
   for bit, also after ``reset()`` and in a new process. Three things break
   it, by nature: real-time links (packets arrive when they arrive),
   ``rf.Async`` (a thread's timing), and your own use of unseeded random
   numbers.

Why radians, ENU, and (w, x, y, z)?
   They are what MuJoCo uses, and ENU with a forward-left-up body is what
   ROS uses. MAVLink's NED is converted at the link. One convention inside
   means one place to get it right.

Can several robots run in one app?
   Not yet: an app has one robot. Run one app per robot; with ROS 2 give
   each link a ``namespace``.

Can I use Gazebo or Isaac Sim instead of MuJoCo?
   Not yet. The ``Physics`` interface is what a backend for them would
   implement (:doc:`../guides/extending`); none is bundled. The ROS link's
   ``profile: gazebo`` / ``isaac`` only makes the topic names match what
   stacks written for those simulators expect.

How do I plug in a learned policy or a VLA?
   A LeRobot policy (ACT, Diffusion, pi0, SmolVLA, ...) is one line,
   ``{type: lerobot, path: ...}`` (:doc:`../tutorials/lerobot`). Any other
   model: an ``rf.Policy`` (:doc:`../tutorials/custom_policy`), wrapped in
   ``rf.Async`` if it is slow (:doc:`../tutorials/pid_to_policy`).

Does QGroundControl work with vehicle mode?
   The vehicle sends the messages ground stations use, and plain pymavlink
   clients and a real PX4 have been flown against the SDK. QGroundControl
   and MAVSDK themselves have not been connected yet, so treat it as
   untested.

Where do my prints go when running in the IDE?
   To the Output panel of the run. Use ``flush=True`` (the SDK's own
   messages do) so lines appear as they happen.

How do I stop an app cleanly from code?
   ``app.stop()`` from any thread ends ``run()`` after the current step.
   Ctrl+C and SIGTERM do the same on the command line. Then ``app.close()``.

How do I report a problem?
   The smallest ``app.yaml`` that shows it, the command, the output with
   ``--check --tap``, and ``python3 -m roboforge doctor``.
