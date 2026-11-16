19. Imitation learning with LeRobot
===================================

**You will:** record demonstrations from a simulated arm straight into a
LeRobot dataset, train an ACT policy on them, and run it as an ordinary
controller. **Time:** 15 minutes, about a minute of it training on a CPU.
**Needs:** MuJoCo and LeRobot (``pip install "lerobot[dataset]"``); a GPU is
optional.

`LeRobot <https://github.com/huggingface/lerobot>`_ is Hugging Face's
library for robot learning: a dataset format the Hugging Face hub hosts,
and policies (ACT, Diffusion Policy, pi0, SmolVLA, ...) that train on it. The
SDK speaks it at both ends: any app can record into it, and any LeRobot
policy can drive any app.

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - In app.yaml
     - Does
   * - ``type: lerobot_recorder``
     - records the run as a ``LeRobotDataset``, one episode per reset or per
       ``episode_seconds``
   * - ``python -m roboforge lerobot train``
     - trains any LeRobot policy on it, with LeRobot's own factories
   * - ``type: lerobot``
     - runs a trained policy (a folder, or a Hugging Face hub id) as a
       controller

Setting up
----------

LeRobot 0.7 requires ``numpy < 2.3`` and its own range of PyTorch, so keep it
in an environment of its own rather than changing the one your other apps
use. To keep ROS 2 importable, build it from the system Python as usual, and
install SciPy inside it, or Ubuntu's own SciPy (built for an older numpy)
will be picked up instead:

.. code-block:: console

   $ uv venv --python /usr/bin/python3 --system-site-packages .venv-lerobot
   $ uv pip install --python .venv-lerobot/bin/python --torch-backend cpu \
         "lerobot[dataset]==0.7.0" mujoco pymavlink pyyaml scipy -e sdk/python
   $ source .venv-lerobot/bin/activate
   $ python -m roboforge doctor          # lerobot: ok

For a GPU, use ``--torch-backend cu126`` (or the one matching your driver;
see `Training on a GPU`_). The whole SDK test suite passes in this
environment too.

1. Record demonstrations
------------------------

``examples/lerobot_reach`` has an expert that reaches a new random goal
every two seconds along a smooth path, and a recorder that writes what a
policy would see and what it should do:

.. literalinclude:: ../../examples/lerobot_reach/record.yaml
   :language: yaml
   :lines: 7-

.. code-block:: console

   $ cd lerobot_reach
   $ python3 -m roboforge run record.yaml --headless --no-realtime --duration 80
   roboforge: lerobot_recorder saved 40 episodes, 1601 frames, to datasets/arm_reach
   $ python3 -m roboforge lerobot info datasets/arm_reach

About 15 seconds. The mapping from signals to LeRobot features:

.. list-table::
   :header-rows: 1
   :widths: 34 26 40

   * - Recorder setting
     - LeRobot feature
     - Here
   * - ``state: [state/joint]``
     - ``observation.state``
     - joint positions then velocities, named ``yaw.pos``, ... ``elbow.vel``
   * - ``environment: [task/goal]``
     - ``observation.environment_state``
     - the goal the expert is reaching for
   * - ``cameras: [overview]``
     - ``observation.images.overview``
     - none here; see `With cameras`_
   * - ``action: cmd/joint``
     - ``action``
     - the joint target sent to ``joint_pd``
   * - ``task`` (or ``cmd/instruction``)
     - ``task``
     - the language instruction stored with every frame

Any signal can go in ``state`` or ``environment``: a pose, an IMU, an
estimate, your own message type with ``as_array()``. The recorder starts
once all of them have been published, and saves the mapping next to the
dataset (``meta/roboforge.json``) so the policy can find its inputs later.

2. Train
--------

.. code-block:: console

   $ python3 -m roboforge lerobot train datasets/arm_reach --out policies/arm_act --steps 1500 \
         chunk_size=10 n_action_steps=10 dim_model=64 n_heads=4 dim_feedforward=256 \
         n_encoder_layers=2 n_decoder_layers=1 use_vae=false optimizer_lr=0.001
   roboforge: training act (170,691 parameters) on 1601 frames from 40 episodes, on cpu
     step    200  loss ...
     step   1500  loss 0.0821  (26s)
   roboforge: saved the policy to policies/arm_act

``--policy`` picks any LeRobot policy type (``act`` by default,
``diffusion``, ...); ``name=value`` arguments go to its configuration. The
options above shrink ACT from its default size (tens of millions of
parameters, built for real robots with cameras) to one that trains in
half a minute on a CPU. The folder holds what LeRobot saves
(``config.json``, ``model.safetensors``, the normalising pre- and
post-processors) plus ``roboforge.json``.

From Python, the same is ``rf.lerobot.train(dataset, out, policy="act", steps=1500, **options)``.
LeRobot's own ``lerobot-train`` command works on these datasets as well.

3. Deploy
---------

.. literalinclude:: ../../examples/lerobot_reach/deploy.yaml
   :language: yaml
   :lines: 8-

.. code-block:: console

   $ python3 -m roboforge run deploy.yaml --headless --duration 10 --tap 2.5
     cmd/joint         20.0 Hz  lerobot    q* [0.52 0.53 -0.88]
     state/joint      500.4 Hz  mujoco     q [0.53 0.55 -0.91]
     task/goal         20.0 Hz  Goals      array([ 0.6,  0.5, -0.9])

The policy runs at the dataset's 20 Hz, reads the joints and the goal as it
was trained to, and its actions go to ``joint_pd`` like any controller's.
LeRobot's queue of chunked actions is cleared on ``app.reset()``. The
policy waits, returning nothing, until its inputs have been published.

4. Evaluate
-----------

.. code-block:: console

   $ python3 evaluate.py
   goal (rad)                 policy   expert
   [ 0.36 -0.14 -1.18]         0.150    0.035
   ...
   worst joint error over 8 new goals: policy 0.190 rad, expert 0.035 rad

Each test starts from the home pose with a goal never seen in training.
The policy gets within 0.06 to 0.19 rad of the goal; the expert it copies
within 0.04. Three times the demonstrations (``--duration 240``) and 4000
training steps (75 seconds) bring the worst case to 0.12 rad. As in any
imitation learning, the data matters most: these demonstrations run back to
back, so they rarely start from home, where the evaluation starts.

With cameras
------------

Add a renderer and list the camera; the recorder adds
``observation.images.overview`` and the policy reads it the same way:

.. code-block:: yaml

   renderer: {type: mujoco, width: 96, height: 72, fps: 20}
   callbacks:
     - {type: lerobot_recorder, root: datasets/arm_cam, fps: 20, state: [state/joint],
        environment: [task/goal], cameras: [overview], action: cmd/joint, episode_seconds: 2.0}

ACT's camera encoder (a ResNet-18) makes training far slower: use a GPU, or
``pretrained_backbone_weights=null`` and few steps to check the pipeline on
a CPU. ``use_videos: true`` stores the frames as MP4 instead of PNG.

A pretrained policy from the hub
--------------------------------

``path`` takes a Hugging Face hub id as well as a folder:

.. code-block:: yaml

   - {type: lerobot, path: <user>/<policy>, state: [state/joint], cameras: [front], output: joint}

A policy trained elsewhere has no ``roboforge.json``, so say which signals
make its inputs (``state``, ``environment``, ``cameras``) and what its
actions mean (``output``). If they do not match the policy's input features,
it says so when the app starts. The policy's camera names, image sizes,
joint order and units must also match your robot's: a policy trained on
another robot does not transfer by itself.

Training on a GPU
-----------------

``--device auto`` (the default) trains on a GPU when PyTorch can use one.
PyTorch has to be built for a CUDA version the driver supports:

.. code-block:: console

   $ nvidia-smi                                   # the driver's CUDA version, top right
   $ uv pip install --torch-backend cu126 "torch>=2.7,<2.16"
   $ python -c "import torch; print(torch.cuda.is_available())"

On a GTX 1070 (driver 560, CUDA 12.6) PyTorch 2.14 built for CUDA 12.6
works; the default PyPI build, for CUDA 13, does not start there. Measured
on that card against an 8-core CPU:

.. list-table::
   :header-rows: 1
   :widths: 50 25 25

   * - Training
     - GPU
     - CPU
   * - the small state-only ACT above, 4000 steps
     - 66 s
     - 72 s
   * - ACT at its default size with a 96×72 camera (ResNet-18), 200 steps
     - 12 s
     - 103 s

A GPU pays off for camera inputs and large models; the small policy of this
tutorial trains as fast on a CPU. The policy runs on the device it was
loaded on (``device: auto`` by default) inside the simulation too.

Writing it yourself
-------------------

The pieces are ordinary SDK parts, so they combine with the rest:

.. code-block:: python

   from roboforge import lerobot as rl

   rec = rl.LeRobotRecorder(root="datasets/x", fps=20, state=["state/joint"], cameras=["front"], action="cmd/joint")
   app = rf.App(robot="arm", controllers=[teleop, rf.JointPD(rate=500)], callbacks=[rec])
   for episode in range(20):
       app.run(duration=5)
       app.reset()                    # ends the episode; rec.discard_episode() drops a bad one
   app.close()                        # writes the dataset

   policy = rl.LeRobotPolicy("policies/x", device="auto")
   app = rf.App(robot="arm", controllers=[rf.Async(policy, rate=20), rf.JointPD(rate=500)])

Wrap a slow policy in ``rf.Async`` (:doc:`pid_to_policy`) so inference never
holds up the control loop.

If something goes wrong
-----------------------

``lerobot ... needs a package that is not installed`` / ``LeRobot is not installed``
   Use the environment from `Setting up`_.

``ImportError: numpy.core.multiarray failed to import`` (from ``scipy``)
   Ubuntu's SciPy was picked up: install ``scipy`` inside the environment.

``datasets/x already holds a LeRobot dataset. Use mode: append ...``
   Recording never overwrites. ``mode: append`` adds episodes, and checks
   that they record the same signals.

``the policy at P takes [...], but this controller would give it [...]``
   The ``state``, ``environment`` and ``cameras`` lists must match the ones
   the dataset was recorded with.

``no LeRobot policy at P: the folder does not exist``
   Train one first, or check the path (relative to ``app.yaml``).

The policy never acts
   One of its inputs is never published (``--tap`` shows which signal is
   missing), for example no goal on ``task/goal``.
