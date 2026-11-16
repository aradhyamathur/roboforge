14. Train and deploy your own policy
====================================

**You will:** define a neural-network policy, train it, save it, and run it
as an ordinary controller on goals it never saw. **Time:** 10 minutes, one
of them training. **Needs:** MuJoCo, gymnasium. No GPU.

What it takes
-------------

One class with two methods. Everything else (the environment, its spaces,
the conversion to messages, waiting for signals, deployment) comes from
``rf.Policy``:

.. literalinclude:: ../../examples/custom_policy.py
   :language: python
   :pyobject: ReachPolicy

``observe`` says what the network sees, ``act`` is the network, ``output``
says what its answer means. Training calls these two; so does the robot.
There is one copy of the preprocessing, so it cannot drift between training
and deployment, a classic source of "it worked in training" bugs.

Train
-----

.. literalinclude:: ../../examples/custom_policy.py
   :language: python
   :pyobject: train

``policy.make_env(app, reward=...)`` is a standard ``gymnasium.Env``:

- observations come from ``policy.observe``
- actions go through ``policy.command``, which turns them into ``output``
  messages
- the action space is the joint limits, because ``output = "joint"``
- one action per policy period (20 Hz here, 25 physics steps)

``on_reset`` gives every episode a new goal; ``env.reset(options={"publish":
{"task/goal": goal}})`` sets a particular one. The goal is a signal, like the
instruction a vision-language-action model reads: the policy reads it,
something else sets it.

.. code-block:: console

   $ python3 custom_policy.py
   policy: 163 parameters; training on random goals ...
   iteration   1: best  -20.65  mean  -37.21
   ...
   iteration  30: best   -7.82  mean   -9.77
   trained in 52s, saved to policy.npz

   goal (rad)                 reached                     error
   [ 0.66  0.27 -0.48]        [ 0.63  0.29 -0.46]         0.031
   [ 0.47 -0.18 -0.33]        [ 0.43 -0.17 -0.3 ]         0.042
   [ 0.63  0.72 -1.43]        [ 0.59  0.76 -1.43]         0.037
   [-0.12  0.18 -0.4 ]        [-0.17  0.19 -0.35]         0.055
   [ 0.35  0.77 -1.02]        [ 0.31  0.8  -1.  ]         0.039

   worst error 0.055 rad over 5 new goals; checks: nothing to report

Deploy
------

.. literalinclude:: ../../examples/custom_policy.py
   :language: python
   :pyobject: deploy

The trained policy is a controller: list it before ``JointPD`` and publish a
goal. Wrap it in ``rf.Async`` if inference is slow (:doc:`pid_to_policy`);
nothing else changes.

With PyTorch and stable-baselines3
----------------------------------

The environment is an ordinary Gymnasium one, so standard trainers take it
as is, with no wrappers. ``examples/sb3_ppo.py`` trains PPO from
stable-baselines3 and deploys the trained PyTorch network through the same
``rf.Policy``:

.. literalinclude:: ../../examples/sb3_ppo.py
   :language: python
   :pyobject: SB3Policy

.. literalinclude:: ../../examples/sb3_ppo.py
   :language: python
   :pyobject: make_env

Three things make this short:

- ``act`` returns a PyTorch tensor, and ``rf.Policy`` converts it (detaching
  it and moving it off the GPU if need be).
- ``decode`` maps the network's [-1, 1] output to a step from the current
  pose, the range PPO expects. Because it runs in training and on the robot
  alike, it is written once, and ``make_env`` sets the action space to
  [-1, 1] by itself.
- ``on_reset`` draws a new goal for every episode. stable-baselines3 calls
  ``reset()`` without options, so this is where per-episode settings go.

.. code-block:: console

   $ pip install torch stable-baselines3          # the CPU build is enough
   $ python3 sb3_ppo.py
   PPO: 24000 steps in 58s
   worst joint error on 5 new goals: 1.15 rad untrained, 0.05 rad trained
   deployed through rf.Policy: act() returned a torch tensor, the SDK made the JointCommand

Several environments in parallel processes are standard Gymnasium too:

.. code-block:: python

   venv = gymnasium.vector.AsyncVectorEnv([make_env] * 8)     # one app per process

Try next
--------

- ``output = "actuators"`` and learn torques directly: harder, as the
  numbers will show.
- Add the camera to ``observe`` (``obs.camera("overview").data``) and a
  convolutional network to ``act``.

If something goes wrong
-----------------------

``with your own `act`, also give `action_space```
   ``app.as_env(act=...)`` cannot guess the range of actions that are not
   motor commands. ``policy.make_env`` fills it in.

The policy never acts
   It waits until every signal ``observe`` reads has arrived. If one never
   does (``task/goal`` not published), it says so after 1000 calls.

``policy ReachPolicy produced nan or inf``
   The network diverged; the bad action is refused before it reaches the arm.
