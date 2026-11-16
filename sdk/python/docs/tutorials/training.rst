11. Train something
===================

**You will:** turn an app into a Gymnasium environment and train a small
policy on it. **Time:** 5 minutes. **Needs:** MuJoCo, gymnasium. No GPU and
no deep-learning library.

Any app is an environment
-------------------------

.. code-block:: python

   env = rf.App(robot="arm").as_env()
   obs, info = env.reset(seed=0)
   obs, reward, terminated, truncated, info = env.step(env.action_space.sample())

``as_env()`` returns a real ``gymnasium.Env`` (it passes Gymnasium's
``check_env``), so wrappers, vector environments and libraries such as
stable-baselines3 accept it.

.. list-table::
   :header-rows: 1
   :widths: 24 76

   * - Argument
     - Meaning
   * - ``observe``
     - ``obs -> array``. Default: joints, base pose and velocity, concatenated
   * - ``act``
     - ``action -> message``. Default: ``cmd/actuators`` (raw motor commands)
   * - ``action_space``
     - Required with your own ``act``: those actions are not motor commands, so
       their range cannot be guessed. ``roboforge.env.box(ranges)`` makes one,
       for example from ``app.physics.joint_range``
   * - ``on_reset``
     - ``rng -> {signal: message}``, published at the start of every episode:
       a new goal, a new start pose
   * - ``reward``, ``terminated``
     - functions of the observations
   * - ``steps_per_action``
     - physics steps per ``env.step`` (default 10: 50 actions per second)
   * - ``max_steps``
     - episode length, after which ``truncated`` is true

Learn to reach
--------------

.. literalinclude:: ../../examples/gym_reach.py
   :language: python
   :lines: 10-

The action here is a *joint target*, not a torque: ``act`` maps it to a
``JointCommand`` and the ``JointPD`` controller already in the app does the
rest. Learning on top of a low-level controller is easier than learning
torques, and it is one argument.

.. code-block:: console

   $ python3 gym_reach.py
   observation (19,), action (3,)
   start      reward   -80.00
   episode 30  reward   -37.31
   ...
   episode 150 reward    -6.76

   reached [ 0.61  0.46 -0.93] for goal [ 0.6  0.5 -0.9]: worst joint error 0.04 rad

Ten seconds on a CPU. Random search over nine numbers is not a serious
algorithm; the point is that the environment is ordinary, so a serious one
drops in.

For a policy of your own, ``rf.Policy.make_env`` fills all of this in from
the policy (:doc:`custom_policy`).

Good to know
------------

- Runs are **deterministic**: the same actions after ``reset()`` give exactly
  the same states, which makes training bugs reproducible.
- Real-time pacing is switched off inside an environment.
- Actions outside the motors' range are clipped; ``nan`` is refused with an
  error naming it.
- One world per process. For parallel rollouts use Gymnasium's
  ``AsyncVectorEnv`` with one app per worker.

If something goes wrong
-----------------------

``cmd/actuators has 11 values but arm has 3 actuators``
   The action does not match the action space; ``env.action_space.shape``
   says what is expected.
