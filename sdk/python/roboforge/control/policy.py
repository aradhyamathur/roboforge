"""A learned policy: a controller split into the two halves a trainer needs.

    class MyPolicy(rf.Policy):
        output = "joint"                         # what the action means: joint targets
        def observe(self, obs):                  # what the model sees, as an array
            return np.concatenate([obs.joint.position, obs.joint.velocity])
        def act(self, x):                        # the model: array in, array out (numpy, torch, jax)
            return self.net(x)
        def decode(self, a, obs):                # optional: network output in [-1, 1] -> a joint target
            return obs.joint.position + 0.3 * np.clip(a, -1, 1)

    env = MyPolicy().make_env(rf.App(robot="arm", controllers=[rf.JointPD(rate=500)]), reward=...)
    ...train against env...
    app = rf.App(robot="arm", controllers=[policy, rf.JointPD(rate=500)])      # deploy: the same object

Training and deployment call the same `observe` and `act`, so there is no
second copy of the preprocessing to get out of step. `step` is provided:
it returns None while a signal `observe` needs has not arrived yet, converts the
model's output to numpy (detaching torch tensors, moving them off the GPU),
and wraps it in the message for `output`.
"""
from __future__ import annotations

import numpy as np

from .. import msgs
from . import Controller

OUTPUTS = {
    "joint": "joint position targets (cmd/joint), for joint_pd underneath",
    "actuators": "motor commands (cmd/actuators), straight to physics",
    "velocity": "body velocity, [forward, sideways, up, roll rate, pitch rate, yaw rate] or [forward, yaw rate] (cmd/velocity)",
    "setpoint": "a world position [x, y, z] or [x, y, z, yaw] (cmd/setpoint), for the multirotor controller",
}


class Policy(Controller):
    """Override `observe(obs) -> array` and `act(x) -> array`; set `output`."""
    output: str = "joint"

    def __init__(self, rate=None, name: str = ""):
        super().__init__(rate=rate, name=name)
        if self.output not in OUTPUTS:
            from ..errors import ConfigError, suggest
            raise ConfigError(f"policy output must be one of {', '.join(OUTPUTS)}, got {self.output!r}.{suggest(self.output, OUTPUTS)}")

    def observe(self, obs) -> np.ndarray:
        raise NotImplementedError("a Policy needs observe(obs) -> array: what the model sees")

    def act(self, x):
        raise NotImplementedError("a Policy needs act(x) -> array: the model")

    def decode(self, action, obs):
        """Map the network's raw output to the values `command` sends. Default: as is.

        Override it to let the network work in a convenient range, typically
        [-1, 1] as PPO and SAC expect, and turn that into, say, a step from the
        current joint positions. It runs in training and on the robot alike,
        and when it is overridden `make_env` gives an action space of [-1, 1].
        """
        return action

    def command(self, action):
        """The model's output as a message on the bus."""
        from ..env import to_numpy
        a = to_numpy(action).reshape(-1)
        if not np.all(np.isfinite(a)):
            raise ValueError(f"policy {self.name or type(self).__name__} produced nan or inf: {a}")
        if self.output == "joint":
            return msgs.JointCommand(position=a)
        if self.output == "actuators":
            return msgs.ActuatorCommand(values=a)
        if self.output == "velocity":
            if len(a) == 2:
                return msgs.velocity_command((a[0], 0.0, 0.0), (0.0, 0.0, a[1]))
            return msgs.velocity_command(a[:3], a[3:6])
        return msgs.Setpoint(position=a[:3], yaw=float(a[3]) if len(a) > 3 else 0.0)

    def step(self, obs, t):
        try:
            x = self.observe(obs)
        except (KeyError, AttributeError, TypeError) as e:
            # A signal the policy reads has not been published yet (obs.joint is None, obs["task/goal"]
            # is missing): wait for it. If it never comes, say so instead of waiting for ever.
            self._missing += 1
            if self._missing >= 1000:
                e.add_note(f"(policy {self.name or type(self).__name__} could not build its observation for 1000 calls in a row: "
                           f"a signal it reads is never published, or observe() has a bug)")
                raise
            return None
        self._missing = 0
        from ..env import to_numpy
        return self.command(self.decode(to_numpy(self.act(np.asarray(x, dtype=np.float32))), obs))

    _missing = 0

    def action_space(self, app):
        """The Gymnasium action space for `output`, from the app's limits."""
        from ..env import box
        p = app.physics
        if self.output == "joint":
            return box(getattr(p, "joint_range", None) if getattr(p, "joint_range", None) is not None
                       else np.tile([-np.pi, np.pi], (len(getattr(p, "joint_names", [])), 1)))
        if self.output == "actuators":
            return box(getattr(p, "ctrl_range", None))
        if self.output == "velocity":
            return box([[-2.0, 2.0], [-3.0, 3.0]])
        return box([[-50, 50], [-50, 50], [0, 30]])

    def make_env(self, app, reward=None, terminated=None, steps_per_action: int = None, max_steps: int = 200, action_space=None,
                 on_reset=None):
        """`app.as_env(...)` wired to this policy: observations from `observe`,
        actions through `decode` and `command`, spaces filled in, one action per
        policy period. `on_reset(rng) -> {signal: message}` starts each episode
        (a new goal, say)."""
        from . import Obs
        from ..env import box, to_numpy
        app.setup()
        if steps_per_action is None:
            steps_per_action = max(1, int(round(1.0 / (self.rate * app.physics.dt)))) if self.rate else 10
        decodes = type(self).decode is not Policy.decode
        if action_space is None:
            space = self.action_space(app)
            action_space = box(np.tile([-1.0, 1.0], (space.shape[0], 1))) if decodes else space
        return app.as_env(observe=self.observe, act=lambda a: self.command(self.decode(to_numpy(a), Obs(app.bus))),
                          reward=reward, terminated=terminated, steps_per_action=steps_per_action, max_steps=max_steps,
                          action_space=action_space, on_reset=on_reset)
