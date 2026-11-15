"""Any stack as a Gymnasium environment, for training."""
from __future__ import annotations

import numpy as np

from . import msgs


def _default_observe(obs) -> np.ndarray:
    parts = []
    if obs.joint is not None:
        parts += [obs.joint.position, obs.joint.velocity]
    if obs.pose is not None:
        parts += [obs.pose.position, obs.pose.orientation]
    if obs.twist is not None:
        parts += [obs.twist.linear, obs.twist.angular]
    return np.concatenate(parts).astype(np.float32) if parts else np.zeros(0, np.float32)


class GymEnv:
    """`reset()` / `step(action)` over an `App`.

    `action` becomes a `cmd/actuators` message unless `act` maps it to
    something else (a setpoint for a lower-level controller already in the
    app, for example). `observe`, `reward` and `terminated` are functions of
    the observations; `steps_per_action` physics steps run per `step`.
    Real-time pacing is switched off: training runs as fast as it can.
    """

    def __init__(self, app, observe=None, act=None, reward=None, terminated=None, steps_per_action: int = 10, max_steps: int = 1000):
        import gymnasium as gym
        from .control import Obs
        self.app = app
        app.realtime = False
        app.setup()
        self._Obs = Obs
        self.observe = observe or _default_observe
        self.act = act or (lambda a: msgs.ActuatorCommand(values=np.asarray(a, float)))
        self.reward = reward or (lambda obs: 0.0)
        self.terminated = terminated or (lambda obs: False)
        self.steps_per_action, self.max_steps = steps_per_action, max_steps
        self._n = 0
        rng = getattr(app.physics, "ctrl_range", None)
        if rng is not None and len(rng) and np.all(np.isfinite(rng)):
            self.action_space = gym.spaces.Box(rng[:, 0].astype(np.float32), rng[:, 1].astype(np.float32))
        else:
            n = 0 if rng is None else len(rng)
            self.action_space = gym.spaces.Box(-np.inf, np.inf, (n,), np.float32)
        o = self.observe(Obs(app.bus))
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, o.shape, np.float32)

    def reset(self, seed=None, options=None):
        self.app.reset()
        self._n = 0
        return self.observe(self._Obs(self.app.bus)), {}

    def step(self, action):
        self.app.publish(self.act(action), source="env")
        for _ in range(self.steps_per_action):
            self.app.step()
        self._n += 1
        obs = self._Obs(self.app.bus)
        return self.observe(obs), float(self.reward(obs)), bool(self.terminated(obs)), self._n >= self.max_steps, {}

    def close(self):
        self.app.close()
