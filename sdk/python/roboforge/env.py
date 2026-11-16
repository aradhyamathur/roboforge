"""Any stack as a Gymnasium environment, for training."""
from __future__ import annotations

import gymnasium as gym
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


def to_numpy(x) -> np.ndarray:
    """An action as a float numpy array, whatever produced it: numpy, a list,
    a PyTorch tensor (also on a GPU, also requiring grad), a JAX array."""
    if hasattr(x, "detach"):                     # torch
        x = x.detach()
    if hasattr(x, "cpu"):
        x = x.cpu()
    if hasattr(x, "numpy") and not isinstance(x, np.ndarray):
        x = x.numpy()
    return np.asarray(x, dtype=float)


def box(ranges, dtype=np.float32):
    """A Gymnasium Box from an `(n, 2)` array of `[low, high]` rows (inf for unbounded)."""
    r = np.asarray(ranges if ranges is not None else np.zeros((0, 2)), float).reshape(-1, 2)
    return gym.spaces.Box(r[:, 0].astype(dtype), r[:, 1].astype(dtype), dtype=dtype)


class GymEnv(gym.Env):
    """`reset()` / `step(action)` over an `App`.

    `action` becomes a `cmd/actuators` message unless `act` maps it to
    something else (a setpoint for a lower-level controller already in the
    app, for example). `observe`, `reward` and `terminated` are functions of
    the observations; `steps_per_action` physics steps run per `step`.
    Real-time pacing is switched off: training runs as fast as it can.
    """

    def __init__(self, app, observe=None, act=None, reward=None, terminated=None, steps_per_action: int = 10, max_steps: int = 1000,
                 action_space=None, observation_space=None, on_reset=None):
        from .control import Obs
        self.app = app
        app.realtime = False
        app.setup()
        self._Obs = Obs
        self.observe = observe or _default_observe
        self.act = act or (lambda a: msgs.ActuatorCommand(values=to_numpy(a)))
        self.reward = reward or (lambda obs: 0.0)
        self.terminated = terminated or (lambda obs: False)
        self.steps_per_action, self.max_steps = steps_per_action, max_steps
        self.on_reset = on_reset
        self._n = 0
        if action_space is not None:
            self.action_space = action_space
        elif act is None:
            self.action_space = box(getattr(app.physics, "ctrl_range", None))
        else:
            raise ValueError("with your own `act`, also give `action_space`: the actions are no longer motor commands, so their "
                             "range cannot be guessed. rf.env.box(ranges) makes one from an (n, 2) array of limits; "
                             "rf.Policy.make_env does this for you.")
        if observation_space is not None:
            self.observation_space = observation_space
        else:
            # The space is measured from one observation, so publish what an episode starts with first.
            for name, msg in (dict(on_reset(self.np_random)) if on_reset else {}).items():
                app.bus.publish(name, msg, "env")
            try:
                o = np.asarray(self.observe(Obs(app.bus)), np.float32)
            except (KeyError, AttributeError, TypeError) as e:
                raise ValueError(f"observe() could not run before the first episode ({type(e).__name__}: {e}). A signal it reads "
                                 f"is not published yet: give on_reset=lambda rng: {{signal: value}} to provide it, "
                                 f"or pass observation_space.") from None
            self.observation_space = gym.spaces.Box(-np.inf, np.inf, o.shape, np.float32)

    def _observe(self):
        obs = self._Obs(self.app.bus)
        return obs, np.asarray(self.observe(obs), dtype=self.observation_space.dtype)

    def reset(self, *, seed=None, options=None):
        """Start a new episode: reset the app and return `(observation, info)`.

        `on_reset(rng)` (given when the environment is made) returns signals to
        publish at the start of every episode, `{"task/goal": rng.uniform(...)}`;
        trainers such as stable-baselines3 call `reset()` without options, so
        this is where per-episode goals belong. `options={"publish": {...}}`
        adds or overrides signals for one reset.
        """
        super().reset(seed=seed)
        self.app.reset()
        signals = dict(self.on_reset(self.np_random)) if self.on_reset else {}
        signals.update((options or {}).get("publish") or {})
        for name, msg in signals.items():
            self.app.bus.publish(name, msg, "env")
        self._n = 0
        return self._observe()[1], {}

    def step(self, action):
        """Apply `action`, advance `steps_per_action` physics steps, and return
        `(observation, reward, terminated, truncated, info)`."""
        self.app.publish(self.act(action), source="env")
        for _ in range(self.steps_per_action):
            self.app.step()
        self._n += 1
        obs, o = self._observe()
        return o, float(self.reward(obs)), bool(self.terminated(obs)), self._n >= self.max_steps, {}

    def close(self):
        """Close the app underneath."""
        self.app.close()
