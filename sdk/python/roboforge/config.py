"""Apps from YAML. Flat on purpose: no inheritance, no interpolation.

    roboforge: 1
    name: quad
    robot: quadrotor                      # or {mjcf: my.xml, profile: arm}
    physics: {type: mujoco}
    renderer: {type: mujoco, fps: 10}
    links:
      - {type: mavlink, mode: vehicle}
    controllers:
      - {type: multirotor, rate: 250}
      - {type: "controller:Patrol", rate: 2}     # your class, in controller.py next to this file
    callbacks:
      - {type: recorder, dir: runs/latest}

Every key under a component is a constructor argument of that class, and
`type` is a registered name or `module:Class`. Overrides are `path=value`
strings (`physics.substeps=2`, `links.0.url=udpout:10.0.0.2:14550`).
`resolve()` returns the exact dict that will be built, for printing.
"""
from __future__ import annotations

import copy
import sys
from pathlib import Path

import yaml

TOP_LEVEL = {"roboforge", "name", "robot", "physics", "renderer", "links", "controllers", "callbacks", "realtime", "speed", "run"}


def load(config) -> tuple[dict, Path]:
    """Read a config file or accept a dict. Returns it with the folder paths are relative to."""
    if isinstance(config, dict):
        return copy.deepcopy(config), Path.cwd()
    path = Path(config)
    cfg = yaml.safe_load(path.read_text()) or {}
    if not isinstance(cfg, dict):
        raise ValueError(f"{path}: expected a mapping at the top level")
    return cfg, path.resolve().parent


def apply_overrides(cfg: dict, overrides) -> dict:
    for item in overrides:
        if "=" not in item:
            raise ValueError(f"override {item!r} is not of the form path=value")
        path, _, raw = item.partition("=")
        value = yaml.safe_load(raw)
        node = cfg
        keys = path.split(".")
        for i, k in enumerate(keys[:-1]):
            k = int(k) if isinstance(node, list) else k
            try:
                nxt = node[k]
            except (KeyError, IndexError):
                if isinstance(node, list):
                    raise KeyError(f"override {item!r}: no index {k} in {'.'.join(keys[:i])}") from None
                nxt = node[k] = {}
            if isinstance(nxt, str):      # "mujoco" -> {type: mujoco} so it can take arguments
                nxt = node[k] = {"type": nxt}
            node = nxt
        last = keys[-1]
        if isinstance(node, list):
            node[int(last)] = value
        else:
            node[last] = value
    return cfg


def resolve(config, overrides=()) -> tuple[dict, Path]:
    cfg, base = load(config)
    apply_overrides(cfg, overrides)
    unknown = set(cfg) - TOP_LEVEL
    if unknown:
        raise ValueError(f"unknown config keys: {', '.join(sorted(unknown))}. Allowed: {', '.join(sorted(TOP_LEVEL))}")
    robot = cfg.get("robot")
    if isinstance(robot, dict):
        for k in ("mjcf", "urdf"):
            if k in robot and not Path(robot[k]).is_absolute():
                robot[k] = str(base / robot[k])
    return cfg, base


def build_app(config, overrides=(), **replace):
    from .app import App
    from .registry import load_plugins
    cfg, base = resolve(config, overrides)
    # Classes named as `module:Class` are looked up next to the config file first.
    if str(base) not in sys.path:
        sys.path.insert(0, str(base))
    load_plugins()
    kwargs = {k: cfg[k] for k in ("robot", "physics", "renderer", "realtime", "speed") if k in cfg}
    for k in ("links", "controllers", "callbacks"):
        kwargs[k] = list(cfg.get(k) or [])
    kwargs["name"] = cfg.get("name", "app")
    kwargs.update(replace)
    app = App(**kwargs)
    app.config = cfg
    return app
