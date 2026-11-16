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

from .errors import ConfigError, suggest

TOP_LEVEL = {"roboforge", "name", "robot", "physics", "renderer", "links", "controllers", "callbacks", "realtime", "speed", "run"}


def load(config) -> tuple[dict, Path]:
    """Read a config file or accept a dict. Returns it with the folder paths are relative to."""
    if isinstance(config, dict):
        return copy.deepcopy(config), Path.cwd()
    path = Path(config)
    try:
        cfg = yaml.safe_load(path.read_text()) or {}
    except yaml.YAMLError as e:
        raise ConfigError(f"{path} is not valid YAML: {' '.join(str(e).split())}") from None
    if not isinstance(cfg, dict):
        raise ConfigError(f"{path}: expected `key: value` settings at the top level, found a {type(cfg).__name__}")
    return cfg, path.resolve().parent


def apply_overrides(cfg: dict, overrides) -> dict:
    for item in overrides:
        if "=" not in item:
            raise ConfigError(f"override {item!r} is not of the form path=value (for example physics.substeps=2)")
        path, _, raw = item.partition("=")
        value = yaml.safe_load(raw)
        node = cfg
        keys = path.split(".")
        for i, k in enumerate(keys[:-1]):
            if isinstance(node, list):
                where = ".".join(keys[:i])
                if not k.lstrip("-").isdigit() or not -len(node) <= int(k) < len(node):
                    raise ConfigError(f"override {item!r}: {where} is a list of {len(node)}, so the next part must be an index "
                                      f"from 0 to {len(node) - 1}, got {k!r}" if node else f"override {item!r}: {where} is empty, there is nothing to override")
                k = int(k)
            elif not isinstance(node, dict):
                raise ConfigError(f"override {item!r}: {'.'.join(keys[:i])} is a plain value, it has no {k!r}")
            try:
                nxt = node[k]
            except KeyError:
                nxt = node[k] = {}
            if isinstance(nxt, str):      # "mujoco" -> {type: mujoco} so it can take arguments
                nxt = node[k] = {"type": nxt}
            node = nxt
        last = keys[-1]
        if isinstance(node, list):
            if not last.lstrip("-").isdigit() or not -len(node) <= int(last) < len(node):
                raise ConfigError(f"override {item!r}: {'.'.join(keys[:-1])} is a list of {len(node)}; {last!r} is not an index into it")
            node[int(last)] = value
        elif isinstance(node, dict):
            node[last] = value
        else:
            raise ConfigError(f"override {item!r}: {'.'.join(keys[:-1])} is a plain value, it has no {last!r}")
    return cfg


def resolve(config, overrides=()) -> tuple[dict, Path]:
    cfg, base = load(config)
    apply_overrides(cfg, overrides)
    unknown = set(cfg) - TOP_LEVEL
    if unknown:
        hints = "".join(suggest(k, TOP_LEVEL) for k in sorted(unknown))
        raise ConfigError(f"unknown config keys: {', '.join(sorted(unknown))}.{hints} Allowed: {', '.join(sorted(TOP_LEVEL))}")
    for k in ("links", "controllers", "callbacks"):
        if cfg.get(k) is not None and not isinstance(cfg[k], list):
            raise ConfigError(f"'{k}' must be a list (lines starting with '- '), got a {type(cfg[k]).__name__}")
    # File paths inside components are relative to the config file, like the robot's model.
    for section in ("controllers", "callbacks", "links"):
        for spec in cfg.get(section) or []:
            if isinstance(spec, dict):
                for k in ("source", "library"):
                    if isinstance(spec.get(k), str) and not Path(spec[k]).is_absolute():
                        spec[k] = str(base / spec[k])
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
