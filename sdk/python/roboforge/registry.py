"""Name -> class lookup, so configs can say `type: mujoco`.

Deliberately small. Entries are either a class or a lazy "module:Class"
string, so optional dependencies (mujoco, rclpy, pymavlink) are imported only
when that component is actually used. Third-party packages add entries with
`register` or through the `roboforge.plugins` entry-point group.
"""
from __future__ import annotations

import importlib
from typing import Any


class Registry:
    def __init__(self, kind: str):
        self.kind = kind
        self._items: dict[str, Any] = {}

    def register(self, name: str, target: Any = None):
        """`REG.register("name", cls_or_path)` or `@REG.register("name")`."""
        if target is not None:
            self._items[name] = target
            return target

        def deco(cls):
            self._items[name] = cls
            return cls
        return deco

    def names(self) -> list[str]:
        return sorted(self._items)

    def get(self, name: str):
        """Resolve a registered name, or an explicit `package.module:Class` path."""
        target = self._items.get(name)
        if target is None:
            if ":" not in name:
                raise KeyError(f"unknown {self.kind} {name!r}. Known: {', '.join(self.names()) or 'none'}. "
                               f"Use 'package.module:Class' for your own.")
            target = name
        if isinstance(target, str):
            module, _, attr = target.partition(":")
            try:
                target = getattr(importlib.import_module(module), attr)
            except ImportError as e:
                raise ImportError(f"{self.kind} {name!r} needs a package that is not installed: {e}. "
                                  f"Run `python -m roboforge doctor` to see what each component needs.") from e
            if name in self._items:
                self._items[name] = target
        return target

    def build(self, spec: Any, **extra):
        """Build from `"name"`, `{"type": "name", ...kwargs}`, or pass an instance through."""
        if spec is None or not isinstance(spec, (str, dict)):
            return spec
        if isinstance(spec, str):
            spec = {"type": spec}
        kwargs = {k: v for k, v in spec.items() if k != "type"}
        kwargs.update(extra)
        return self.get(spec["type"])(**kwargs)


PHYSICS = Registry("physics backend")
RENDERERS = Registry("renderer")
LINKS = Registry("link")
CONTROLLERS = Registry("controller")
CALLBACKS = Registry("callback")

PHYSICS.register("mujoco", "roboforge.physics.mujoco:MujocoPhysics")
PHYSICS.register("none", "roboforge.physics.none:NoPhysics")
RENDERERS.register("mujoco", "roboforge.render.mujoco:MujocoRenderer")
LINKS.register("ros2", "roboforge.links.ros2:Ros2Link")
LINKS.register("mavlink", "roboforge.links.mavlink:MavlinkLink")
CONTROLLERS.register("pid", "roboforge.control.pid:PID")
CONTROLLERS.register("joint_pd", "roboforge.control.joint:JointPD")
CONTROLLERS.register("multirotor", "roboforge.control.multirotor:MultirotorController")
CONTROLLERS.register("diffdrive", "roboforge.control.diffdrive:DiffDriveController")
CALLBACKS.register("recorder", "roboforge.callbacks:Recorder")
CALLBACKS.register("rate_monitor", "roboforge.callbacks:RateMonitor")
CALLBACKS.register("viewer", "roboforge.callbacks:Viewer")


def load_plugins() -> None:
    """Import packages that registered themselves under `roboforge.plugins`."""
    try:
        from importlib.metadata import entry_points
        for ep in entry_points(group="roboforge.plugins"):
            ep.load()
    except Exception:
        pass
