"""Name -> class lookup, so configs can say `type: mujoco`.

Deliberately small. Entries are either a class or a lazy "module:Class"
string, so optional dependencies (mujoco, rclpy, pymavlink) are imported only
when that component is actually used. Third-party packages add entries with
`register` or through the `roboforge.plugins` entry-point group.
"""
from __future__ import annotations

import importlib
import inspect
from typing import Any

from .errors import ConfigError, suggest


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
            if not isinstance(name, str):
                raise ConfigError(f"a {self.kind} is named by a string, got {name!r}")
            if ":" not in name:
                raise ConfigError(f"unknown {self.kind} {name!r}.{suggest(name, self.names())} Known: {', '.join(self.names()) or 'none'}. "
                                  f"Use 'module:Class' for your own.")
            target = name
        if isinstance(target, str):
            module, _, attr = target.partition(":")
            try:
                mod = importlib.import_module(module)
            except ModuleNotFoundError as e:
                if name not in self._items and e.name == module.split(".")[0]:
                    # The user's own module, not a missing dependency of ours.
                    raise ConfigError(f"{self.kind} {name!r}: there is no module {module!r}. "
                                      f"Put {module.replace('.', '/')}.py next to the config file, or check the spelling.") from None
                raise ImportError(f"{self.kind} {name!r} needs a package that is not installed: {e}. "
                                  f"Run `python -m roboforge doctor` to see what each component needs.") from e
            except ImportError as e:
                raise ImportError(f"{self.kind} {name!r} needs a package that is not installed: {e}. "
                                  f"Run `python -m roboforge doctor` to see what each component needs.") from e
            if not hasattr(mod, attr):
                have = sorted(k for k, v in vars(mod).items() if inspect.isclass(v) and v.__module__ == mod.__name__)
                raise ConfigError(f"{self.kind} {name!r}: {module} has no class {attr!r}.{suggest(attr, have)} "
                                  f"It defines: {', '.join(have) or 'no classes'}.")
            target = getattr(mod, attr)
            if name in self._items:
                self._items[name] = target
        return target

    def build(self, spec: Any, **extra):
        """Build from `"name"`, `{"type": "name", ...kwargs}`, or pass an instance through."""
        if spec is None or not isinstance(spec, (str, dict)):
            return spec
        if isinstance(spec, str):
            spec = {"type": spec}
        if "type" not in spec:
            raise ConfigError(f"a {self.kind} needs a `type`, got {spec!r}. Known: {', '.join(self.names())}.")
        kwargs = {k: v for k, v in spec.items() if k != "type"}
        kwargs.update(extra)
        cls = self.get(spec["type"])
        # A misspelt argument should read as a config mistake, not as a Python TypeError.
        try:
            params = inspect.signature(cls).parameters
        except (TypeError, ValueError):
            params = None
        if params is not None and not any(p.kind is p.VAR_KEYWORD for p in params.values()):
            allowed = [n for n, p in params.items() if p.kind in (p.POSITIONAL_OR_KEYWORD, p.KEYWORD_ONLY)]
            for k in kwargs:
                if k not in allowed:
                    raise ConfigError(f"{self.kind} {spec['type']!r} has no argument {k!r}.{suggest(k, allowed)} "
                                      f"Its arguments: {', '.join(allowed) or 'none'}.")
        return cls(**kwargs)


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
CALLBACKS.register("tap", "roboforge.callbacks:Tap")
CALLBACKS.register("checks", "roboforge.callbacks:Checks")


def load_plugins() -> None:
    """Import packages that registered themselves under `roboforge.plugins`."""
    try:
        from importlib.metadata import entry_points
        for ep in entry_points(group="roboforge.plugins"):
            ep.load()
    except Exception:
        pass
