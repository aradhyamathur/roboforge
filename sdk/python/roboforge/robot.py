"""A robot: a model file plus the few facts the rest of the stack needs.

    Robot.from_zoo("quadrotor")              # bundled models
    Robot.from_mjcf("my_arm.xml")            # your MuJoCo model
    Robot.from_urdf("humanoid.urdf")         # your URDF (MuJoCo loads URDF too)

`profile` says what kind of machine it is (`multirotor`, `arm`, `diffdrive`,
or the generic `articulated`), which picks sensible default controllers and
link mappings. `params` holds anything a controller needs that the model does
not state (wheel radius, hover altitude, gains).
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import yaml

from .errors import ConfigError, suggest

ZOO = Path(__file__).parent / "zoo"
PROFILES = ("multirotor", "arm", "diffdrive", "articulated")


@dataclass
class Robot:
    name: str
    model: str                      # path to an MJCF or URDF file
    profile: str = "articulated"
    base: Optional[str] = None      # base body name; default: the first body
    cameras: list = field(default_factory=list)
    params: dict = field(default_factory=dict)

    def __post_init__(self):
        if self.profile not in PROFILES:
            raise ConfigError(f"unknown robot profile {self.profile!r}.{suggest(self.profile, PROFILES)} Choose one of: {', '.join(PROFILES)}")
        if not Path(self.model).is_file():
            raise FileNotFoundError(f"robot model not found: {self.model}")

    @classmethod
    def zoo_names(cls) -> dict:
        return yaml.safe_load((ZOO / "zoo.yaml").read_text())

    @classmethod
    def from_zoo(cls, name: str, **overrides) -> "Robot":
        zoo = cls.zoo_names()
        if name not in zoo:
            raise ConfigError(f"no robot named {name!r} in the zoo.{suggest(name, zoo)} Available: {', '.join(zoo)}. "
                              f"For your own model use {{mjcf: file.xml}} or {{urdf: file.urdf}}.")
        spec = dict(zoo[name])
        spec.pop("summary", None)
        params = {**spec.pop("params", {}), **overrides.pop("params", {})}
        return cls(name=name, model=str(ZOO / spec.pop("model")), params=params, **{**spec, **overrides})

    @classmethod
    def from_mjcf(cls, path, profile: str = "articulated", name: Optional[str] = None, **kw) -> "Robot":
        return cls(name=name or Path(path).stem, model=str(path), profile=profile, **kw)

    @classmethod
    def from_urdf(cls, path, profile: str = "articulated", name: Optional[str] = None, **kw) -> "Robot":
        """Load a URDF. Mesh paths must be resolvable from the URDF's folder
        (add `<mujoco><compiler meshdir=.../></mujoco>` to the URDF if not)."""
        return cls(name=name or Path(path).stem, model=str(path), profile=profile, **kw)

    @classmethod
    def from_spec(cls, spec) -> "Robot":
        """Build from a config value: a zoo name, or a mapping with `zoo`, `mjcf` or `urdf`."""
        if isinstance(spec, Robot):
            return spec
        if isinstance(spec, str):
            return cls.from_zoo(spec)
        if not isinstance(spec, dict):
            raise ConfigError(f"robot must be a zoo name or a mapping with zoo, mjcf or urdf; got {spec!r}")
        spec = dict(spec)
        allowed = ["zoo", "mjcf", "urdf", "profile", "name", "base", "cameras", "params"]
        for k in spec:
            if k not in allowed:
                raise ConfigError(f"robot has no setting {k!r}.{suggest(k, allowed)} Its settings: {', '.join(allowed)}.")
        if "zoo" in spec:
            return cls.from_zoo(spec.pop("zoo"), **spec)
        if "mjcf" in spec:
            return cls.from_mjcf(spec.pop("mjcf"), **spec)
        if "urdf" in spec:
            return cls.from_urdf(spec.pop("urdf"), **spec)
        raise ConfigError("robot needs one of: zoo, mjcf, urdf")
