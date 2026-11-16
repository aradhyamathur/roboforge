"""Controllers written in C++.

    controllers:
      - {type: cpp, source: arm_pid.cpp, rate: 500, kp: 60, inputs: [cmd/joint]}

`source` is compiled into a shared library the first time the app runs (and
again whenever it, or the SDK's header, changes); `library` names one you
built yourself, with CMake for example. Every other key is a parameter the
C++ constructor reads with `rf::Params`. `inputs` lists the signals, beyond
the standard state, that the controller reads with `obs.signal(name)`.

The header is `roboforge/controller.hpp`; `python -m roboforge cpp include`
prints the folder to add to a compiler's include path, and `python -m
roboforge cpp new NAME` writes a starting point.
"""
from __future__ import annotations

import ctypes
import hashlib
import os
import shlex
import shutil
import subprocess
from pathlib import Path
from time import perf_counter as _clock

import numpy as np

from .. import msgs
from ..control import Controller
from ..errors import ConfigError

HERE = Path(__file__).resolve().parent
INCLUDE = HERE / "include"
HEADER = INCLUDE / "roboforge" / "controller.hpp"
ABI_VERSION = 1
FLAGS = ["-std=c++17", "-O2", "-shared", "-fPIC", "-Wall"]


def include_dir() -> Path:
    """The folder to put on a C++ compiler's include path (`-I`)."""
    return INCLUDE


def compiler() -> str:
    for c in ([os.environ["CXX"]] if os.environ.get("CXX") else []) + ["g++", "clang++", "c++"]:
        path = shutil.which(shlex.split(c)[0])
        if path:
            return c
    raise ConfigError("no C++ compiler found (tried $CXX, g++, clang++, c++). Install one (on Ubuntu: sudo apt install g++), "
                      "or build the library yourself and give `library:` instead of `source:`.")


def build(source, out_dir=None, flags=(), quiet: bool = False) -> Path:
    """Compile `source` into a shared library and return its path.

    The library's name carries a hash of the source, the SDK header, the
    compiler and the flags, so an unchanged controller is not rebuilt and a
    changed one never collides with an old build already loaded in this
    process. Older builds of the same source are removed.
    """
    source = Path(source).resolve()
    if not source.is_file():
        raise ConfigError(f"C++ source not found: {source}")
    cxx = compiler()
    flags = list(FLAGS) + list(flags)
    digest = hashlib.sha256()
    for part in (source.read_bytes(), HEADER.read_bytes(), cxx.encode(), " ".join(flags).encode()):
        digest.update(part)
    out_dir = Path(out_dir) if out_dir else source.parent / "build"
    out = out_dir / f"lib{source.stem}-{digest.hexdigest()[:12]}.so"
    if out.is_file():
        return out
    out_dir.mkdir(parents=True, exist_ok=True)
    cmd = shlex.split(cxx) + flags + ["-I", str(INCLUDE), "-o", str(out) + ".tmp", str(source)]
    if not quiet:
        print(f"roboforge: compiling {source.name} ...", flush=True)
    p = subprocess.run(cmd, capture_output=True, text=True)
    if p.returncode != 0:
        lines = [l for l in p.stderr.splitlines() if l.strip()]
        shown = "\n".join(lines[:25]) + ("\n..." if len(lines) > 25 else "")
        raise ConfigError(f"{source.name} did not compile:\n{shown}\n(command: {' '.join(shlex.quote(c) for c in cmd)})")
    os.replace(str(out) + ".tmp", out)
    for old in out_dir.glob(f"lib{source.stem}-*.so"):
        if old != out:
            try:
                old.unlink()
            except OSError:
                pass
    return out


# ---------------------------------------------------------------------------- ABI
class _Param(ctypes.Structure):
    _fields_ = [("name", ctypes.c_char_p), ("n", ctypes.c_int), ("values", ctypes.POINTER(ctypes.c_double)), ("text", ctypes.c_char_p)]


class _Signal(ctypes.Structure):
    _fields_ = [("name", ctypes.c_char_p), ("n", ctypes.c_int), ("values", ctypes.POINTER(ctypes.c_double))]


_D3, _D4 = ctypes.c_double * 3, ctypes.c_double * 4


class _Obs(ctypes.Structure):
    _fields_ = [("t", ctypes.c_double), ("n_joints", ctypes.c_int), ("q", ctypes.POINTER(ctypes.c_double)),
                ("qd", ctypes.POINTER(ctypes.c_double)), ("effort", ctypes.POINTER(ctypes.c_double)),
                ("has_pose", ctypes.c_int), ("position", _D3), ("orientation", _D4), ("linear", _D3), ("angular", _D3),
                ("has_imu", ctypes.c_int), ("accel", _D3), ("gyro", _D3),
                ("has_status", ctypes.c_int), ("armed", ctypes.c_int),
                ("n_signals", ctypes.c_int), ("signals", ctypes.POINTER(_Signal))]


class _Action(ctypes.Structure):
    _fields_ = [("kind", ctypes.c_int), ("n", ctypes.c_int), ("values", ctypes.c_double * 256)]


class _Info(ctypes.Structure):
    _fields_ = [("dt", ctypes.c_double), ("n_joints", ctypes.c_int), ("joint_names", ctypes.POINTER(ctypes.c_char_p)),
                ("n_actuators", ctypes.c_int), ("actuator_names", ctypes.POINTER(ctypes.c_char_p)),
                ("ctrl_low", ctypes.POINTER(ctypes.c_double)), ("ctrl_high", ctypes.POINTER(ctypes.c_double))]


def _ptr(a: np.ndarray):
    return a.ctypes.data_as(ctypes.POINTER(ctypes.c_double))


def _ptr_row(a: np.ndarray, i: int):
    return ctypes.cast(a.ctypes.data + i * a.strides[0], ctypes.POINTER(ctypes.c_double))


class CppController(Controller):
    """A controller compiled from C++ (see the module docstring)."""

    def __init__(self, source=None, library=None, rate=None, name: str = "", inputs=(), cxxflags=(), **params):
        super().__init__(rate=rate, name=name)
        if (source is None) == (library is None):
            raise ConfigError("a cpp controller needs either `source:` (a .cpp file, built for you) or `library:` (a .so you built)")
        self.source, self.library = source, library
        self.inputs = [inputs] if isinstance(inputs, str) else list(inputs)
        self.cxxflags = shlex.split(cxxflags) if isinstance(cxxflags, str) else list(cxxflags)
        self.params = params
        self._h = self._lib = None
        self._err = ctypes.create_string_buffer(1024)
        self.native_time, self.calls = 0.0, 0     # seconds spent inside the C++ step, and how many steps

    # --- loading ----------------------------------------------------------------
    def _load(self):
        path = Path(self.library) if self.library else build(self.source, flags=self.cxxflags)
        if not path.is_file():
            raise ConfigError(f"C++ controller library not found: {path}. Build it first, or give `source:` to have it built.")
        try:
            lib = ctypes.CDLL(str(path.resolve()), mode=os.RTLD_LOCAL | os.RTLD_NOW)
        except OSError as e:
            raise ConfigError(f"could not load {path.name}: {e}") from None
        missing = [s for s in ("rf_abi_version", "rf_create", "rf_setup", "rf_step", "rf_reset", "rf_destroy") if not hasattr(lib, s)]
        if missing:
            raise ConfigError(f"{path.name} is not a RoboForge controller (no {', '.join(missing)}). "
                              f"Add RF_EXPORT_CONTROLLER(YourClass) at the end of the source.")
        version = lib.rf_abi_version()
        if version != ABI_VERSION:
            raise ConfigError(f"{path.name} was built against controller.hpp version {version}; this SDK speaks version {ABI_VERSION}. Rebuild it.")
        lib.rf_create.restype = ctypes.c_void_p
        lib.rf_create.argtypes = [ctypes.c_int, ctypes.POINTER(_Param), ctypes.c_char_p, ctypes.c_int]
        lib.rf_setup.argtypes = [ctypes.c_void_p, ctypes.POINTER(_Info), ctypes.c_char_p, ctypes.c_int]
        lib.rf_step.argtypes = [ctypes.c_void_p, ctypes.POINTER(_Obs), ctypes.POINTER(_Action), ctypes.c_char_p, ctypes.c_int]
        lib.rf_reset.argtypes = lib.rf_destroy.argtypes = [ctypes.c_void_p]
        for f in ("rf_unused_params", "rf_read_params", "rf_class_name"):
            if hasattr(lib, f):
                getattr(lib, f).restype = ctypes.c_char_p
        if hasattr(lib, "rf_unused_params"):
            lib.rf_unused_params.argtypes = lib.rf_read_params.argtypes = [ctypes.c_void_p]
        self._lib, self._path = lib, path
        if not self.name:
            self.name = lib.rf_class_name().decode() if hasattr(lib, "rf_class_name") else path.stem

    def _create(self):
        keep, items = [], []
        for k, v in self.params.items():
            if isinstance(v, str):
                items.append(_Param(k.encode(), 0, None, v.encode()))
                continue
            try:
                a = np.ascontiguousarray(np.asarray(v, float).reshape(-1))
            except (TypeError, ValueError):
                raise ConfigError(f"C++ controller parameter {k!r} must be a number, a list of numbers or text, got {v!r}") from None
            keep.append(a)
            items.append(_Param(k.encode(), len(a), _ptr(a), None))
        arr = (_Param * max(len(items), 1))(*items)
        h = self._lib.rf_create(len(items), arr, self._err, len(self._err))
        if not h:
            raise ConfigError(f"C++ controller {self.name} could not be created: {self._err.value.decode(errors='replace')}")
        unused = [u for u in self._lib.rf_unused_params(h).decode().split(",") if u] if hasattr(self._lib, "rf_unused_params") else []
        if unused:
            read = [r for r in self._lib.rf_read_params(h).decode().split(",") if r]
            self._lib.rf_destroy(h)
            from ..errors import suggest
            hints = "".join(suggest(u, read) for u in unused)
            raise ConfigError(f"C++ controller {self.name} does not read parameter {', '.join(map(repr, unused))}.{hints} "
                              f"It reads: {', '.join(read) or 'none'}.")
        self._h = h

    def setup(self, app):
        self._load()
        self._create()
        p = app.physics
        self._bus = app.bus
        jn = [n.encode() for n in getattr(p, "joint_names", [])]
        an = [n.encode() for n in getattr(p, "actuator_names", [])]
        rng = np.asarray(getattr(p, "ctrl_range", np.zeros((len(an), 2))), float).reshape(-1, 2)
        self._low, self._high = np.ascontiguousarray(rng[:, 0]), np.ascontiguousarray(rng[:, 1])
        self._jn, self._an = (ctypes.c_char_p * max(len(jn), 1))(*jn), (ctypes.c_char_p * max(len(an), 1))(*an)
        info = _Info(float(p.dt), len(jn), self._jn, len(an), self._an, _ptr(self._low), _ptr(self._high))
        if self._lib.rf_setup(self._h, ctypes.byref(info), self._err, len(self._err)) != 0:
            raise RuntimeError(f"C++ controller {self.name} failed in setup(): {self._err.value.decode(errors='replace')}")
        # Everything passed to C++ lives in buffers made once here and refilled in place each step:
        # allocating per step would cost more than a typical control law.
        self._obs, self._act = _Obs(), _Action()
        self._obs_ref, self._act_ref = ctypes.byref(self._obs), ctypes.byref(self._act)
        n = len(jn)
        self._jbuf = np.zeros((3, max(n, 1)))
        self._obs.q, self._obs.qd, self._obs.effort = (_ptr_row(self._jbuf, i) for i in range(3))
        self._pos, self._quat = np.ctypeslib.as_array(self._obs.position), np.ctypeslib.as_array(self._obs.orientation)
        self._lin, self._ang = np.ctypeslib.as_array(self._obs.linear), np.ctypeslib.as_array(self._obs.angular)
        self._acc, self._gyro = np.ctypeslib.as_array(self._obs.accel), np.ctypeslib.as_array(self._obs.gyro)
        self._actv = np.ctypeslib.as_array(self._act.values)
        self._sig = (_Signal * max(len(self.inputs), 1))()
        self._sigbuf = [np.zeros(16) for _ in self.inputs]
        for i, name in enumerate(self.inputs):
            self._sig[i].name = name.encode()
        self._obs.signals = self._sig
        from ..callbacks import as_row
        self._as_row = as_row

    # --- stepping ---------------------------------------------------------------
    def step(self, obs, t):
        o = self._obs
        o.t = t
        js = obs.joint
        if js is not None:
            n = len(js.position)
            if n > self._jbuf.shape[1]:
                raise ValueError(f"C++ controller {self.name}: state/joint has {n} joints, more than the {self._jbuf.shape[1]} the robot declared")
            self._jbuf[0, :n] = js.position
            self._jbuf[1, :n] = js.velocity if len(js.velocity) == n else 0.0
            self._jbuf[2, :n] = js.effort if len(js.effort) == n else 0.0
            o.n_joints = n
        else:
            o.n_joints = 0
        pose, twist, imu, st = obs.pose, obs.twist, obs.imu, obs.status
        o.has_pose = pose is not None
        if pose is not None:
            self._pos[:] = pose.position
            self._quat[:] = pose.orientation
        if twist is not None:
            self._lin[:] = twist.linear
            self._ang[:] = twist.angular
        o.has_imu = imu is not None
        if imu is not None:
            self._acc[:] = imu.accel
            self._gyro[:] = imu.gyro
        o.has_status = st is not None
        o.armed = bool(st.armed) if st is not None else False
        k = 0
        for i, name in enumerate(self.inputs):
            m = obs.get(name)
            row = None if m is None else self._as_row(m)
            if row is None:
                continue
            if len(row) > len(self._sigbuf[i]):
                self._sigbuf[i] = np.zeros(2 * len(row))
            self._sigbuf[i][:len(row)] = row
            sig = self._sig[k]
            if k != i:
                sig.name = self._sig[i].name
            sig.n, sig.values = len(row), _ptr(self._sigbuf[i])
            k += 1
        o.n_signals = k
        t0 = _clock()
        failed = self._lib.rf_step(self._h, self._obs_ref, self._act_ref, self._err, 1024) != 0
        self.native_time += _clock() - t0
        self.calls += 1
        if failed:
            raise RuntimeError(f"C++ controller {self.name} threw: {self._err.value.decode(errors='replace')}")
        return self._message()

    def _message(self):
        a = self._act
        kind, n = a.kind, a.n
        if kind == 0:
            return None
        v = self._actv[:n].copy()
        if kind == 1:
            return msgs.ActuatorCommand(values=v)
        if kind == 2:
            return msgs.JointCommand(position=v)
        if kind == 3:
            if n != 6:
                raise ValueError(f"C++ controller {self.name}: a velocity action has 6 values, got {n}")
            return msgs.velocity_command(v[:3], v[3:])
        if kind == 4:
            if n != 4:
                raise ValueError(f"C++ controller {self.name}: a setpoint action has 4 values (x, y, z, yaw), got {n}")
            return msgs.Setpoint(position=v[:3], yaw=float(v[3]))
        raise ValueError(f"C++ controller {self.name} returned an unknown action kind {kind}")

    def reset(self):
        if self._h:
            self._lib.rf_reset(self._h)

    def close(self):
        if self._h:
            self._lib.rf_destroy(self._h)
            self._h = None

    def describe(self):
        return {"type": f"cpp:{self.name}", "library": str(getattr(self, "_path", self.library or self.source))}


TEMPLATE = '''// A RoboForge controller in C++. Build and run it from app.yaml:
//
//   controllers:
//     - {{type: cpp, source: {file}, rate: 500, kp: 40, kd: 4}}
//
// Header: roboforge/controller.hpp (python3 -m roboforge cpp include).
#include <roboforge/controller.hpp>

class {cls} : public rf::Controller {{
 public:
  // Parameters come from the controller's line in app.yaml.
  explicit {cls}(const rf::Params& p) : kp_(p.number("kp", 40.0)), kd_(p.number("kd", 4.0)) {{}}

  // Once, before the first step: what is being controlled.
  void setup(const rf::Info& info) override {{ low_ = info.ctrl_low; high_ = info.ctrl_high; }}

  // At `rate` Hz: read the robot, return a command.
  rf::Action step(const rf::Obs& obs) override {{
    rf::Vec u(obs.joints());
    for (int i = 0; i < obs.joints(); ++i)
      u[i] = rf::clamp(-kp_ * obs.q(i) - kd_ * obs.qd(i), low_[i], high_[i]);   // hold every joint at 0
    return rf::Action::actuators(u);
  }}

 private:
  double kp_, kd_;
  rf::Vec low_, high_;
}};

RF_EXPORT_CONTROLLER({cls})
'''


def new(name: str, folder=".") -> Path:
    """Write a starting controller, NAME.cpp, and return its path."""
    stem = Path(name).stem
    cls = "".join(w.capitalize() for w in stem.replace("-", "_").split("_")) or "MyController"
    path = Path(folder) / f"{stem}.cpp"
    if path.exists():
        raise ConfigError(f"{path} exists already")
    path.write_text(TEMPLATE.format(file=path.name, cls=cls))
    return path
