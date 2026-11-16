"""C++ controllers under load: long runs and many load/unload cycles must not leak."""
import pathlib
import shutil
import tempfile

import numpy as np

import roboforge as rf
from harness import fds, rss_mb, scenario, threads

PID = pathlib.Path(__file__).resolve().parents[2] / "examples" / "cpp_controller" / "arm_pid.cpp"


def ctl(**kw):
    from roboforge.cpp import CppController
    return CppController(source=str(PID), rate=500, inputs=["cmd/joint"], **kw)


@scenario(timeout=300, needs=("mujoco",))
def cpp_long_run_and_reload_cycles():
    if not shutil.which("g++") and not shutil.which("clang++"):
        return "skipped: no compiler"
    d = pathlib.Path(tempfile.mkdtemp())
    shutil.copy(PID, d / "arm_pid.cpp")
    globals()["PID"] = d / "arm_pid.cpp"
    wave = rf.controller(lambda o, t: rf.JointCommand(position=[0.8 * np.sin(0.6 * t), 0.5, -0.9]), rate=50)
    app = rf.App(robot="arm", controllers=[wave, ctl()], realtime=False)
    app.run(duration=30)
    m0 = rss_mb()
    app.run(duration=270)                                   # 135 000 C++ calls
    grew = rss_mb() - m0
    q = app.bus.latest("state/joint").position
    calls = app.controllers[1].calls
    app.close()
    assert np.all(np.isfinite(q)) and grew < 10, f"memory grew {grew:.1f} MB over {calls} calls"
    for i in range(20):                                     # settle
        a = rf.App(robot="arm", controllers=[ctl(kp=40 + i)], realtime=False)
        a.run(steps=50)
        a.close()
    m1, th0, fd0 = rss_mb(), threads(), fds()
    for i in range(200):
        a = rf.App(robot="arm", controllers=[ctl(kp=40 + i % 7)], realtime=False)
        a.run(steps=50)
        a.close()
    grew2, th, fd = rss_mb() - m1, threads() - th0, fds() - fd0
    assert grew2 < 15 and th <= 0 and fd <= 1, f"200 load/unload cycles: +{grew2:.1f} MB, {th:+d} threads, {fd:+d} fds"
    return f"{calls} calls, +{grew:.1f} MB; 200 reloads +{grew2:.1f} MB, {th:+d} threads, {fd:+d} fds"
