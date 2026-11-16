"""Run the arm with the C++ PID and with the built-in Python one, and compare.

They implement the same loop, so the joint trajectories should match to the
last digit. For a loop this small the time per call is mostly the hand-over
between Python and C++ (about ten microseconds), not the control law; C++
pays off when the law itself is heavy (an MPC, a filter, a network).

    python3 compare.py
"""
import pathlib
import time

import numpy as np

import roboforge as rf

HERE = pathlib.Path(__file__).resolve().parent


def run(low_level, seconds=5.0):
    trace = []
    log = rf.controller(lambda obs, t: trace.append(obs.joint.position.copy()), rate=100, name="log")
    app = rf.App.from_config(HERE / "app.yaml", realtime=False)
    if low_level == "python":
        app.controllers[1] = rf.JointPD(rate=500, kp=60, ki=20, kd=6)
    app.controllers.append(log)
    app.setup()
    pid = app.controllers[1]
    calls, spent = 0, 0.0
    inner = pid.step

    def timed(obs, t):
        nonlocal calls, spent
        t0 = time.perf_counter()
        out = inner(obs, t)
        spent += time.perf_counter() - t0
        calls += 1
        return out
    pid.step = timed
    app.run(duration=seconds)
    app.close()
    native = 1e6 * pid.native_time / pid.calls if hasattr(pid, "native_time") else None
    return np.array(trace), 1e6 * spent / calls, native, pid.name or type(pid).__name__


def main():
    cpp, us_cpp, native, name = run("cpp")
    py, us_py, _, _ = run("python")
    diff = np.abs(cpp - py).max()
    print(f"{'low-level controller':<22} {'per call':>10}")
    print(f"{'C++ (' + name + ')':<22} {us_cpp:8.1f} us   of which {native:.1f} us in C++; the rest is handing the data across")
    print(f"{'Python (JointPD)':<22} {us_py:8.1f} us")
    print(f"\nfinal joints: C++ {np.round(cpp[-1], 4)}  Python {np.round(py[-1], 4)}")
    print(f"largest difference over {len(cpp)} samples: {diff:.2e} rad -> {'identical' if diff < 1e-9 else 'DIFFERENT'}")


if __name__ == "__main__":
    main()
