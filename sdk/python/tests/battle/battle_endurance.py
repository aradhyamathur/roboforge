"""Long runs, many restarts, determinism, pacing. These find leaks, drift and
anything that only goes wrong after a while."""
import time

import numpy as np

import roboforge as rf
from harness import fds, rss_mb, scenario, threads

BUILTIN = {"quadrotor": lambda: [rf.MultirotorController(rate=250, hover=[0, 0, 1.5])],
           "arm": lambda: [rf.JointPD(rate=500, target=[0.5, 0.4, -0.8])],
           "diffdrive": lambda: [rf.controller(lambda o, t: rf.velocity_command((0.3, 0, 0), (0, 0, 0.5)), rate=10), rf.DiffDriveController()]}


def state(app):
    p = app.bus.latest("state/pose")
    j = app.bus.latest("state/joint")
    return np.concatenate([p.position, p.orientation, j.position if j is not None else []])


@scenario(timeout=300)
def endurance_ten_minutes_each_robot():
    """600 s of simulated time per robot: still stable, still where it should be, memory flat."""
    out = []
    for name, ctrl in BUILTIN.items():
        app = rf.App(robot=name, controllers=ctrl(), realtime=False)
        app.run(duration=60)
        m0, t0 = rss_mb(), time.time()
        app.run(duration=540)
        grew, wall = rss_mb() - m0, time.time() - t0
        s = state(app)
        assert np.all(np.isfinite(s)), f"{name}: state is not finite after 600 s"
        assert abs(app.time - 600.0) < 1e-6, f"{name}: clock drifted to {app.time!r}"
        if name == "quadrotor":
            assert np.linalg.norm(s[:3] - [0, 0, 1.5]) < 0.05, f"quadrotor wandered to {s[:3]}"
        if name == "arm":
            assert np.abs(s[7:] - [0.5, 0.4, -0.8]).max() < 0.02, f"arm settled at {s[7:]}"
        if name == "diffdrive":
            assert np.linalg.norm(s[:2]) < 1.5 and abs(s[2]) < 0.2, f"base left its circle: {s[:3]}"
        assert grew < 20, f"{name}: memory grew {grew:.0f} MB over 540 s of simulation"
        out.append(f"{name} x{540 / wall:.0f} realtime, +{grew:.1f} MB")
        app.close()
    return "; ".join(out)


@scenario(timeout=120)
def endurance_same_inputs_same_result():
    """Two runs of the same app end in exactly the same state (bitwise), also after reset()."""
    out = []
    for name, ctrl in BUILTIN.items():
        ends = []
        for _ in range(2):
            app = rf.App(robot=name, controllers=ctrl(), realtime=False)
            app.run(duration=20)
            ends.append(state(app))
            app.reset()
            app.run(duration=20)
            ends.append(state(app))
            app.close()
        for e in ends[1:]:
            assert np.array_equal(ends[0], e), f"{name}: runs differ by {np.abs(ends[0] - e).max():.3e} (run, reset+run, new app...)"
        out.append(name)
    return "identical: " + ", ".join(out)


@scenario(timeout=240)
def endurance_open_and_close_300_apps():
    """Build, run and close many apps in one process: no growth in memory, threads or file descriptors."""
    def one(i):
        app = rf.App(robot=["quadrotor", "arm", "diffdrive"][i % 3], renderer=rf.MujocoRenderer(width=64, height=48, fps=20),
                     controllers=BUILTIN[["quadrotor", "arm", "diffdrive"][i % 3]](), callbacks=[rf.RateMonitor()], realtime=False)
        app.run(steps=60)
        app.close()
    for i in range(150):                # memory settles during the first apps (allocator pools, GL state)
        one(i)
    m0, th0, fd0 = rss_mb(), threads(), fds()
    for i in range(300):
        one(i)
    grew, th, fd = rss_mb() - m0, threads() - th0, fds() - fd0
    assert th <= 0, f"{th} threads were left behind"
    assert fd <= 2, f"{fd} file descriptors were left open"
    assert grew < 30, f"memory grew {grew:.0f} MB over 300 apps after settling"
    return f"+{grew:.1f} MB over 300 apps after 150 to settle, {th:+d} threads, {fd:+d} fds"


@scenario(timeout=120)
def endurance_reset_2000_times():
    app = rf.App(robot="quadrotor", controllers=BUILTIN["quadrotor"](), realtime=False)
    app.run(steps=50)
    first = None
    m0 = rss_mb()
    for i in range(2000):
        app.reset()
        assert app.time == 0.0, f"reset {i} left the clock at {app.time}"
        app.run(steps=20)
        s = state(app)
        first = s if first is None else first
        assert np.array_equal(s, first), f"reset {i} did not reproduce the first episode"
    grew = rss_mb() - m0
    assert grew < 20, f"memory grew {grew:.0f} MB over 2000 resets"
    app.close()
    return f"+{grew:.1f} MB"


@scenario(timeout=120)
def endurance_realtime_pacing():
    """Real-time mode tracks the wall clock, at speed 1 and 2, and does not sprint after a stall."""
    out = []
    for speed in (1.0, 2.0, 0.5):
        app = rf.App(robot="arm", controllers=BUILTIN["arm"](), speed=speed)
        app.setup()
        t0 = time.monotonic()
        app.run(duration=4 * speed)
        wall = time.monotonic() - t0
        assert abs(wall - 4.0) < 0.15, f"speed {speed}: 4 s of wall time expected, took {wall:.2f}"
        out.append(f"x{speed}: {wall:.2f}s")
        app.close()
    stall = {"done": False}

    def slow(obs, t):
        if t > 1.0 and not stall["done"]:
            stall["done"] = True
            time.sleep(2.0)                      # a breakpoint, a page fault, a GC pause
    app = rf.App(robot="arm", controllers=[rf.controller(slow)] + BUILTIN["arm"]())
    t0 = time.monotonic()
    app.run(duration=3.0)
    wall = time.monotonic() - t0
    assert 4.7 < wall < 5.4, f"after a 2 s stall, 3 s of sim should take about 5 s of wall time, took {wall:.2f}"
    app.close()
    return ", ".join(out) + f", stall: {wall:.2f}s"


@scenario(timeout=120)
def endurance_controller_rates_hold():
    """Every controller runs at its declared rate, exactly, including rates that do not divide the physics rate."""
    rates = [500, 250, 100, 50, 30, 7, 1, 0.5]
    counts = {r: 0 for r in rates}

    def make(r):
        def f(obs, t):
            counts[r] += 1
        return rf.controller(f, rate=r)
    app = rf.App(robot="arm", controllers=[make(r) for r in rates], realtime=False)
    app.run(duration=60)
    app.close()
    bad = {r: c for r, c in counts.items() if abs(c - 60 * r) > 1}
    assert not bad, f"expected rate x 60 calls, got {bad}"
    too_fast = rf.App(robot="arm", controllers=[rf.controller(lambda o, t: None, rate=5000)], realtime=False)
    too_fast.run(duration=1)
    r = too_fast.controller_rate(too_fast.controllers[0])
    too_fast.close()
    assert abs(r - 500) < 2, f"a 5000 Hz controller on 500 Hz physics should run every step, measured {r}"
    return f"all exact; 5000 Hz request runs at {r:.0f} Hz"


@scenario(timeout=120)
def endurance_async_policy_survives_abuse():
    """A slow policy that sometimes raises, sometimes returns nothing, sometimes takes far too long."""
    class Flaky(rf.Controller):
        rate = 5
        n = 0

        def step(self, obs, t):
            Flaky.n += 1
            if Flaky.n % 7 == 3:
                raise RuntimeError("model crashed")
            if Flaky.n % 7 == 5:
                time.sleep(0.6)
            if Flaky.n % 7 == 6:
                return None
            return rf.Chunk([rf.JointCommand(position=np.array([0.3, 0.2, -0.5]))] * 5, dt=0.02)
    th0 = threads()
    a = rf.Async(Flaky(), rate=50, timeout=0.3, fallback="hold")
    app = rf.App(robot="arm", controllers=[a, rf.JointPD(rate=500)], speed=1.0)
    app.run(duration=8)
    q = app.bus.latest("state/joint").position
    rate = app.controller_rate(app.controllers[1])
    app.close()
    time.sleep(0.8)
    assert np.all(np.isfinite(q)) and np.abs(q).max() < 2.0, f"arm state {q}"
    assert abs(rate - 500) < 3, f"the PID loop dropped to {rate:.0f} Hz"
    assert threads() <= th0, f"{threads() - th0} worker threads left after close"
    assert a.errors >= 1, "the policy's exceptions were not counted"
    return f"PID {rate:.0f} Hz, {a.errors} policy errors survived, {a.late} late steps"
