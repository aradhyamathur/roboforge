"""The extension points: what a policy, renderer, solver or estimator of your own gets for free."""
import numpy as np
import pytest

pytest.importorskip("mujoco")
import roboforge as rf  # noqa: E402
from roboforge.env import box, to_numpy  # noqa: E402


class FakeTensor:
    """What a PyTorch tensor on a GPU that requires grad looks like to to_numpy."""
    def __init__(self, a):
        self.a, self.calls = np.asarray(a), []

    def detach(self):
        self.calls.append("detach")
        return self

    def cpu(self):
        self.calls.append("cpu")
        return self

    def numpy(self):
        return self.a


class Reach(rf.Policy):
    output = "joint"

    def observe(self, obs):
        return np.concatenate([obs.joint.position, obs["task/goal"]])

    def act(self, x):
        return FakeTensor(x[3:6])                          # "the network" returns a tensor


def arm_app(**kw):
    app = rf.App(robot="arm", controllers=[rf.JointPD(rate=500)], realtime=False, **kw)
    app.setup()
    return app


def test_to_numpy_and_box():
    t = FakeTensor([1, 2])
    assert to_numpy(t).tolist() == [1.0, 2.0] and t.calls == ["detach", "cpu"]
    b = box([[-1, 1], [0, np.inf]])
    assert b.shape == (2,) and b.high[1] == np.inf


def test_policy_waits_for_its_inputs_then_drives_the_arm():
    pol = Reach(rate=20)
    app = rf.App(robot="arm", controllers=[pol, rf.JointPD(rate=500)], realtime=False)
    app.run(duration=0.5)                                  # no goal yet: the policy waits, nothing breaks
    assert app.bus.latest("cmd/joint") is None
    app.bus.publish("task/goal", np.array([0.4, 0.3, -0.6]))
    app.run(duration=2.0)
    q = app.bus.latest("state/joint").position
    app.close()
    assert np.abs(q - [0.4, 0.3, -0.6]).max() < 0.03


def test_policy_reports_a_signal_that_never_comes():
    app = rf.App(robot="arm", controllers=[Reach(rate=500), rf.JointPD(rate=500)], realtime=False)
    with pytest.raises(KeyError) as e:
        app.run(duration=3.0)
    app.close()
    assert any("1000 calls" in n for n in e.value.__notes__)


def test_policy_env_has_the_right_spaces_and_goals_per_episode():
    app = arm_app()
    app.bus.publish("task/goal", np.zeros(3))
    env = Reach(rate=20).make_env(app, reward=lambda o: -float(np.abs(o["task/goal"] - o.joint.position).sum()), max_steps=5)
    assert env.steps_per_action == 25
    assert env.action_space.shape == (3,) and np.allclose(env.action_space.high, np.radians([170, 100, 140]), atol=1e-5)
    x, _ = env.reset(options={"publish": {"task/goal": np.array([0.1, 0.2, 0.3])}})
    assert np.allclose(x[3:6], [0.1, 0.2, 0.3])
    x, r, *_ = env.step(FakeTensor([0.1, 0.2, 0.3]))
    assert np.isfinite(r)
    with pytest.raises(ValueError, match="nan"):
        env.step(np.array([np.nan, 0, 0]))
    env.close()


def test_own_act_needs_an_action_space():
    with pytest.raises(ValueError, match="action_space"):
        arm_app().as_env(act=lambda a: rf.JointCommand(position=a))


class Flat(rf.Renderer):
    """The smallest renderer: implements render() only."""
    name = "flat"

    def setup(self, app):
        super().setup(app)
        self.physics = app.physics

    def render(self, camera):
        pose, fovy = self.physics.camera(camera)
        return self.finish(rf.Image(data=np.full((4, 6, 3), 7, np.uint8), camera=camera, pose=pose, fovy=fovy, t=self.physics.time))


def test_a_renderer_only_implements_render():
    r = Flat(fps=20)
    seen = []
    r.add_pass(lambda img, rr: seen.append(img.t))
    app = rf.App(robot="arm", renderer=r, callbacks=[rf.Recorder(dir="/tmp/rf-test-flat", rate=10)], realtime=False)
    app.run(duration=1.0)
    img = app.bus.latest("sensor/camera/overview")
    app.close()
    assert r.cameras == ["overview"] and len(seen) in (20, 21) and img.data.shape == (4, 6, 3)      # 20 fps, both ends of the second
    assert img.fovy == 50.0 and np.allclose(img.pose[:3, 3], [1.3, -1.1, 0.9])
    assert len(rf.Dataset("/tmp/rf-test-flat").frames()) == len(seen), "frames of any renderer are recorded"
    with pytest.raises(rf.ConfigError, match="Its cameras: overview"):
        app2 = rf.App(robot="arm", renderer=Flat(cameras="nope"), realtime=False)
        app2.run(steps=2)


class Spring(rf.Physics):
    """The smallest solver: a mass on a spring with a motor."""
    name = "spring"

    def setup(self, app):
        self.bus, self.dt = app.bus, 0.001
        self.joint_names = self.actuator_names = ["x"]
        self.u = 0.0
        self.bus.subscribe("cmd/actuators", lambda n, m: setattr(self, "u", float(m.values[0])))
        self.reset()

    def reset(self):
        self.x, self.v, self.time = 1.0, 0.0, 0.0
        self.publish_state(joint=rf.JointState(names=["x"], position=np.array([self.x]), velocity=np.array([self.v])))

    def step(self):
        self.v += (-4.0 * self.x - 0.5 * self.v + self.u) * self.dt
        self.x += self.v * self.dt
        self.time += self.dt
        self.publish_state(joint=rf.JointState(names=["x"], position=np.array([self.x]), velocity=np.array([self.v])))


def test_a_solver_works_with_the_builtin_controllers():
    app = rf.App(physics=Spring(), controllers=[rf.JointPD(kp=40, ki=30, kd=6, target=[0.3])], callbacks=[rf.Checks()], realtime=False)
    app.run(duration=5.0)
    js, clock = app.bus.latest("state/joint"), app.bus.latest("clock")
    app.close()
    assert abs(js.position[0] - 0.3) < 0.01
    assert abs(js.t - 5.0) < 1e-6 and abs(clock.t - 5.0) < 1e-6, "publish_state stamps messages with the time"
    with pytest.raises(NotImplementedError, match="no cameras"):
        Spring().camera("front")


def test_recorder_keeps_any_pose_and_own_types(tmp_path):
    from dataclasses import dataclass
    from typing import ClassVar

    @dataclass
    class Odo:
        signal: ClassVar[str] = "sensor/odo"
        v: float = 0.0

        def as_array(self):
            return [self.v, 2 * self.v]

    class Est(rf.Callback):
        def on_step(self, app, t):
            p = app.bus.latest("state/pose")
            app.bus.publish("slam/pose", rf.Pose(position=p.position + 0.1, orientation=p.orientation, t=t))
            app.bus.publish("sensor/odo", Odo(v=t))
            app.bus.publish("raw/array", np.array([t, -t]))

    app = rf.App(robot="arm", callbacks=[Est(), rf.Recorder(dir=str(tmp_path / "r"), rate=50, signals=["state/pose", "slam/pose", "sensor/odo", "raw/array"])],
                 realtime=False)
    app.run(duration=1.0)
    app.close()
    ds = rf.Dataset(tmp_path / "r")
    t, est = ds.trajectory("slam/pose")
    _, gt = ds.trajectory("state/pose")
    assert est.shape == gt.shape == (len(t), 7)
    assert rf.eval.ate(est, gt) == pytest.approx(np.sqrt(3) * 0.1, rel=1e-6)
    assert ds.signals()["sensor/odo"].shape[1] == 2 and ds.signals()["raw/array"].shape[1] == 2
    with pytest.raises(ValueError, match="not a pose signal"):
        ds.trajectory("sensor/odo")


def test_eval_metrics_and_tum_round_trip(tmp_path):
    rng = np.random.default_rng(0)
    gt = np.cumsum(rng.normal(0, 0.1, (200, 3)), axis=0)
    th = 0.3
    R = np.array([[np.cos(th), -np.sin(th), 0], [np.sin(th), np.cos(th), 0], [0, 0, 1]])
    est = 2.0 * gt @ R.T + [1, 2, 3]                      # rotated, scaled and shifted copy
    assert rf.eval.ate(est, gt) > 1.0
    assert rf.eval.ate(est, gt, align_first=True, scale=True) < 1e-9
    assert rf.eval.rpe(gt + 5.0, gt) < 1e-12              # a constant offset has no relative error
    i, j = rf.eval.associate([0.0, 0.1, 0.5], [0.0, 0.1, 0.2], max_dt=0.02)
    assert i.tolist() == [0, 1] and j.tolist() == [0, 1]
    poses = np.hstack([gt, np.tile([np.cos(0.2), 0, 0, np.sin(0.2)], (200, 1))])
    path = rf.eval.write_tum(tmp_path / "t.txt", np.arange(200) * 0.05, poses)
    t2, p2 = rf.eval.read_tum(path)
    assert np.allclose(p2, poses, atol=1e-6) and np.allclose(t2, np.arange(200) * 0.05)
    with pytest.raises(ValueError, match="associate"):
        rf.eval.ate(gt[:10], gt)


class Stepper(Reach):
    """A network in [-1, 1], decoded into a step from the current pose."""
    def act(self, x):
        return np.clip((x[3:6] - x[:3]) / 0.2, -1, 1)

    def decode(self, a, obs):
        return obs.joint.position + 0.2 * np.clip(a, -1, 1)


def test_decode_runs_in_training_and_deployment_and_sets_a_unit_action_space():
    app = arm_app()
    goals = []
    env = Stepper(rate=20).make_env(app, max_steps=30, on_reset=lambda rng: {"task/goal": goals.append(rng.uniform(-0.5, 0.5, 3)) or goals[-1]})
    assert np.allclose(env.action_space.low, -1) and np.allclose(env.action_space.high, 1)
    x, _ = env.reset(seed=3)
    assert np.allclose(x[3:6], goals[-1]), "on_reset's goal is in the first observation"
    pol = Stepper(rate=20)
    for _ in range(30):
        x, *_ = env.step(pol.act(x))
    assert np.abs(x[:3] - goals[-1]).max() < 0.05, "decode turned unit actions into steps towards the goal"
    env.reset(seed=3)
    assert np.allclose(goals[-1], goals[-2]), "seeded resets draw the same goal"
    env.close()
    deployed = rf.App(robot="arm", controllers=[Stepper(rate=20), rf.JointPD(rate=500)], realtime=False)
    deployed.setup()
    deployed.bus.publish("task/goal", np.array([0.3, 0.2, -0.4]))
    deployed.run(duration=2.0)
    assert np.abs(deployed.bus.latest("state/joint").position - [0.3, 0.2, -0.4]).max() < 0.03
    deployed.close()


def test_env_explains_a_missing_signal_before_the_first_episode():
    with pytest.raises(ValueError, match="on_reset"):
        Reach(rate=20).make_env(arm_app())
