"""Controllers in C++: built from source, loaded, stepped, and every way it can go wrong."""
import shutil
import subprocess
import sys
import textwrap

import numpy as np
import pytest

pytest.importorskip("mujoco")
import roboforge as rf  # noqa: E402
from roboforge import cpp  # noqa: E402

pytestmark = pytest.mark.skipif(not any(shutil.which(c) for c in ("g++", "clang++", "c++")), reason="no C++ compiler")


def source(tmp_path, body, name="ctl"):
    f = tmp_path / f"{name}.cpp"
    f.write_text("#include <roboforge/controller.hpp>\n" + textwrap.dedent(body))
    return f


HOLD = """
class Hold : public rf::Controller {
 public:
  explicit Hold(const rf::Params& p) : kp_(p.number("kp", 40.0)), kd_(p.number("kd", 4.0)), goal_(p.numbers("goal", {0.3, 0.2, -0.5})) {}
  rf::Action step(const rf::Obs& obs) override {
    rf::Vec u(obs.joints());
    for (int i = 0; i < obs.joints(); ++i) u[i] = kp_ * (goal_[i] - obs.q(i)) - kd_ * obs.qd(i);
    return rf::Action::actuators(u);
  }
 private:
  double kp_, kd_;
  rf::Vec goal_;
};
RF_EXPORT_CONTROLLER(Hold)
"""


def arm(ctl, **kw):
    app = rf.App(robot="arm", controllers=[ctl], realtime=False, **kw)
    return app


def test_builds_once_and_rebuilds_on_change(tmp_path):
    f = source(tmp_path, HOLD)
    a = cpp.build(f, quiet=True)
    assert a.is_file() and cpp.build(f, quiet=True) == a, "an unchanged source is not rebuilt"
    f.write_text(f.read_text().replace("40.0", "41.0"))
    b = cpp.build(f, quiet=True)
    assert b != a and b.is_file() and not a.exists(), "a changed source gets a new library and the old one goes"


def test_a_cpp_controller_drives_the_arm_with_parameters(tmp_path):
    f = source(tmp_path, HOLD)
    app = arm(rf.CONTROLLERS.build({"type": "cpp", "source": str(f), "rate": 500, "kp": 60, "goal": [0.4, 0.3, -0.6]}))
    app.run(duration=3)
    q = app.bus.latest("state/joint").position
    src = app.bus.describe()
    app.close()
    assert np.abs(q - [0.4, 0.3, -0.6]).max() < 0.05
    assert any(s["name"] == "cmd/actuators" and s["sources"] == ["Hold"] for s in src), "named after the C++ class"


def test_every_action_kind_and_inputs(tmp_path):
    f = source(tmp_path, """
    class Many : public rf::Controller {
     public:
      explicit Many(const rf::Params& p) : kind_(p.text("kind")) {}
      rf::Action step(const rf::Obs& obs) override {
        if (kind_ == "joints") return rf::Action::joints(obs.signal("task/goal"));
        if (kind_ == "velocity") return rf::Action::velocity(0.3, 0.4);
        if (kind_ == "setpoint") return rf::Action::setpoint(1, 2, 3, 0.5);
        return rf::Action::none();
      }
     private:
      std::string kind_;
    };
    RF_EXPORT_CONTROLLER(Many)
    """)
    app = rf.App(robot="arm", controllers=[cpp.CppController(source=str(f), kind="joints", inputs=["task/goal"]), rf.JointPD(rate=500)], realtime=False)
    app.setup()
    app.bus.publish("task/goal", np.array([0.2, 0.1, -0.3]))
    app.run(duration=2)
    assert np.allclose(app.bus.latest("cmd/joint").position, [0.2, 0.1, -0.3])
    app.close()
    base = rf.App(robot="diffdrive", controllers=[cpp.CppController(source=str(f), kind="velocity", rate=10), rf.DiffDriveController()], realtime=False)
    base.run(duration=3)
    assert np.linalg.norm(base.bus.latest("state/pose").position[:2]) > 0.5
    base.close()
    quad = rf.App(robot="quadrotor", controllers=[cpp.CppController(source=str(f), kind="setpoint", rate=1), rf.MultirotorController(rate=250)], realtime=False)
    quad.run(duration=1)
    sp = quad.bus.latest("cmd/setpoint")
    quad.close()
    assert np.allclose(sp.position, [1, 2, 3]) and sp.yaw == 0.5
    none = arm(cpp.CppController(source=str(f), kind="none"))
    none.run(steps=10)
    assert none.bus.latest("cmd/actuators") is None
    none.close()


def test_matches_the_python_joint_controller_exactly(tmp_path):
    import pathlib
    ex = pathlib.Path(__file__).resolve().parents[1] / "examples" / "cpp_controller"
    shutil.copytree(ex, tmp_path / "ex")
    out = subprocess.run([sys.executable, "compare.py"], cwd=tmp_path / "ex", capture_output=True, text=True, timeout=180)
    assert out.returncode == 0, out.stderr[-1500:]
    assert "-> identical" in out.stdout, out.stdout


@pytest.mark.parametrize("case, body, extra, words", [
    ("compile error", "class X : public rf::Controller { oops };\n", {}, ["did not compile", "oops"]),
    ("no export", "class X {};\n", {}, ["not a RoboForge controller", "RF_EXPORT_CONTROLLER"]),
    ("typo in a parameter", HOLD, {"kpp": 3}, ["does not read parameter 'kpp'", "Did you mean 'kp'", "It reads"]),
    ("wrong parameter type", HOLD, {"kp": [1, 2]}, ["could not be created", "'kp' must be one number"]),
    ("constructor throws", HOLD.replace('p.number("kp", 40.0)', 'p.number("required_gain")'), {}, ["required_gain", "is required"]),
])
def test_mistakes_are_reported_clearly(tmp_path, case, body, extra, words):
    f = source(tmp_path, body)
    with pytest.raises(rf.ConfigError) as e:
        app = arm(cpp.CppController(source=str(f), **extra))
        app.setup()
    for w in words:
        assert w in str(e.value), f"{case}: {w!r} not in {e.value}"


def test_exceptions_in_step_are_caught_and_named(tmp_path):
    f = source(tmp_path, """
    class Bad : public rf::Controller {
     public:
      explicit Bad(const rf::Params&) {}
      rf::Action step(const rf::Obs& obs) override {
        if (obs.t() > 0.01) return rf::Action::actuators({obs.q(7)});    // out of range: throws
        return rf::Action::none();
      }
    };
    RF_EXPORT_CONTROLLER(Bad)
    """)
    app = arm(cpp.CppController(source=str(f)))
    with pytest.raises(RuntimeError) as e:
        app.run(steps=100)
    app.close()
    assert "C++ controller Bad threw" in str(e.value) and "joint index 7 out of range" in str(e.value)
    assert any("in controller Bad" in n for n in e.value.__notes__)


def test_wrong_sized_or_nan_output_is_refused(tmp_path):
    f = source(tmp_path, """
    #include <limits>
    class Bad : public rf::Controller {
     public:
      explicit Bad(const rf::Params& p) : nan_(p.number("nan", 0) > 0) {}
      rf::Action step(const rf::Obs&) override {
        if (nan_) return rf::Action::actuators({std::numeric_limits<double>::quiet_NaN(), 0, 0});
        return rf::Action::actuators({1.0, 2.0});
      }
     private:
      bool nan_;
    };
    RF_EXPORT_CONTROLLER(Bad)
    """)
    for extra, words in (({}, "2 values but arm has 3"), ({"nan": 1}, "nan")):
        app = arm(cpp.CppController(source=str(f), **extra))
        with pytest.raises(ValueError, match=words):
            app.run(steps=10)
        app.close()


def test_source_or_library_and_missing_files(tmp_path):
    with pytest.raises(rf.ConfigError, match="either `source:`"):
        cpp.CppController()
    with pytest.raises(rf.ConfigError, match="not found"):
        arm(cpp.CppController(source=str(tmp_path / "nope.cpp"))).setup()
    with pytest.raises(rf.ConfigError, match="library not found"):
        arm(cpp.CppController(library=str(tmp_path / "libnope.so"))).setup()


def test_paths_in_app_yaml_are_relative_to_it_and_cli(tmp_path):
    (tmp_path / "sub").mkdir()
    source(tmp_path / "sub", HOLD, name="hold")
    (tmp_path / "sub" / "app.yaml").write_text("roboforge: 1\nrobot: arm\nrealtime: false\ncontrollers:\n  - {type: cpp, source: hold.cpp, rate: 500}\n")
    out = subprocess.run([sys.executable, "-m", "roboforge", "run", "sub/app.yaml", "--duration", "1", "--tap", "1"], cwd=tmp_path,
                         capture_output=True, text=True, timeout=120)
    assert out.returncode == 0 and "Hold" in out.stdout, out.stdout + out.stderr
    assert (tmp_path / "sub" / "build").is_dir(), "built next to the source"
    inc = subprocess.run([sys.executable, "-m", "roboforge", "cpp", "include"], capture_output=True, text=True).stdout.strip()
    assert (inc and (__import__("pathlib").Path(inc) / "roboforge" / "controller.hpp").is_file())
    new = subprocess.run([sys.executable, "-m", "roboforge", "cpp", "new", "my_ctl"], cwd=tmp_path, capture_output=True, text=True)
    assert new.returncode == 0 and (tmp_path / "my_ctl.cpp").is_file()
    lib = subprocess.run([sys.executable, "-m", "roboforge", "cpp", "build", "my_ctl.cpp"], cwd=tmp_path, capture_output=True, text=True)
    assert lib.returncode == 0 and lib.stdout.strip().endswith(".so"), lib.stderr
    bad = subprocess.run([sys.executable, "-m", "roboforge", "cpp", "new", "my_ctl"], cwd=tmp_path, capture_output=True, text=True)
    assert bad.returncode == 2 and "exists already" in bad.stderr


def test_cmake_build_of_the_example(tmp_path):
    if not shutil.which("cmake"):
        pytest.skip("no cmake")
    import os
    import pathlib
    ex = pathlib.Path(__file__).resolve().parents[1] / "examples" / "cpp_controller"
    shutil.copytree(ex, tmp_path / "ex")
    env = dict(os.environ, PATH=f"{pathlib.Path(sys.executable).parent}:{os.environ['PATH']}")
    for cmd in (["cmake", "-S", ".", "-B", "build/cmake"], ["cmake", "--build", "build/cmake"]):
        r = subprocess.run(cmd, cwd=tmp_path / "ex", capture_output=True, text=True, env=env, timeout=300)
        assert r.returncode == 0, r.stdout[-1500:] + r.stderr[-1500:]
    run = subprocess.run([sys.executable, "-m", "roboforge", "run", "app_cmake.yaml", "--headless", "--duration", "2", "--no-realtime", "--tap", "2"],
                         cwd=tmp_path / "ex", capture_output=True, text=True, timeout=120)
    assert run.returncode == 0 and "ArmPID" in run.stdout, run.stdout + run.stderr


def test_async_reset_and_many_instances(tmp_path):
    f = source(tmp_path, HOLD)
    ctl = cpp.CppController(source=str(f), rate=50)
    app = rf.App(robot="arm", controllers=[rf.Async(ctl, rate=50, timeout=0.5), rf.controller(lambda o, t: None)], realtime=False)
    app.run(duration=1)
    app.reset()
    app.run(duration=1)
    app.close()
    assert ctl.calls > 0
    apps = [arm(cpp.CppController(source=str(f), kp=k)) for k in (20, 60, 100)]
    for a in apps:
        a.run(duration=1.5)
    qs = [a.bus.latest("state/joint").position[1] for a in apps]
    for a in apps:
        a.close()
    assert len(set(np.round(qs, 6))) == 3, "each instance keeps its own state and parameters"
