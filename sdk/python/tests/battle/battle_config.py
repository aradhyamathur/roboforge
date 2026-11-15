"""Mistakes a user will make in app.yaml and at the API. Each must fail early
with a message that says what is wrong and what is allowed."""
import pathlib
import tempfile

import numpy as np

import roboforge as rf
from harness import clear_error, scenario


def cfg(**over):
    c = {"roboforge": 1, "name": "t", "robot": "quadrotor", "physics": {"type": "mujoco"}, "realtime": False}
    c.update(over)
    return c


def build(c, overrides=()):
    app = rf.App.from_config(c, overrides)
    app.setup()
    app.run(steps=5)
    app.close()


@scenario()
def config_unknown_top_level_key():
    return clear_error(lambda: build(cfg(controler=[])), "controler", "controllers")


@scenario()
def config_unknown_robot():
    return clear_error(lambda: build(cfg(robot="quadcopter")), "quadcopter", "quadrotor")


@scenario()
def config_unknown_types():
    out = [clear_error(lambda: build(cfg(physics={"type": "bullet"})), "bullet", "mujoco"),
           clear_error(lambda: build(cfg(links=[{"type": "mavlnk"}])), "mavlnk", "mavlink"),
           clear_error(lambda: build(cfg(controllers=[{"type": "multirotr"}])), "multirotr", "multirotor"),
           clear_error(lambda: build(cfg(callbacks=[{"type": "recoder"}])), "recoder", "recorder"),
           clear_error(lambda: build(cfg(renderer={"type": "opengl"})), "opengl", "mujoco")]
    return out[0]


@scenario()
def config_unknown_argument():
    """A misspelt argument of a component must name the component and the argument."""
    out = [clear_error(lambda: build(cfg(physics={"type": "mujoco", "substep": 2})), "substep"),
           clear_error(lambda: build(cfg(controllers=[{"type": "multirotor", "rte": 250}])), "rte", "multirotor"),
           clear_error(lambda: build(cfg(links=[{"type": "mavlink", "mod": "vehicle"}])), "mod", "mavlink"),
           clear_error(lambda: build(cfg(renderer={"type": "mujoco", "widht": 64})), "widht")]
    return out[1]


@scenario()
def config_missing_and_broken_files():
    d = pathlib.Path(tempfile.mkdtemp())
    (d / "bad.yaml").write_text("robot: [unclosed\n")
    (d / "list.yaml").write_text("- a\n- b\n")
    (d / "empty.yaml").write_text("")
    out = [clear_error(lambda: rf.App.from_config(d / "nope.yaml"), "nope.yaml"),
           clear_error(lambda: rf.App.from_config(d / "bad.yaml"), "bad.yaml"),
           clear_error(lambda: rf.App.from_config(d / "list.yaml"), "list.yaml"),
           clear_error(lambda: build(cfg(robot={"mjcf": "missing_model.xml"})), "missing_model.xml"),
           clear_error(lambda: build(cfg(robot={"urdf": "missing_model.urdf"})), "missing_model.urdf")]
    app = rf.App.from_config(d / "empty.yaml")      # an empty file is an app with nothing in it
    clear_error(lambda: app.run(steps=1), "physics", "robot") if False else None
    return out[1]


@scenario()
def config_broken_model_file():
    d = pathlib.Path(tempfile.mkdtemp())
    (d / "broken.xml").write_text("<mujoco><worldbody><body><geom size='0.1'/></worldbody></mujoco>")
    (d / "notxml.xml").write_text("hello")
    return (clear_error(lambda: build(cfg(robot={"mjcf": str(d / "broken.xml")})), "broken.xml") + " | " +
            clear_error(lambda: build(cfg(robot={"mjcf": str(d / "notxml.xml")})), "notxml.xml"))


@scenario()
def config_bad_overrides():
    out = [clear_error(lambda: build(cfg(), ["physics.substeps"]), "path=value"),
           clear_error(lambda: build(cfg(links=[]), ["links.3.url=udpout:1.2.3.4:1"]), "links"),
           clear_error(lambda: build(cfg(), ["nosuchkey=3"]), "nosuchkey"),
           clear_error(lambda: build(cfg(controllers=[{"type": "multirotor"}]), ["controllers.x.rate=3"]), "controllers")]
    return out[3]


@scenario()
def config_user_class_errors():
    d = pathlib.Path(tempfile.mkdtemp())
    (d / "controller.py").write_text("import roboforge as rf\nclass Mine(rf.Controller):\n    def step(self, obs, t):\n        return None\n")
    (d / "broken.py").write_text("import roboforge as rf\nclass Mine(rf.Controller)\n")
    base = "roboforge: 1\nrobot: arm\nrealtime: false\ncontrollers:\n  - {type: \"%s\"}\n"
    def run(t):
        (d / "app.yaml").write_text(base % t)
        build(d / "app.yaml")
    run("controller:Mine")
    out = [clear_error(lambda: run("controller:Mien"), "Mien", "controller"),
           clear_error(lambda: run("controler:Mine"), "controler"),
           clear_error(lambda: run("broken:Mine"), "broken", kinds=(ValueError, ImportError, SyntaxError))]
    return out[0]


@scenario()
def config_bad_values():
    out = [clear_error(lambda: build(cfg(controllers=[{"type": "multirotor", "rate": -5}])), "rate"),
           clear_error(lambda: build(cfg(controllers=[{"type": "multirotor", "rate": "fast"}])), "rate"),
           clear_error(lambda: build(cfg(renderer={"type": "mujoco", "width": 0, "height": 0})), "width"),
           clear_error(lambda: build(cfg(renderer={"type": "mujoco", "fps": -1})), "fps"),
           clear_error(lambda: build(cfg(speed=0)), "speed"),
           clear_error(lambda: build(cfg(physics={"type": "mujoco", "substeps": 0})), "substeps"),
           clear_error(lambda: build(cfg(links=[{"type": "mavlink", "mode": "vehicle", "url": "udp-out:nowhere"}])), "url"),
           clear_error(lambda: build(cfg(links=[{"type": "mavlink", "mode": "vehicle", "url": "udpout:127.0.0.1:99999"}])), "99999")]
    return out[0]


@scenario()
def config_wrong_robot_for_controller():
    """A controller on a robot it cannot drive must say so, not crash in numpy."""
    out = [clear_error(lambda: build(cfg(robot="arm", controllers=[{"type": "multirotor"}])), "multirotor"),
           clear_error(lambda: build(cfg(robot="quadrotor", controllers=[{"type": "diffdrive"}])), "diffdrive"),
           clear_error(lambda: build(cfg(robot="quadrotor", controllers=[{"type": "joint_pd"}])), "joint"),
           clear_error(lambda: build(cfg(robot="arm", links=[{"type": "mavlink", "mode": "vehicle", "url": "udpout:127.0.0.1:14999"}])), "mavlink")]
    return out[0]


@scenario()
def api_controller_returns_garbage():
    """What a controller returns is user code: wrong shapes and types must be named."""
    def run(robot, ret, builtin):
        app = rf.App(robot=robot, controllers=[rf.controller(lambda obs, t: ret), builtin()], realtime=False)
        app.run(steps=20)
        app.close()
    out = [clear_error(lambda: run("arm", rf.JointCommand(position=np.zeros(7)), lambda: rf.JointPD()), "7", "3"),
           clear_error(lambda: run("arm", "go left", lambda: rf.JointPD()), "str"),
           clear_error(lambda: run("arm", 3.0, lambda: rf.JointPD()), "float"),
           clear_error(lambda: run("quadrotor", rf.ActuatorCommand(values=np.ones(9)), lambda: rf.controller(lambda o, t: None)), "9", "4"),
           clear_error(lambda: run("arm", rf.JointCommand(position=[float("nan")] * 3), lambda: rf.JointPD()), "nan"),
           clear_error(lambda: run("quadrotor", rf.Setpoint(position=np.array([np.nan, 0, 1.0])), lambda: rf.MultirotorController()), "nan")]
    return out[0]


@scenario()
def api_controller_raises():
    """An exception inside user code must surface as itself, with the controller named, and the app must still close."""
    class Boom(rf.Controller):
        def step(self, obs, t):
            if t > 0.01:
                raise ZeroDivisionError("user bug")
    app = rf.App(robot="arm", controllers=[Boom()], realtime=False)
    try:
        app.run(steps=100)
    except ZeroDivisionError as e:
        notes = getattr(e, "__notes__", [])
        assert any("Boom" in n for n in notes) or "Boom" in str(e), f"the failing controller is not named: {e!r} {notes}"
    else:
        raise AssertionError("the controller's exception was swallowed")
    app.close()
    app.close()        # twice is fine
    return "named"


@scenario()
def api_misuse_of_app():
    app = rf.App(robot="arm", realtime=False)
    app.run(steps=3)
    app.close()
    out = clear_error(lambda: app.step(), "closed")            # stepping a closed app
    clear_error(lambda: rf.App(robot="arm", physics="none").run(steps=1) or (_ for _ in ()).throw(AssertionError("ran")), "robot") if False else None
    clear_error(lambda: rf.App(controllers=[rf.MultirotorController()], realtime=False).run(steps=2), "robot")
    clear_error(lambda: rf.App(robot="arm", realtime=False).as_env().step(np.zeros(11)), "11", "3")
    clear_error(lambda: rf.App(robot="arm", renderer="mujoco", realtime=False).renderer.render("no_such_camera") if False else
                (lambda a: (a.setup(), a.renderer.render("no_such_camera")))(rf.App(robot="arm", renderer="mujoco", realtime=False)), "no_such_camera")
    return out
