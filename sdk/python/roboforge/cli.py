"""Command line: `python -m roboforge <command>`.

    run CONFIG [key=value ...] [--duration S] [--headless] [--no-realtime] [--tap [S]] [--check]
    info CONFIG [--json]          components and signal wiring of an app
    doctor [--json]               which optional parts work in this environment
    zoo                           bundled robot models
    data export DIR --format nerfstudio|tum --out OUT [--camera NAME]
    cpp include | build SOURCE [--out DIR] | new NAME     C++ controllers
"""
from __future__ import annotations

import argparse
import importlib
import json
import os
import sys

from . import __version__

# What each optional part needs, and how to get it.
PARTS = [
    ("mujoco", "mujoco", "MuJoCo physics and renderer", "pip install mujoco"),
    ("mavlink", "pymavlink", "MAVLink link (vehicle, client, hil)", "pip install pymavlink"),
    ("ros2", "rclpy", "ROS 2 link", "source a ROS 2 setup file (e.g. /opt/ros/<distro>/setup.bash)"),
    ("gym", "gymnasium", "Gymnasium environment adapter", "pip install gymnasium"),
    ("lerobot", "lerobot", "LeRobot datasets and policies (optional)", 'pip install "lerobot[dataset]" (needs numpy < 2.3; see the LeRobot tutorial)'),
]


def doctor() -> dict:
    report = {"version": __version__, "python": sys.version.split()[0], "display": bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")), "parts": []}
    for key, module, what, fix in PARTS:
        try:
            m = importlib.import_module(module)
            report["parts"].append({"name": key, "what": what, "ok": True, "version": str(getattr(m, "__version__", ""))})
        except Exception as e:
            report["parts"].append({"name": key, "what": what, "ok": False, "error": f"{type(e).__name__}: {e}", "fix": fix})
    return report


def _info(config, overrides, steps: int) -> dict:
    """Describe an app without touching the network: links are declared, not started."""
    from .app import App
    from .config import resolve
    from . import registry as reg
    cfg, _ = resolve(config, overrides)
    link_specs = list(cfg.get("links") or [])
    app = App.from_config(config, overrides, links=[], callbacks=[], realtime=False)
    links = []
    for spec in link_specs:
        try:
            links.append(reg.LINKS.build(spec).describe())
        except Exception as e:
            t = spec if isinstance(spec, str) else spec.get("type", "?")
            links.append({"type": t, "error": f"{type(e).__name__}: {e}", "inputs": [], "outputs": []})
    try:
        app.run(steps=steps)
        d = app.describe()
    finally:
        app.close()
    d["links"] = links
    d["callbacks"] = [c if isinstance(c, str) else c.get("type", "?") for c in (cfg.get("callbacks") or [])]
    return d


def main(argv=None) -> int:
    """Run a command. Mistakes in a config, missing files and packages, ports in use and a
    simulation that blows up are reported in one line; an exception from the user's own
    code keeps its traceback, because the line number is what they need."""
    from .errors import ConfigError, SimulationError
    try:
        return _main(argv)
    except (ConfigError, SimulationError, ImportError, OSError) as e:
        text = str(e) if not isinstance(e, FileNotFoundError) or e.filename is None else f"no such file: {e.filename}"
        notes = " ".join(getattr(e, "__notes__", []))
        print(f"roboforge: error: {text}{' ' + notes if notes else ''}", file=sys.stderr, flush=True)
        return 2


def _main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="python -m roboforge", description="RoboForge SDK")
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run", help="run an app from a config file")
    r.add_argument("config")
    r.add_argument("overrides", nargs="*", help="key=value overrides, e.g. physics.substeps=2")
    r.add_argument("--duration", type=float, help="seconds of app time to run (default: until Ctrl+C)")
    r.add_argument("--headless", action="store_true", help="no viewer window")
    r.add_argument("--no-realtime", action="store_true", help="run as fast as possible")
    r.add_argument("--tap", nargs="?", const=1.0, type=float, metavar="SECONDS", help="print every signal's rate, source and value this often (default 1 s)")
    r.add_argument("--check", action="store_true", help="watch for common problems (no command, saturation, disarmed, too slow) and report them")
    i = sub.add_parser("info", help="show an app's components and signal wiring")
    i.add_argument("config")
    i.add_argument("overrides", nargs="*")
    i.add_argument("--json", action="store_true")
    i.add_argument("--steps", type=int, default=50)
    d = sub.add_parser("doctor", help="check which optional parts work here")
    d.add_argument("--json", action="store_true")
    sub.add_parser("zoo", help="list bundled robots")
    da = sub.add_parser("data", help="work with recorded datasets")
    dsub = da.add_subparsers(dest="data_cmd", required=True)
    ex = dsub.add_parser("export")
    ex.add_argument("dir")
    ex.add_argument("--format", required=True, choices=["nerfstudio", "tum"])
    ex.add_argument("--out", required=True)
    ex.add_argument("--camera")
    # Overrides may come before or after the options (`run app.yaml --headless physics.substeps=2`).
    lr = sub.add_parser("lerobot", help="LeRobot: train a policy on a recorded dataset, describe a dataset")
    lsub = lr.add_subparsers(dest="lerobot_cmd", required=True)
    lt = lsub.add_parser("train", help="train a LeRobot policy on a dataset")
    lt.add_argument("dataset")
    lt.add_argument("--out", required=True, help="folder to save the policy to")
    lt.add_argument("--policy", default="act", help="LeRobot policy type: act, diffusion, ... (default act)")
    lt.add_argument("--steps", type=int, default=2000)
    lt.add_argument("--batch-size", type=int, default=32)
    lt.add_argument("--lr", type=float)
    lt.add_argument("--device", default="auto", help="auto (GPU if PyTorch can use one), cpu, cuda, mps")
    lt.add_argument("--log-every", type=int, default=200, help="print the loss every N steps")
    lt.add_argument("options", nargs="*", help="policy config options, e.g. chunk_size=10 dim_model=64 use_vae=false")
    li = lsub.add_parser("info", help="describe a LeRobot dataset")
    li.add_argument("dataset")
    cp = sub.add_parser("cpp", help="C++ controllers: header location, build, new")
    csub = cp.add_subparsers(dest="cpp_cmd", required=True)
    csub.add_parser("include", help="print the folder to add to the include path (-I)")
    cb = csub.add_parser("build", help="compile a controller into a shared library")
    cb.add_argument("source")
    cb.add_argument("--out", help="folder for the library (default: build/ next to the source)")
    cn = csub.add_parser("new", help="write a starting controller NAME.cpp")
    cn.add_argument("name")
    a, extra = ap.parse_known_args(argv)
    if extra:
        if a.cmd in ("run", "info") and not any(x.startswith("-") for x in extra):
            a.overrides = list(a.overrides) + extra
        elif a.cmd == "lerobot" and getattr(a, "lerobot_cmd", "") == "train" and not any(x.startswith("-") for x in extra):
            a.options = list(a.options) + extra
        else:
            ap.error(f"unrecognized arguments: {' '.join(extra)}")

    if a.cmd == "doctor":
        rep = doctor()
        if a.json:
            print(json.dumps(rep, indent=2))
        else:
            print(f"roboforge {rep['version']} on Python {rep['python']}  (display: {'yes' if rep['display'] else 'no'})")
            for p in rep["parts"]:
                print(f"  {'ok ' if p['ok'] else 'MISSING'}  {p['name']:8s} {p['what']}" + (f"  [{p['version']}]" if p.get("version") else "") + ("" if p["ok"] else f"\n           fix: {p['fix']}"))
        return 0
    if a.cmd == "zoo":
        from .robot import Robot
        for name, spec in Robot.zoo_names().items():
            print(f"  {name:12s} {spec['profile']:12s} {spec.get('summary', '')}")
        return 0
    if a.cmd == "lerobot":
        from . import lerobot
        if a.lerobot_cmd == "info":
            print(json.dumps(lerobot.info(a.dataset), indent=2))
            return 0
        import yaml
        opts = {}
        for item in a.options:
            if "=" not in item:
                from .errors import ConfigError
                raise ConfigError(f"policy option {item!r} is not of the form name=value (for example chunk_size=10)")
            k, _, v = item.partition("=")
            opts[k] = yaml.safe_load(v)
        lerobot.train(a.dataset, a.out, policy=a.policy, steps=a.steps, batch_size=a.batch_size, lr=a.lr, device=a.device, log_every=a.log_every, **opts)
        return 0
    if a.cmd == "cpp":
        from . import cpp
        if a.cpp_cmd == "include":
            print(cpp.include_dir())
        elif a.cpp_cmd == "build":
            print(cpp.build(a.source, out_dir=a.out))
        else:
            path = cpp.new(a.name)
            print(f"wrote {path}\nuse it in app.yaml:  - {{type: cpp, source: {path.name}, rate: 500}}")
        return 0
    if a.cmd == "data":
        from .data import Dataset
        ds = Dataset(a.dir)
        out = ds.export_nerfstudio(a.out, a.camera) if a.format == "nerfstudio" else ds.export_tum(a.out, a.camera)
        print(f"exported {a.format} dataset to {out}")
        return 0
    if a.cmd == "info":
        info = _info(a.config, a.overrides, a.steps)
        if a.json:
            print(json.dumps(info, indent=2, default=str))
            return 0
        rb = info["robot"]
        print(f"{info['name']}: robot {rb['name'] + ' (' + rb['profile'] + ')' if rb else 'none'}, physics {info['physics']['type']} (dt {info['physics']['dt']:.4f}s)")
        if info["renderer"]:
            print(f"  renderer   {info['renderer']['type']}  cameras: {', '.join(info['renderer'].get('cameras', []))}")
        for c in info["controllers"]:
            print(f"  controller {c['type']}  {str(c['rate']) + ' Hz' if c['rate'] else 'every step'}")
        for l in info["links"]:
            print(f"  link       {l['type']}{' (' + l['mode'] + ')' if l.get('mode') else ''}  in: {', '.join(l['inputs']) or '-'}  out: {', '.join(l['outputs']) or '-'}" + (f"  ERROR {l['error']}" if l.get("error") else ""))
        print("  signals    " + ", ".join(f"{s['name']} [{s['type']}]" for s in info["signals"]))
        return 0
    if a.cmd == "run":
        from .app import App
        from .config import resolve
        cfg, _ = resolve(a.config, a.overrides)
        replace = {}
        if a.headless:
            replace["callbacks"] = [c for c in (cfg.get("callbacks") or []) if (c if isinstance(c, str) else c.get("type")) != "viewer"]
        if a.no_realtime:
            replace["realtime"] = False
        if a.tap or a.check:
            extra_cb = ([{"type": "tap", "every": a.tap}] if a.tap else []) + ([{"type": "checks"}] if a.check else [])
            replace["callbacks"] = list(replace.get("callbacks", cfg.get("callbacks") or [])) + extra_cb
        app = App.from_config(a.config, a.overrides, **replace)
        duration = a.duration if a.duration is not None else (cfg.get("run") or {}).get("duration")
        rb = app.robot
        print(f"roboforge: running {app.name!r} (robot: {rb.name if rb else 'none'}, links: {', '.join(l.name for l in app.links) or 'none'})"
              + (f" for {duration:g}s" if duration else "; Ctrl+C to stop"), flush=True)
        # Stop cleanly on Ctrl+C and on the IDE's Stop (SIGTERM), also when started
        # in the background, where the shell leaves SIGINT ignored.
        import signal

        def stop(_sig, _frame):
            raise KeyboardInterrupt
        signal.signal(signal.SIGINT, stop)
        signal.signal(signal.SIGTERM, stop)
        try:
            app.run(duration=duration)
        except BaseException:
            print(f"roboforge: stopped at t={app.time:.2f}s after {app.steps} steps, on an error", flush=True)
            raise
        finally:
            t = app.time
            try:
                app.close()
            except Exception as e:              # do not let a failing shutdown hide the real error
                print(f"roboforge: while closing: {type(e).__name__}: {e}", file=sys.stderr, flush=True)
        print(f"roboforge: stopped at t={t:.2f}s after {app.steps} steps", flush=True)
        return 0
    return 2


if __name__ == "__main__":
    sys.exit(main())
