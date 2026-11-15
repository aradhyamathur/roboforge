"""Command line: `python -m roboforge <command>`.

    run CONFIG [key=value ...] [--duration S] [--headless] [--no-realtime]
    info CONFIG [--json]          components and signal wiring of an app
    doctor [--json]               which optional parts work in this environment
    zoo                           bundled robot models
    data export DIR --format nerfstudio|tum --out OUT [--camera NAME]
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
    ap = argparse.ArgumentParser(prog="python -m roboforge", description="RoboForge SDK")
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run", help="run an app from a config file")
    r.add_argument("config")
    r.add_argument("overrides", nargs="*", help="key=value overrides, e.g. physics.substeps=2")
    r.add_argument("--duration", type=float, help="seconds of app time to run (default: until Ctrl+C)")
    r.add_argument("--headless", action="store_true", help="no viewer window")
    r.add_argument("--no-realtime", action="store_true", help="run as fast as possible")
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
    a = ap.parse_args(argv)

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
        finally:
            t = app.time
            app.close()
        print(f"roboforge: stopped at t={t:.2f}s after {app.steps} steps", flush=True)
        return 0
    return 2


if __name__ == "__main__":
    sys.exit(main())
