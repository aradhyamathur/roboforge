#!/usr/bin/env python3
"""Create a project for every stack the New Project picker offers, through the
IDE's HTTP API, and run each one for a few seconds. Small, CPU-only runs.

  ROBOFORGE_HOME=<scratch home> ./target/release/roboforge --browser --no-open --port 7979 &
  python3 tests/api/stacks.py http://127.0.0.1:7979 <scratch dir for projects>
"""
import json, os, re, sys, time, urllib.request

URL, OUT = sys.argv[1].rstrip("/"), os.path.abspath(sys.argv[2])
TOKEN = re.search(r"const TOKEN = '([^']+)'", urllib.request.urlopen(URL + "/").read().decode()).group(1)

URDF = """<robot name="twolink">
  <link name="base"><inertial><mass value="1"/><inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/></inertial></link>
  <link name="l1"><inertial><origin xyz="0 0 0.15"/><mass value="0.5"/><inertia ixx="0.005" iyy="0.005" izz="0.001" ixy="0" ixz="0" iyz="0"/></inertial>
    <visual><origin xyz="0 0 0.15"/><geometry><cylinder radius="0.02" length="0.3"/></geometry></visual></link>
  <link name="l2"><inertial><origin xyz="0 0 0.15"/><mass value="0.3"/><inertia ixx="0.003" iyy="0.003" izz="0.001" ixy="0" ixz="0" iyz="0"/></inertial>
    <visual><origin xyz="0 0 0.15"/><geometry><cylinder radius="0.02" length="0.3"/></geometry></visual></link>
  <joint name="j1" type="revolute"><parent link="base"/><child link="l1"/><axis xyz="0 1 0"/><limit lower="-2" upper="2" effort="20" velocity="5"/></joint>
  <joint name="j2" type="revolute"><parent link="l1"/><child link="l2"/><origin xyz="0 0 0.3"/><axis xyz="0 1 0"/><limit lower="-2" upper="2" effort="20" velocity="5"/></joint>
</robot>
"""


def api(path, **body):
    req = urllib.request.Request(URL + path, json.dumps(body).encode(), {"content-type": "application/json", "x-rf-token": TOKEN})
    try:
        return json.loads(urllib.request.urlopen(req, timeout=120).read() or b"{}")
    except urllib.error.HTTPError as e:
        return {"error": json.loads(e.read() or b"{}").get("error", str(e))}


def job(res, timeout=300, until=None):
    """Wait for a job; returns (exit code, output). `until`: stop waiting once this text appears."""
    if "error" in res:
        return -1, res["error"]
    out, off, end = "", 0, time.time() + timeout
    while time.time() < end:
        r = api("/api/job/poll", id=res["job"], offset=off)
        out, off = out + r.get("data", ""), r.get("offset", off)
        if not r.get("running") or (until and until in out):
            return r.get("code"), out
        time.sleep(0.3)
    api("/api/job/kill", id=res["job"])
    return -2, out + "\n[timed out]"


def run(d, cmd, **kw):
    return job(api("/api/run", dir=d, cmd=cmd, workspace="app"), **kw)


os.makedirs(OUT, exist_ok=True)
urdf = os.path.join(OUT, "twolink.urdf")
open(urdf, "w").write(URDF)

M = "mujoco"
GOOD = [  # name, sim, robot, mavlink, ros
    ("quad_mav", M, "quadrotor", "vehicle", False), ("quad_mav_ros", M, "quadrotor", "vehicle", True),
    ("quad_plain", M, "quadrotor", "none", False), ("quad_ros", M, "quadrotor", "none", True),
    ("quad_px4", M, "quadrotor", "hil", False),
    ("arm_plain", M, "arm", "none", False), ("arm_ros", M, "arm", "none", True),
    ("base_plain", M, "diffdrive", "none", False), ("base_ros", M, "diffdrive", "none", True),
    ("urdf_plain", M, "custom", "none", False), ("urdf_ros", M, "custom", "none", True),
    ("bridge", "none", "", "client", True), ("mav_client", "none", "", "client", False), ("ros_only", "none", "", "none", True),
]
BAD = [("arm + MAVLink vehicle", M, "arm", "vehicle", False, ""), ("simulator + MAVLink client", M, "quadrotor", "client", False, ""),
       ("nothing to get state from", "none", "", "none", False, ""), ("vehicle without simulator", "none", "", "vehicle", True, ""),
       ("custom robot without a model", M, "custom", "none", False, "")]

fails, first = [], True


def check(name, ok, detail=""):
    print(f"  {'ok  ' if ok else 'FAIL'} {name}" + (f"\n       {detail.strip()[-600:]}" if not ok and detail else ""), flush=True)
    if not ok:
        fails.append(name)


print("refused combinations")
for label, sim, robot, mav, ros, model in BAD:
    d = os.path.join(OUT, "bad_" + re.sub(r"\W+", "_", label))
    r = api("/api/project/new", dir=d, name="bad", ros="ros2-jazzy" if ros else "", modules=[], sdk={"sim": sim, "robot": robot, "mavlink": mav, "ros": ros, "model": model})
    check(f"{label}: {r.get('error', 'ACCEPTED')}", "error" in r and not os.path.exists(os.path.join(d, "roboforge.json")))

for name, sim, robot, mav, ros in GOOD:
    print(f"{name}: sim={sim} robot={robot or '-'} mavlink={mav} ros={ros}")
    d = os.path.join(OUT, name)
    r = api("/api/project/new", dir=d, name=name, ros="ros2-jazzy" if ros else "", modules=[],
            sdk={"sim": sim, "robot": robot, "mavlink": mav, "ros": ros, "model": urdf if robot == "custom" else ""})
    check("project created", "error" not in r and os.path.isfile(os.path.join(d, "app", "app.yaml")), r.get("error", ""))
    if "error" in r:
        continue
    if first:  # a fresh home has no SDK environment yet: the Stack view must say so, then setup must fix it
        first = False
        s = api("/api/project/stack", dir=d, ws=0)
        if "error" in s:
            print("       (before setup the Stack view says: " + s["error"].splitlines()[1][:90] + ")")
        code, out = job(api("/api/sdk/setup", dir=d), timeout=600)
        check("Set up SDK environment", code == 0, out)
    else:
        code, out = job(api("/api/sdk/setup", dir=d), timeout=600)   # second time on: must be a quick no-op
        check("SDK environment reused", code == 0, out)
    proj = json.load(open(os.path.join(d, "roboforge.json")))
    ws = [i for i, w in enumerate(proj["workspaces"]) if w["kind"] == "app"][0]
    s = api("/api/project/stack", dir=d, ws=ws)
    links = [l["type"] + (":" + l["mode"] if l.get("mode") else "") for l in s.get("links", [])]
    errs = [l["error"] for l in s.get("links", []) if l.get("error")]
    check(f"Stack view: links {links or '-'}, {len(s.get('signals', []))} signals", "error" not in s and not errs, s.get("error", "") + " ".join(errs))
    names = [c["name"] for c in proj.get("run", [])]
    check(f"run configurations: {names}", "App: run" in names and (mav != "vehicle" or any("Mission" in n for n in names)))
    if mav == "hil":
        print("       (not run: it waits for PX4 SITL to connect, and PX4 is not installed here)")
        continue
    if mav == "vehicle":
        app = api("/api/run", dir=d, cmd="python3 -m roboforge run app.yaml --headless --duration 80", workspace="app")
        time.sleep(2.5)
        code, out = run(d, "python3 mission.py", timeout=120)
        check("mission over MAVLink: arm, take off, square, land", code == 0 and "landed and disarmed" in out, out)
        if ros:
            code, out = run(d, "timeout 8 ros2 topic list", timeout=30)
            check("ROS topics while flying", "/clock" in out and "/odom" in out, out)
        api("/api/job/kill", id=app["job"])
        code, out = job(app, timeout=20)
        check("app stops cleanly from the IDE's Stop", "stopped at t=" in out and "Traceback" not in out, out)
        continue
    app = api("/api/run", dir=d, cmd="python3 -m roboforge run app.yaml --headless --duration 8", workspace="app")
    if ros:
        time.sleep(3)
        code, out = run(d, "timeout 6 ros2 topic list", timeout=30)
        want = ["/clock"] if sim == M else ["/odom"] if mav == "client" else []
        check(f"ROS topics while running ({len([l for l in out.splitlines() if l.startswith('/')])} listed)", all(w in out for w in want), out)
    code, out = job(app, timeout=60)
    check("app ran 8 s and exited 0", code == 0 and "stopped at t=8.00s" in out and "Traceback" not in out, out)

print(f"\n{'ALL OK' if not fails else str(len(fails)) + ' FAILED: ' + '; '.join(fails)}")
sys.exit(1 if fails else 0)
