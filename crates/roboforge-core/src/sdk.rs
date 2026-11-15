//! The RoboForge Python SDK: shipped inside the binary, unpacked into the
//! RoboForge home, and the starting files of a project built on it.

use crate::paths::Home;
use std::path::{Path, PathBuf};

include!(concat!(env!("OUT_DIR"), "/sdk_files.rs"));

/// A cheap fingerprint of the embedded SDK, to know when the unpacked copy is stale.
fn fingerprint() -> String {
    let mut h: u64 = 0xcbf29ce484222325;
    for (name, body) in SDK_FILES {
        for b in name.bytes().chain(body.bytes()) {
            h = (h ^ b as u64).wrapping_mul(0x100000001b3);
        }
    }
    format!("{h:016x}")
}

/// Where the SDK lives once unpacked (the folder to put on PYTHONPATH).
pub fn dir(home: &Home) -> PathBuf {
    home.root.join("sdk").join("python")
}

/// Unpack the embedded SDK if it is missing or from another build.
pub fn install(home: &Home) -> Result<PathBuf, String> {
    let root = dir(home);
    let stamp = root.join(".fingerprint");
    let want = fingerprint();
    if std::fs::read_to_string(&stamp).ok().as_deref() == Some(want.as_str()) {
        return Ok(root);
    }
    let pkg = root.join("roboforge");
    if pkg.is_dir() {
        std::fs::remove_dir_all(&pkg).map_err(|e| format!("{}: {e}", pkg.display()))?;
    }
    for (rel, body) in SDK_FILES {
        let f = root.join(rel);
        if let Some(p) = f.parent() {
            std::fs::create_dir_all(p).map_err(|e| format!("{}: {e}", p.display()))?;
        }
        std::fs::write(&f, body).map_err(|e| format!("{}: {e}", f.display()))?;
    }
    std::fs::write(&stamp, want).map_err(|e| e.to_string())?;
    Ok(root)
}

/// What the new-project stack picker chose.
#[derive(Debug, Clone, Default, serde::Deserialize)]
pub struct Stack {
    /// `mujoco` or `none`.
    #[serde(default)]
    pub sim: String,
    /// Mirror the app on ROS 2 topics.
    #[serde(default)]
    pub ros: bool,
    /// `none`, `vehicle`, `hil` or `client`.
    #[serde(default)]
    pub mavlink: String,
    /// `quadrotor`, `arm`, `diffdrive`, or `custom` with `model`.
    #[serde(default)]
    pub robot: String,
    /// Path to a URDF or MJCF file, for `robot: custom`.
    #[serde(default)]
    pub model: String,
}

impl Stack {
    pub fn validate(&self) -> Result<(), String> {
        let one_of = |what: &str, v: &str, ok: &[&str]| if ok.contains(&v) { Ok(()) } else { Err(format!("{what} must be one of: {}", ok.join(", "))) };
        one_of("simulator", &self.sim, &["mujoco", "none"])?;
        one_of("MAVLink mode", &self.mavlink, &["none", "vehicle", "hil", "client"])?;
        if self.sim == "mujoco" {
            one_of("robot", &self.robot, &["quadrotor", "arm", "diffdrive", "custom"])?;
            if self.robot == "custom" && self.model.trim().is_empty() {
                return Err("a custom robot needs a URDF or MJCF file".into());
            }
            if matches!(self.mavlink.as_str(), "vehicle" | "hil") && self.robot != "quadrotor" {
                return Err("MAVLink vehicle and PX4-in-the-loop modes need the quadrotor (they fly a multirotor)".into());
            }
            if self.mavlink == "client" {
                return Err("MAVLink client mode talks to an existing autopilot, so pick 'no simulator' with it".into());
            }
        } else {
            if self.mavlink != "client" && !self.ros {
                return Err("without a simulator, pick a MAVLink client and/or ROS 2 to get state from".into());
            }
            if matches!(self.mavlink.as_str(), "vehicle" | "hil") {
                return Err("MAVLink vehicle and PX4-in-the-loop modes need the MuJoCo simulator".into());
            }
        }
        Ok(())
    }

    /// Catalog components the stack needs, beyond the ROS distro.
    pub fn components(&self) -> Vec<String> {
        let mut c = vec!["roboforge-sdk".to_string()];
        if self.sim == "mujoco" {
            c.push("mujoco".into());
        }
        if self.mavlink != "none" {
            c.push("pymavlink".into());
        }
        c
    }

    fn title(&self) -> String {
        let mut parts = Vec::new();
        if self.sim == "mujoco" { parts.push("MuJoCo"); }
        if self.ros { parts.push("ROS 2"); }
        if self.mavlink != "none" { parts.push("MAVLink"); }
        parts.join(" + ")
    }
}

/// The starting files of an SDK app, relative to its workspace folder.
pub fn scaffold(name: &str, st: &Stack) -> Result<Vec<(String, String)>, String> {
    st.validate()?;
    let sim = st.sim == "mujoco";
    let external = matches!(st.mavlink.as_str(), "vehicle" | "hil");     // something outside decides where to go
    let mut y = String::new();
    y += &format!("# {} app, built on the RoboForge SDK.\n", st.title());
    y += "# Run it:      python3 -m roboforge run app.yaml          (or the Run / Launch buttons)\n";
    y += "# Inspect it:  python3 -m roboforge info app.yaml         (components and signal wiring)\n";
    y += "# Every key under a component is an argument of that class; `type` may also be\n";
    y += "# `module:Class` for your own code in this folder. Override from the command line:\n";
    y += "#   python3 -m roboforge run app.yaml physics.substeps=2 --headless\n";
    y += "roboforge: 1\n";
    y += &format!("name: {name}\n\n");
    if sim {
        match st.robot.as_str() {
            "custom" => {
                let key = if st.model.trim().to_ascii_lowercase().ends_with(".urdf") { "urdf" } else { "mjcf" };
                y += "# Your own model. profile: articulated | arm | multirotor | diffdrive\n";
                y += &format!("robot: {{{key}: {}, profile: articulated}}\n", crate::sh_quote(st.model.trim()).replace('\'', "\""));
            }
            r => y += &format!("robot: {r}                 # bundled model; list them with: python3 -m roboforge zoo\n"),
        }
        y += "physics: {type: mujoco}\n";
        y += "renderer: {type: mujoco, width: 320, height: 240, fps: 10}\n";
    } else {
        y += "# No simulator: state comes in through the links below, time is the wall clock.\n";
        y += "physics: {type: none, rate: 50}\n";
    }
    y += "\nlinks:\n";
    match st.mavlink.as_str() {
        "vehicle" => {
            y += "  # The app is the vehicle: QGroundControl (14550) and MAVSDK / pymavlink (14540) connect to it.\n";
            y += "  - {type: mavlink, mode: vehicle, urls: [\"udpout:127.0.0.1:14550\", \"udpout:127.0.0.1:14540\"]}\n";
        }
        "hil" => {
            y += "  # PX4 SITL connects here and flies the simulated vehicle (experimental).\n";
            y += "  # Start PX4 with its `none` simulator target after this app is running.\n";
            y += "  - {type: mavlink, mode: hil, url: \"tcpin:0.0.0.0:4560\", connect_timeout: 120}\n";
        }
        "client" => {
            y += "  # Talks to an autopilot (PX4 SITL sends to 14540 by default).\n";
            y += "  - {type: mavlink, mode: client, url: \"udpin:0.0.0.0:14540\"}\n";
        }
        _ => {}
    }
    if st.ros {
        if sim {
            y += "  # State out (/clock, joint_states, odom, /tf, imu, camera/*), commands in (cmd_vel, joint_command, setpoint_pose).\n";
            y += "  - {type: ros2}\n";
        } else {
            y += "  # Telemetry out as odom + /tf, pose setpoints in on setpoint_pose.\n";
            y += "  - {type: ros2, enable: [odom, setpoint]}\n";
        }
    }
    if st.mavlink == "none" && !st.ros {
        y += "  []\n";
    }
    let mut files: Vec<(String, String)> = Vec::new();
    if sim {
        y += "\ncontrollers:\n";
        match st.robot.as_str() {
            "quadrotor" => {
                if st.mavlink == "hil" {
                    y += "  []        # PX4 is the controller in this mode\n";
                } else {
                    if !external {
                        y += "  - {type: \"controller:Patrol\", rate: 0.25}     # yours: where to fly (controller.py)\n";
                        files.push(("controller.py".into(), PATROL.into()));
                    }
                    y += "  - {type: multirotor, rate: 250}                # built in: setpoint -> motor commands\n";
                }
            }
            "arm" => {
                y += "  - {type: \"controller:Wave\", rate: 50}            # yours: joint targets (controller.py)\n";
                y += "  - {type: joint_pd, rate: 500}                    # built in: joint targets -> torques\n";
                files.push(("controller.py".into(), WAVE.into()));
            }
            "diffdrive" => {
                y += "  - {type: \"controller:Circle\", rate: 10}          # yours: velocity command (controller.py)\n";
                y += "  - {type: diffdrive}                              # built in: velocity -> wheel speeds\n";
                files.push(("controller.py".into(), CIRCLE.into()));
            }
            _ => {
                y += "  - {type: \"controller:Hold\", rate: 500}           # yours: a PID loop per joint (controller.py)\n";
                files.push(("controller.py".into(), HOLD.into()));
            }
        }
        y += "\ncallbacks:\n";
        y += "  - {type: viewer}                                 # MuJoCo window; --headless skips it\n";
        y += "  # - {type: recorder, dir: runs/latest}           # record a dataset (export: python3 -m roboforge data export ...)\n";
    }
    files.insert(0, ("app.yaml".into(), y));
    if st.mavlink == "vehicle" {
        files.push(("mission.py".into(), MISSION.into()));
    }
    Ok(files)
}

const PATROL: &str = r#""""Where the quadrotor should go. Edit freely: this is your code.

A controller is anything with `step(obs, t)`. This one runs four times a
minute and publishes a position setpoint; the built-in multirotor controller
(see app.yaml) turns setpoints into motor commands 250 times a second.
"""
import numpy as np
import roboforge as rf

CORNERS = [(0, 0, 1.5), (2, 0, 1.5), (2, 2, 2.0), (0, 2, 1.5)]


class Patrol(rf.Controller):
    def step(self, obs, t):
        x, y, z = CORNERS[int(t / 4.0) % len(CORNERS)]
        return rf.Setpoint(position=np.array([x, y, z], float), yaw=0.0)
"#;

const WAVE: &str = r#""""Joint targets for the arm. Edit freely: this is your code.

`step` returns a message; the built-in joint controller (see app.yaml) turns
joint targets into torques. `obs.joint` is the current joint state.
"""
import numpy as np
import roboforge as rf


class Wave(rf.Controller):
    def step(self, obs, t):
        return rf.JointCommand(position=np.array([0.8 * np.sin(0.6 * t), 0.5 + 0.3 * np.sin(1.1 * t), -0.9]))
"#;

const CIRCLE: &str = r#""""Velocity command for the base. Edit freely: this is your code.

Publishes forward speed and turn rate; the built-in diff-drive controller
(see app.yaml) turns that into wheel speeds. With ROS 2 enabled you can
remove this controller from app.yaml and drive with `cmd_vel` instead.
"""
import roboforge as rf


class Circle(rf.Controller):
    def step(self, obs, t):
        return rf.velocity_command(linear=(0.3, 0.0, 0.0), angular=(0.0, 0.0, 0.4))
"#;

const HOLD: &str = r#""""A PID loop per joint for your own robot. Edit freely: this is your code.

Holds every joint at the angle it starts at. `self.physics.raw` is the
MuJoCo (model, data) pair if you need more than the bus gives you.
"""
import numpy as np
import roboforge as rf


class Hold(rf.Controller):
    def setup(self, app):
        self.physics = app.physics
        self.pid = rf.PID(kp=40.0, ki=10.0, kd=4.0)
        self.target = None

    def step(self, obs, t):
        js = obs.joint
        if js is None or not len(js.position):
            return None
        if self.target is None:
            self.target = js.position.copy()
        u = self.pid(self.target - js.position, self.physics.dt, rate=-js.velocity)
        return rf.ActuatorCommand(values=np.atleast_1d(u))
"#;

const MISSION: &str = r#""""Fly the simulated vehicle over MAVLink: arm, take off, fly a square, land.

Run the app first (python3 -m roboforge run app.yaml), then this script.
It is an ordinary MAVLink client, so it would fly a real PX4 the same way.
"""
import numpy as np
import roboforge as rf


def main():
    mav = rf.MavlinkLink(mode="client", url="udpin:127.0.0.1:14540")
    app = rf.App(physics={"type": "none", "rate": 50}, links=[mav])
    app.setup()

    def position():
        pose = app.bus.latest("state/pose")
        return None if pose is None else pose.position

    def fly_to(x, y, z, timeout=40.0):
        target = np.array([x, y, z], float)
        app.bus.publish("cmd/setpoint", rf.Setpoint(position=target))
        end = app.time + timeout
        app.run(until=lambda a: a.time > end or np.linalg.norm(position() - target) < 0.3)
        print(f"at {np.round(position(), 2)}", flush=True)

    print("waiting for the vehicle...", flush=True)
    if not mav.wait_connected(60.0):
        raise SystemExit("no vehicle heard on udp 14540. Is the app running?")
    app.run(until=lambda a: position() is not None)
    print("armed:", mav.arm(), flush=True)
    print("takeoff:", mav.takeoff(2.0), flush=True)
    app.run(until=lambda a: position()[2] > 1.7)
    for corner in [(2, 0, 2), (2, 2, 2), (0, 2, 2), (0, 0, 2)]:
        fly_to(*corner)
    print("landing:", mav.land(), flush=True)
    app.run(until=lambda a: not app.bus.latest("state/status").armed)
    print("landed and disarmed", flush=True)
    app.close()


if __name__ == "__main__":
    main()
"#;

/// Run configurations every SDK project gets.
pub fn run_configs(st: &Stack) -> Vec<(String, String)> {
    let mut v = vec![
        ("App: run".to_string(), "python3 -m roboforge run app.yaml".to_string()),
        ("App: show wiring".to_string(), "python3 -m roboforge info app.yaml".to_string()),
        ("SDK: check environment".to_string(), "python3 -m roboforge doctor".to_string()),
    ];
    if st.mavlink == "vehicle" {
        v.push(("Mission: fly a square (MAVLink)".to_string(), "python3 mission.py".to_string()));
    }
    v
}

/// Shell script that creates a small Python environment for SDK apps: the
/// system Python (so a system ROS 2 stays importable) plus the pip packages
/// the stack needs. Uses `uv` when present, `python3 -m venv` otherwise.
pub fn light_env_script(home: &Home) -> String {
    let venv = light_env(home);
    let v = crate::sh_quote(&venv.to_string_lossy());
    format!(
        "set -e\nV={v}\nPKGS=\"numpy pyyaml mujoco pymavlink gymnasium\"\nif command -v uv >/dev/null 2>&1; then\n  uv venv --allow-existing --python /usr/bin/python3 --system-site-packages \"$V\"\n  uv pip install --python \"$V/bin/python\" $PKGS\nelse\n  /usr/bin/python3 -m venv --system-site-packages \"$V\" || {{ echo 'Could not create a virtualenv. Install uv (https://docs.astral.sh/uv/) or the python3-venv package, then run this again.'; exit 1; }}\n  \"$V/bin/python\" -m pip install $PKGS\nfi\necho\necho \"Environment ready: $V\"\n\"$V/bin/python\" -m roboforge doctor\n"
    )
}

/// The interpreter of the light SDK environment.
pub fn light_env(home: &Home) -> PathBuf {
    home.root.join("envs").join("sdk-venv")
}

pub fn light_python(home: &Home) -> PathBuf {
    light_env(home).join("bin").join("python")
}

/// True if `dir` holds an SDK app config.
pub fn is_app_config(path: &Path) -> bool {
    std::fs::read_to_string(path).map_or(false, |t| t.lines().any(|l| l.trim_start() == l && l.starts_with("roboforge:")))
}
