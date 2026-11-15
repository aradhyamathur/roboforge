//! Project generator. A project is a ROS choice plus a set of modules; the
//! generator hosts one ROS package and lets every module contribute files,
//! dependencies, run configurations and launch entries to it. Modules can be
//! added to or removed from an existing project.

use crate::catalog::Catalog;
use crate::modules::{parse_component, LaunchDef, Module, Registry};
use crate::paths::Home;
use crate::project::{default_ws_path, find_ws, write_activation, Project, RunConfig, Workspace, MANIFEST};
use crate::resolve::resolve;
use serde::Serialize;
use std::collections::BTreeSet;
use std::path::Path;

#[derive(Debug, Clone, Serialize)]
pub struct Preset {
    pub id: &'static str,
    pub name: &'static str,
    pub summary: &'static str,
    /// `ros2`, `ros1` or `none`.
    pub ros: &'static str,
    pub modules: &'static [&'static str],
    /// Extra SDK components.
    pub components: &'static [&'static str],
}

pub const PRESETS: &[Preset] = &[
    Preset { id: "ros2", name: "ROS 2 workspace", summary: "colcon workspace with a Python package, node and launch file.", ros: "ros2", modules: &[], components: &[] },
    Preset { id: "ros1", name: "ROS 1 workspace", summary: "catkin workspace on Noetic with a rospy node.", ros: "ros1", modules: &[], components: &[] },
    Preset { id: "px4", name: "PX4 only", summary: "PX4 SITL with Gazebo and a MAVSDK offboard script. No ROS.", ros: "none", modules: &["px4-sitl", "mavsdk"], components: &[] },
    Preset { id: "mujoco", name: "MuJoCo only", summary: "Standalone MuJoCo model and viewer script.", ros: "none", modules: &["mujoco"], components: &[] },
    Preset { id: "ros2-px4", name: "ROS 2 + PX4", summary: "Offboard control node over uXRCE-DDS, PX4 SITL in Gazebo.", ros: "ros2", modules: &["px4-offboard", "gazebo"], components: &[] },
    Preset { id: "ros2-mujoco", name: "ROS 2 + MuJoCo", summary: "MuJoCo stepping inside a ROS 2 node.", ros: "ros2", modules: &["mujoco"], components: &[] },
    Preset { id: "manipulation", name: "Manipulation", summary: "MoveIt 2 arm client with the Panda demo.", ros: "ros2", modules: &["moveit"], components: &[] },
    Preset { id: "navigation", name: "Navigation + SLAM", summary: "Nav2 with slam_toolbox on the TurtleBot3 simulation.", ros: "ros2", modules: &["nav2", "slam-toolbox"], components: &[] },
    Preset { id: "mixed", name: "Mixed mission", summary: "MoveIt, Nav2 + SLAM and a MAVSDK drone in one package, sequenced by a mission node.", ros: "ros2", modules: &["moveit", "nav2", "slam-toolbox", "px4-sitl", "mavsdk", "mission"], components: &[] },
    Preset { id: "full", name: "Aerial full stack", summary: "ROS 2, PX4, Gazebo, MuJoCo and PyTorch in one workspace.", ros: "ros2", modules: &["px4-offboard", "gazebo", "mujoco", "torch"], components: &[] },
    Preset { id: "isaac", name: "ROS 2 + Isaac Sim", summary: "ROS 2 workspace next to an Isaac Sim scene in its own environment.", ros: "ros2", modules: &["isaac"], components: &[] },
    Preset { id: "vision3d", name: "3D vision research", summary: "PyTorch, PyTorch3D and gsplat demos.", ros: "none", modules: &["pytorch3d", "gsplat"], components: &[] },
    Preset { id: "gs-slam", name: "Gaussian Splatting SLAM", summary: "ROS 2 SLAM node skeleton with gsplat, Gazebo and a MonoGS checkout.", ros: "ros2", modules: &["gazebo", "gs-slam"], components: &["monogs"] },
];

/// What a project is made of.
#[derive(Debug, Clone, Default)]
pub struct Selection {
    pub ros: Option<String>,
    pub modules: Vec<String>,
    pub components: Vec<String>,
    /// Existing ROS workspace to put the project's package into, instead of
    /// a new one inside the project folder.
    pub ros_workspace: Option<String>,
}

/// Build a selection from a preset. `ros2` is the ROS 2 distro component to
/// use when the preset wants ROS 2.
pub fn preset_selection(preset: &str, ros2: &str) -> Option<Selection> {
    PRESETS.iter().find(|p| p.id == preset).map(|p| Selection {
        ros: match p.ros {
            "ros2" => Some(ros2.to_string()),
            "ros1" => Some("ros1-noetic".to_string()),
            _ => None,
        },
        modules: p.modules.iter().map(|s| s.to_string()).collect(),
        components: p.components.iter().map(|s| s.to_string()).collect(),
        ros_workspace: None,
    })
}

/// The selection stored in a project manifest. Manifests written before
/// modules existed keep their ROS distro inside `components`.
pub fn project_selection(cat: &Catalog, proj: &Project) -> Selection {
    let mut components = proj.components.clone();
    let ros = proj.ros.clone().or_else(|| {
        let i = components.iter().position(|c| cat.get(c).map_or(false, |c| c.ros_version.is_some()))?;
        Some(components.remove(i))
    });
    Selection { ros, modules: proj.modules.clone(), components, ros_workspace: None }
}

pub fn project_components(cat: &Catalog, reg: &Registry, proj: &Project) -> Result<Vec<String>, String> {
    components_for(cat, reg, &project_selection(cat, proj))
}

fn ros_version(cat: &Catalog, sel: &Selection) -> Result<Option<u8>, String> {
    match &sel.ros {
        None => Ok(None),
        Some(id) => cat
            .get(id)
            .and_then(|c| c.ros_version)
            .map(Some)
            .ok_or_else(|| format!("{id} is not a ROS distro component")),
    }
}

/// Every SDK component the selection needs: the ROS distro, what each
/// module asks for (group requests fall back to their default), and extras.
pub fn components_for(cat: &Catalog, reg: &Registry, sel: &Selection) -> Result<Vec<String>, String> {
    let mods = reg.expand(&sel.modules)?;
    let ros = ros_version(cat, sel)?;
    for m in &mods {
        let need = match m.def.ros.as_str() {
            "ros2" => Some((2, "a ROS 2 distro")),
            "ros1" => Some((1, "ROS 1")),
            _ => None,
        };
        if let Some((v, what)) = need {
            if ros != Some(v) {
                return Err(format!("The {} module needs {what}. Pick one, or drop the module.", m.def.name));
            }
        }
    }
    let mut out: Vec<String> = Vec::new();
    let push = |id: &str, out: &mut Vec<String>| {
        if !out.iter().any(|x| x == id) {
            out.push(id.to_string());
        }
    };
    if let Some(r) = &sel.ros {
        push(r, &mut out);
    }
    // Explicit picks first, so a group request sees them.
    let explicit: Vec<&str> = sel
        .components
        .iter()
        .map(String::as_str)
        .chain(mods.iter().flat_map(|m| m.def.components.iter()).filter_map(|c| match parse_component(c) {
            (None, id) => Some(id),
            _ => None,
        }))
        .collect();
    let group_taken = |group: &str| explicit.iter().any(|id| cat.get(id).map_or(false, |c| c.group.as_deref() == Some(group)));
    for m in &mods {
        for c in &m.def.components {
            match parse_component(c) {
                (Some(group), default) => {
                    if !group_taken(group) {
                        push(default, &mut out);
                    }
                }
                (None, id) => push(id, &mut out),
            }
        }
    }
    for c in &sel.components {
        push(c, &mut out);
    }
    Ok(out)
}

/// Turn a project name into a valid ROS package / Python identifier.
pub fn package_name(name: &str) -> String {
    let mut s: String = name
        .chars()
        .map(|c| if c.is_ascii_alphanumeric() { c.to_ascii_lowercase() } else { '_' })
        .collect();
    while s.contains("__") {
        s = s.replace("__", "_");
    }
    let s = s.trim_matches('_').to_string();
    if s.is_empty() || s.chars().next().unwrap().is_ascii_digit() {
        format!("pkg_{s}")
    } else {
        s
    }
}

struct Ctx {
    pkg: String,
    name: String,
    ros: Option<u8>,
    distro: String,
    px4_branch: String,
    mods: BTreeSet<String>,
    ws: Vec<Workspace>,
}

impl Ctx {
    /// The concrete kind behind `ros` (`ros2` or `ros1`), if the project has ROS.
    fn kind(&self, kind: &str) -> Option<String> {
        match kind {
            "ros" | "ros2" | "ros1" => self.ros.map(|v| if v == 2 { "ros2".to_string() } else { "ros1".to_string() }),
            other => Some(other.to_string()),
        }
    }
    /// Where a workspace-relative path lands: project-relative for
    /// workspaces inside the project, absolute for reused external ones.
    fn loc(&self, kind: Option<&str>, rel: &str) -> String {
        let Some(w) = kind.and_then(|k| find_ws(&self.ws, k)) else { return rel.to_string() };
        if w.external() {
            w.abs(Path::new("")).join(rel).to_string_lossy().to_string()
        } else if w.path.is_empty() || w.path == "." {
            rel.to_string()
        } else {
            format!("{}/{rel}", w.path.trim_end_matches('/'))
        }
    }
    fn subst(&self, s: &str) -> String {
        s.replace("@PKGDIR@", &format!("src/{}", self.pkg))
            .replace("@PKG@", &self.pkg)
            .replace("@NAME@", &self.name)
            .replace("@ROS_DISTRO@", &self.distro)
            .replace("@PX4_BRANCH@", &self.px4_branch)
    }
    fn when(&self, tokens: &[String]) -> bool {
        tokens.iter().all(|t| {
            let (neg, t) = match t.strip_prefix('!') {
                Some(rest) => (true, rest),
                None => (false, t.as_str()),
            };
            let hit = match t {
                "ros2" => self.ros == Some(2),
                "ros1" => self.ros == Some(1),
                "noros" => self.ros.is_none(),
                other => other.strip_prefix("mod:").map_or(false, |id| self.mods.contains(id)),
            };
            hit != neg
        })
    }
    /// Workspace kind a module file belongs to; None = project root.
    /// Err = the file targets ROS but the project has none (skip it).
    fn file_ws(&self, f: &crate::modules::FileDef) -> Result<Option<String>, ()> {
        let want = f.workspace.clone().or_else(|| f.dest.contains("@PKGDIR@").then(|| "ros".to_string()));
        match want {
            None => Ok(None),
            Some(k) => self.kind(&k).map(Some).ok_or(()),
        }
    }
    /// Make sure every workspace the modules write to or run in exists in
    /// the list, adding missing kinds at their default location.
    fn ensure_workspaces(&mut self, mods: &[&Module]) {
        let mut kinds: Vec<String> = self.kind("ros").into_iter().collect();
        for m in mods {
            for f in m.def.files.iter().filter(|f| self.when(&f.when)) {
                kinds.extend(self.file_ws(f).ok().flatten());
            }
            for r in m.def.run.iter().filter(|r| self.when(&r.when)) {
                kinds.extend(r.workspace.as_deref().and_then(|k| self.kind(k)));
            }
        }
        for k in kinds {
            if find_ws(&self.ws, &k).is_none() {
                self.ws.push(Workspace { path: default_ws_path(&k), kind: k });
            }
        }
    }
}

struct Rendered {
    /// Written once; never overwritten, so user edits survive.
    files: Vec<(String, String)>,
    /// Package manifests and bringup launch: regenerated on module changes.
    managed: Vec<(String, String)>,
    run: Vec<RunConfig>,
}

fn py(s: &str) -> String {
    format!("'{}'", s.replace('\\', "\\\\").replace('\'', "\\'"))
}

fn xml(s: &str) -> String {
    s.replace('&', "&amp;").replace('<', "&lt;").replace('>', "&gt;")
}

fn bringup_launch(cx: &Ctx, entries: &[&LaunchDef]) -> String {
    let mut body = format!("        Node(package={}, executable='talker', output='screen'),\n", py(&cx.pkg));
    for l in entries {
        let arg = format!("use_{}", l.name.replace('-', "_"));
        body += &format!(
            "        DeclareLaunchArgument({}, default_value={}, description={}),\n",
            py(&arg),
            py(if l.default { "true" } else { "false" }),
            py(&cx.subst(&l.description))
        );
        let cond = format!("condition=IfCondition(LaunchConfiguration({}))", py(&arg));
        if let Some(n) = &l.node {
            body += &format!(
                "        Node(package={}, executable={}, output='screen', {cond}),\n",
                py(&cx.subst(&n.package)),
                py(&cx.subst(&n.executable))
            );
        } else if let Some(i) = &l.include {
            let parts: Vec<String> = i.file.split('/').map(|p| py(&cx.subst(p))).collect();
            let args: Vec<String> = i.args.iter().map(|(k, v)| format!("{}: {}", py(k), py(&cx.subst(v)))).collect();
            body += &format!(
                "        IncludeLaunchDescription(\n            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare({}), {}])),\n            launch_arguments={{{}}}.items(),\n            {cond},\n        ),\n",
                py(&cx.subst(&i.package)),
                parts.join(", "),
                args.join(", ")
            );
        }
    }
    format!(
"# Managed by RoboForge: regenerated when the project's modules change.
# Each module contributes an optional entry, switched on with use_<name>:=true.
# Put your own launch logic in a separate launch file.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
{body}    ])
")
}

fn render(cx: &Ctx, mods: &[&Module], extra_depends: &[String], existing_mains: &[String]) -> Rendered {
    let mut r = Rendered { files: vec![], managed: vec![], run: vec![] };
    let pkg = &cx.pkg;
    let ros_kind = cx.kind("ros");
    // Paths of the project's own package, wherever its ROS workspace is.
    let base = cx.loc(Some("ros"), &format!("src/{pkg}"));
    let run = |name: &str, cmd: String, ws: Option<String>, env: Option<String>, r: &mut Rendered| {
        if !r.run.iter().any(|x| x.name == name) {
            r.run.push(RunConfig { name: name.to_string(), cmd, workspace: ws, env });
        }
    };

    match cx.ros {
        Some(2) => {
            r.files.push((format!("{base}/resource/{pkg}"), String::new()));
            r.files.push((format!("{base}/{pkg}/__init__.py"), String::new()));
            r.files.push((format!("{base}/{pkg}/talker.py"), cx.subst(include_str!("ros2_talker.py"))));
            r.files.push((format!("{base}/test/test_import.py"), format!("def test_import():\n    import {pkg}.talker  # noqa: F401\n")));
            run("Build workspace (colcon)", "colcon build --symlink-install".into(), ros_kind.clone(), None, &mut r);
            run("Launch: bringup", format!("ros2 launch {pkg} bringup.launch.py"), ros_kind.clone(), None, &mut r);
            run("Test (colcon)", "colcon test && colcon test-result --verbose".into(), ros_kind.clone(), None, &mut r);
        }
        Some(_) => {
            r.files.push((format!("{base}/scripts/talker.py"), include_str!("ros1_talker.py").to_string()));
            run("Build workspace (catkin)", format!("chmod +x src/{pkg}/scripts/*.py && catkin_make"), ros_kind.clone(), None, &mut r);
            run("Launch: bringup", format!("roslaunch {pkg} bringup.launch"), ros_kind.clone(), None, &mut r);
        }
        None => {}
    }

    let mut depends: Vec<String> = vec!["rclpy".into(), "std_msgs".into()];
    let mut data_dirs: Vec<String> = Vec::new();
    let mut launch: Vec<&LaunchDef> = Vec::new();
    for m in mods {
        for f in m.def.files.iter().filter(|f| cx.when(&f.when)) {
            let Ok(ws) = cx.file_ws(f) else { continue };
            r.files.push((cx.loc(ws.as_deref(), &cx.subst(&f.dest)), cx.subst(&m.files[&f.src])));
        }
        for rc in m.def.run.iter().filter(|rc| cx.when(&rc.when)) {
            let ws = rc.workspace.as_deref().and_then(|k| cx.kind(k));
            run(&cx.subst(&rc.name), cx.subst(&rc.cmd), ws, rc.env.clone(), &mut r);
        }
        for d in &m.def.depends {
            if !depends.contains(d) {
                depends.push(d.clone());
            }
        }
        for d in &m.def.data_dirs {
            if !data_dirs.contains(d) {
                data_dirs.push(d.clone());
            }
        }
        launch.extend(m.def.launch.iter().filter(|l| cx.when(&l.when)));
    }
    for d in extra_depends {
        if !depends.contains(d) {
            depends.push(d.clone());
        }
    }

    if cx.ros == Some(2) {
        // Every module of the package that defines main() becomes a node, so
        // files added by hand are picked up on the next regeneration too.
        let prefix = format!("{base}/{pkg}/");
        let mut mains: BTreeSet<String> = existing_mains.iter().cloned().collect();
        for (path, body) in &r.files {
            if let Some(stem) = path.strip_prefix(&prefix).and_then(|p| p.strip_suffix(".py")) {
                if !stem.contains('/') && body.contains("\ndef main(") {
                    mains.insert(stem.to_string());
                }
            }
        }
        let entries: Vec<String> = mains.iter().map(|m| format!("'{m} = {pkg}.{m}:main'")).collect();
        let mut data = vec!["(os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))".to_string()];
        data.extend(data_dirs.iter().map(|d| format!("(os.path.join('share', package_name, {}), glob({}))", py(d), py(&format!("{d}/*")))));
        let dep_xml: String = depends.iter().map(|d| format!("  <exec_depend>{}</exec_depend>\n", xml(d))).collect();
        r.managed.push((format!("{base}/package.xml"), format!(
"<?xml version=\"1.0\"?>
<!-- Managed by RoboForge: regenerated when the project's modules change.
     Add your own dependencies to \"depends\" in roboforge.json. -->
<package format=\"3\">
  <name>{pkg}</name>
  <version>0.1.0</version>
  <description>{}</description>
  <maintainer email=\"you@example.com\">you</maintainer>
  <license>Apache-2.0</license>
{dep_xml}  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>
  <test_depend>python3-pytest</test_depend>
  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
", xml(&cx.name))));
        r.managed.push((format!("{base}/setup.py"), format!(
"# Managed by RoboForge: regenerated when the project's modules change.
# Every module in {pkg}/ that defines main() is registered as a node.
import os
from glob import glob
from setuptools import find_packages, setup

package_name = '{pkg}'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        {},
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={{
        'console_scripts': [
            {},
        ],
    }},
)
", data.join(",\n        "), entries.join(",\n            "))));
        r.managed.push((format!("{base}/setup.cfg"), format!("[develop]\nscript_dir=$base/lib/{pkg}\n[install]\ninstall_scripts=$base/lib/{pkg}\n")));
        r.managed.push((format!("{base}/launch/bringup.launch.py"), bringup_launch(cx, &launch)));
    } else if cx.ros == Some(1) {
        r.managed.push((format!("{base}/package.xml"), format!(
"<?xml version=\"1.0\"?>
<package format=\"2\">
  <name>{pkg}</name>
  <version>0.1.0</version>
  <description>{}</description>
  <maintainer email=\"you@example.com\">you</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>catkin</buildtool_depend>
  <depend>rospy</depend>
  <depend>std_msgs</depend>
</package>
", xml(&cx.name))));
        r.managed.push((format!("{base}/CMakeLists.txt"), format!(
"cmake_minimum_required(VERSION 3.0.2)
project({pkg})
find_package(catkin REQUIRED COMPONENTS rospy std_msgs)
catkin_package()
file(GLOB NODE_SCRIPTS scripts/*.py)
catkin_install_python(PROGRAMS ${{NODE_SCRIPTS}} DESTINATION ${{CATKIN_PACKAGE_BIN_DESTINATION}})
install(DIRECTORY launch DESTINATION ${{CATKIN_PACKAGE_SHARE_DESTINATION}})
")));
        r.managed.push((format!("{base}/launch/bringup.launch"), format!(
            "<launch>\n  <node pkg=\"{pkg}\" type=\"talker.py\" name=\"talker\" output=\"screen\"/>\n</launch>\n")));
    }
    r
}

fn context(cat: &Catalog, name: &str, sel: &Selection, comps: &[String], mods: &[&Module], ws: Vec<Workspace>) -> Result<Ctx, String> {
    let ros = ros_version(cat, sel)?;
    let distro = sel.ros.as_ref().and_then(|r| cat.get(r)).and_then(|c| c.distro.clone()).unwrap_or_default();
    let px4 = comps.iter().filter_map(|c| cat.get(c)).find(|c| c.group.as_deref() == Some("px4"));
    let mut cx = Ctx {
        pkg: package_name(name),
        name: name.to_string(),
        ros,
        distro,
        px4_branch: px4.map(|c| format!("release/{}", c.version)).unwrap_or_else(|| "main".into()),
        mods: mods.iter().map(|m| m.def.id.clone()).collect(),
        ws,
    };
    cx.ensure_workspaces(mods);
    Ok(cx)
}

fn write_file(dir: &Path, rel: &str, body: &str) -> Result<(), String> {
    let full = dir.join(rel);
    if let Some(parent) = full.parent() {
        std::fs::create_dir_all(parent).map_err(|e| format!("{}: {e}", parent.display()))?;
    }
    std::fs::write(&full, body).map_err(|e| format!("{}: {e}", full.display()))
}

fn readme(name: &str, mods: &[&Module], comps: &[String], cat: &Catalog, ws: &[Workspace], run: &[RunConfig]) -> String {
    let mut s = format!("# {name}\n\nGenerated by RoboForge. SDKs are pinned in `roboforge.json` and installed in isolation under `~/.roboforge`, never into the system.\n");
    if !ws.is_empty() {
        s += "\n## Workspaces\n\nEach kind of work has its own folder, listed under `workspaces` in `roboforge.json`. Point a path at an existing workspace to reuse it.\n\n";
        for w in ws {
            let what = match w.kind.as_str() {
                "ros2" => "colcon workspace: ROS 2 packages in `src/`",
                "ros1" => "catkin workspace: ROS 1 packages in `src/`",
                "mav" => "standalone MAVSDK / PX4 applications, no ROS",
                "sim" => "simulator models, worlds and scripts outside ROS",
                "ml" => "training and research code",
                "isaac" => "Isaac Sim apps, run in the dedicated Isaac environment",
                _ => "custom workspace",
            };
            s += &format!("- `{}` ({}): {what}\n", w.path, w.kind);
        }
    }
    if !mods.is_empty() {
        s += "\n## Modules\n\n";
        for m in mods {
            s += &format!("- **{}** ({}): {}\n", m.def.name, m.def.id, m.def.summary);
        }
    }
    s += "\n## SDK components\n\n";
    for c in comps.iter().filter_map(|c| cat.get(c)) {
        s += &format!("- **{}** ({}): {}\n", c.name, c.id, c.summary);
    }
    s += "\n## Run configurations\n\n";
    for r in run {
        let at = r.workspace.as_deref().map(|w| format!(" (in the {w} workspace)")).unwrap_or_default();
        s += &format!("- **{}**: `{}`{at}\n", r.name, r.cmd.replace('\n', " "));
    }
    s += "\n## Outside the IDE\n\n```bash\nroboforge sync              # install whatever SDKs are missing\nroboforge add <module>      # mix another module into this project\nsource .roboforge/activate.sh   # also sets RF_WS_ROS, RF_WS_MAV, ... to the workspace folders\nroboforge run \"<config>\"\n```\n";
    s
}

/// Create the workspace folders. When the ROS workspace is the project
/// root itself, mark the other folders so colcon does not crawl them.
fn make_workspaces(dir: &Path, ws: &[Workspace]) -> Result<(), String> {
    let ros_at_root = find_ws(ws, "ros").map_or(false, |w| !w.external() && (w.path.is_empty() || w.path == "."));
    for w in ws {
        let d = w.abs(dir);
        std::fs::create_dir_all(&d).map_err(|e| format!("{}: {e}", w.path))?;
        if ros_at_root && !w.is_ros() && !w.external() && !d.join("COLCON_IGNORE").exists() {
            let _ = std::fs::write(d.join("COLCON_IGNORE"), "");
        }
    }
    Ok(())
}

const GITIGNORE: &str = "build/\ninstall/\nlog/\ndevel/\n.catkin_tools/\n__pycache__/\n*.pyc\n.roboforge/activate*.sh\noutputs/\nmaps/\n";

/// Create a new project in an empty or missing directory.
pub fn generate(home: &Home, cat: &Catalog, reg: &Registry, dir: &Path, name: &str, sel: &Selection) -> Result<Project, String> {
    let name = name.trim();
    if name.is_empty() {
        return Err("project name is empty".into());
    }
    let comps = components_for(cat, reg, sel)?;
    resolve(cat, &comps)?;
    if dir.exists() && std::fs::read_dir(dir).map(|mut d| d.next().is_some()).unwrap_or(true) {
        return Err(format!("{} already exists and is not empty", dir.display()));
    }
    let mods = reg.expand(&sel.modules)?;
    let mut ws = Vec::new();
    if let Some(path) = sel.ros_workspace.as_deref().map(str::trim).filter(|p| !p.is_empty()) {
        let kind = match ros_version(cat, sel)? {
            Some(2) => "ros2",
            Some(_) => "ros1",
            None => return Err("An existing ROS workspace was given, but the project has no ROS distro.".into()),
        };
        ws.push(Workspace { kind: kind.into(), path: path.to_string() });
    }
    let cx = context(cat, name, sel, &comps, &mods, ws)?;
    if let Some(w) = find_ws(&cx.ws, "ros") {
        let pkg_dir = w.abs(dir).join("src").join(&cx.pkg);
        if pkg_dir.exists() {
            return Err(format!("{} already exists. Pick another project name; the package is named after it.", pkg_dir.display()));
        }
    }
    let r = render(&cx, &mods, &[], &[]);
    for (path, body) in r.files.iter().chain(r.managed.iter()) {
        write_file(dir, path, body)?;
    }
    make_workspaces(dir, &cx.ws)?;
    write_file(dir, ".gitignore", GITIGNORE)?;
    write_file(dir, "README.md", &readme(name, &mods, &comps, cat, &cx.ws, &r.run))?;
    let proj = Project {
        name: name.to_string(),
        ros: sel.ros.clone(),
        modules: mods.iter().map(|m| m.def.id.clone()).collect(),
        workspaces: cx.ws.clone(),
        components: sel.components.clone(),
        depends: vec![],
        env: default_env(cx.ros),
        run: r.run,        setup: Default::default(),
        launch: None,
    };
    proj.save(dir)?;
    write_activation(home, cat, dir, &proj, &comps)?;
    Ok(proj)
}

/// Sensible per-project defaults; edit them in roboforge.json.
pub fn default_env(ros: Option<u8>) -> std::collections::BTreeMap<String, String> {
    let mut m = std::collections::BTreeMap::new();
    if ros == Some(2) {
        m.insert("ROS_DOMAIN_ID".into(), "0".into());
        m.insert("ROS_LOCALHOST_ONLY".into(), "0".into());
    }
    m
}

/// Put a RoboForge config into an existing ROS workspace without touching
/// its packages: the folder itself becomes the project and its ROS
/// workspace. Modules can be mixed in afterwards.
pub fn adopt(home: &Home, cat: &Catalog, reg: &Registry, dir: &Path, name: &str, ros: &str) -> Result<Project, String> {
    if dir.join(MANIFEST).exists() {
        return Err(format!("{} already has a roboforge.json; open it instead.", dir.display()));
    }
    if !dir.join("src").is_dir() {
        return Err(format!("{} has no src/ folder, so it does not look like a ROS workspace.", dir.display()));
    }
    let sel = Selection { ros: Some(ros.to_string()), ..Default::default() };
    let comps = components_for(cat, reg, &sel)?;
    let v2 = ros_version(cat, &sel)? == Some(2);
    let kind = if v2 { "ros2" } else { "ros1" };
    let rc = |name: &str, cmd: &str| RunConfig { name: name.into(), cmd: cmd.into(), workspace: Some(kind.into()), env: None };
    let run = if v2 {
        vec![rc("Build workspace (colcon)", "colcon build --symlink-install"), rc("Test (colcon)", "colcon test && colcon test-result --verbose")]
    } else {
        vec![rc("Build workspace (catkin)", "catkin_make")]
    };
    let name = if name.trim().is_empty() { dir.file_name().map(|n| n.to_string_lossy().to_string()).unwrap_or_else(|| "workspace".into()) } else { name.trim().to_string() };
    let proj = Project {
        name,
        ros: sel.ros.clone(),
        modules: vec![],
        workspaces: vec![Workspace { kind: kind.into(), path: ".".into() }],
        components: vec![],
        depends: vec![],
        env: default_env(if v2 { Some(2) } else { Some(1) }),
        run,        setup: Default::default(),
        launch: None,
    };
    proj.save(dir)?;
    write_activation(home, cat, dir, &proj, &comps)?;
    Ok(proj)
}

/// Names of the ROS packages found under a workspace's `src/`.
pub fn ros_packages(ws_dir: &Path) -> Vec<String> {
    fn scan(dir: &Path, depth: u32, out: &mut Vec<String>) {
        if dir.join("package.xml").is_file() {
            out.push(dir.file_name().map(|n| n.to_string_lossy().to_string()).unwrap_or_default());
            return;
        }
        if depth == 0 {
            return;
        }
        for e in std::fs::read_dir(dir).into_iter().flatten().flatten() {
            if e.path().is_dir() && !e.file_name().to_string_lossy().starts_with('.') {
                scan(&e.path(), depth - 1, out);
            }
        }
    }
    let mut out = Vec::new();
    scan(&ws_dir.join("src"), 4, &mut out);
    out.sort();
    out
}

#[derive(Debug, Clone, Serialize)]
pub struct ModuleChange {
    pub project: Project,
    /// Files created for newly added modules.
    pub written: Vec<String>,
    /// Files of removed modules that were left in place.
    pub left: Vec<String>,
}

fn mains_on_disk(dir: &Path, cx: &Ctx) -> Vec<String> {
    let pkg_dir = dir.join(cx.loc(Some("ros"), &format!("src/{}/{}", cx.pkg, cx.pkg)));
    std::fs::read_dir(pkg_dir)
        .into_iter()
        .flatten()
        .flatten()
        .filter_map(|e| {
            let p = e.path();
            let stem = p.file_stem()?.to_string_lossy().to_string();
            if p.extension()? != "py" {
                return None;
            }
            let text = std::fs::read_to_string(&p).ok()?;
            (text.starts_with("def main(") || text.contains("\ndef main(")).then_some(stem)
        })
        .collect()
}

/// A project's workspaces; manifests from before workspaces existed keep
/// their ROS packages at the project root.
fn project_workspaces(proj: &Project, ros: Option<u8>) -> Vec<Workspace> {
    if !proj.workspaces.is_empty() {
        return proj.workspaces.clone();
    }
    match ros {
        Some(v) => vec![Workspace { kind: if v == 2 { "ros2".into() } else { "ros1".into() }, path: ".".into() }],
        None => vec![],
    }
}

/// Change the module set of an existing project. New modules' files are
/// created (existing files are never overwritten), package manifests and
/// the bringup launch are regenerated, and run configurations are merged.
pub fn set_modules(home: &Home, cat: &Catalog, reg: &Registry, dir: &Path, modules: &[String]) -> Result<ModuleChange, String> {
    let mut proj = Project::load(dir)?;
    let old_sel = project_selection(cat, &proj);
    let new_sel = Selection { modules: modules.to_vec(), ..old_sel.clone() };
    let comps = components_for(cat, reg, &new_sel)?;
    resolve(cat, &comps)?;
    let ws = project_workspaces(&proj, ros_version(cat, &new_sel)?);

    let new_mods = reg.expand(&new_sel.modules)?;
    let cx = context(cat, &proj.name, &new_sel, &comps, &new_mods, ws.clone())?;
    let new = render(&cx, &new_mods, &proj.depends, &mains_on_disk(dir, &cx));

    // What the previous module set had contributed. Unknown ids (a user
    // module that has since been deleted) are simply skipped.
    let known: Vec<String> = old_sel.modules.iter().filter(|m| reg.get(m).is_some()).cloned().collect();
    let old_mods = reg.expand(&known)?;
    let old_comps = components_for(cat, reg, &Selection { modules: known, ..old_sel.clone() }).unwrap_or_default();
    let old_cx = context(cat, &proj.name, &old_sel, &old_comps, &old_mods, cx.ws.clone())?;
    let old = render(&old_cx, &old_mods, &[], &[]);

    let mut written = Vec::new();
    for (path, body) in &new.files {
        if !dir.join(path).exists() {
            write_file(dir, path, body)?;
            written.push(path.clone());
        }
    }
    for (path, body) in &new.managed {
        write_file(dir, path, body)?;
    }
    make_workspaces(dir, &cx.ws)?;
    let left: Vec<String> = old
        .files
        .iter()
        .map(|(p, _)| p.clone())
        .filter(|p| !new.files.iter().any(|(n, _)| n == p) && dir.join(p).exists())
        .collect();

    // Drop run configurations that only the removed modules provided; keep
    // everything the user wrote or edited; append the new ones.
    let old_names: BTreeSet<&str> = old.run.iter().map(|r| r.name.as_str()).collect();
    let new_names: BTreeSet<&str> = new.run.iter().map(|r| r.name.as_str()).collect();
    proj.run.retain(|r| !old_names.contains(r.name.as_str()) || new_names.contains(r.name.as_str()));
    for rc in &new.run {
        if !proj.run.iter().any(|r| r.name == rc.name) {
            proj.run.push(rc.clone());
        }
    }
    proj.ros = new_sel.ros.clone();
    proj.components = new_sel.components.clone();
    proj.modules = new_mods.iter().map(|m| m.def.id.clone()).collect();
    proj.workspaces = cx.ws.clone();
    proj.save(dir)?;
    write_activation(home, cat, dir, &proj, &comps)?;
    Ok(ModuleChange { project: proj, written, left })
}

/// Replace a project's workspace list. Folders are created, non-ROS folders
/// inside a root-level ROS workspace get COLCON_IGNORE, and activation is
/// rewritten. The first ROS workspace keeps hosting the project's package.
pub fn set_workspaces(home: &Home, cat: &Catalog, reg: &Registry, dir: &Path, list: &[Workspace]) -> Result<Project, String> {
    let mut proj = Project::load(dir)?;
    let ros = ros_version(cat, &project_selection(cat, &proj))?;
    let mut seen = BTreeSet::new();
    for w in list {
        if w.kind.is_empty() || !w.kind.chars().all(|c| c.is_ascii_alphanumeric() || c == '-' || c == '_') {
            return Err(format!("invalid workspace kind {:?}", w.kind));
        }
        if w.path.trim().is_empty() {
            return Err(format!("the {} workspace has no path", w.kind));
        }
        match (w.kind.as_str(), ros) {
            ("ros2", Some(2)) | ("ros1", Some(1)) => {}
            ("ros2" | "ros1", _) => return Err(format!("a {} workspace does not match this project's ROS choice", w.kind)),
            _ => {}
        }
        if !seen.insert(w.abs(dir)) {
            return Err(format!("{} is listed twice", w.path));
        }
    }
    if let (Some(old), Some(new)) = (find_ws(&proj.workspaces, "ros"), find_ws(list, "ros")) {
        if old.abs(dir) != new.abs(dir) && old.abs(dir).join("src").join(package_name(&proj.name)).exists() {
            return Err(format!("The project's package lives in {}. Move it before changing the first ROS workspace, or add the new one after it.", old.path));
        }
    }
    make_workspaces(dir, list)?;
    proj.workspaces = list.to_vec();
    proj.save(dir)?;
    let comps = project_components(cat, reg, &proj)?;
    write_activation(home, cat, dir, &proj, &comps)?;
    Ok(proj)
}

/// Create a new ROS package in a ROS workspace of the project.
/// `kind`: `ament_python`, `ament_cmake` (ROS 2) or `catkin` (ROS 1, inferred).
pub fn new_package(dir: &Path, ws_index: usize, name: &str, kind: &str) -> Result<Vec<String>, String> {
    let proj = Project::load(dir)?;
    let w = proj.workspaces.get(ws_index).ok_or_else(|| format!("no workspace #{ws_index}"))?;
    if !w.is_ros() {
        return Err(format!("workspace {} is not a ROS workspace", w.path));
    }
    let pkg = package_name(name);
    let root = w.abs(dir).join("src").join(&pkg);
    if root.exists() {
        return Err(format!("{} already exists", root.display()));
    }
    let kind = if w.kind == "ros1" { "catkin" } else if kind == "ament_cmake" { "ament_cmake" } else { "ament_python" };
    let mut files: Vec<(String, String)> = Vec::new();
    let node_py = include_str!("ros2_talker.py").replace("@NAME@", &pkg).replace("@PKG@", &pkg);
    match kind {
        "ament_python" => {
            files.push(("package.xml".into(), format!(
"<?xml version=\"1.0\"?>
<package format=\"3\">
  <name>{pkg}</name>
  <version>0.1.0</version>
  <description>{pkg}</description>
  <maintainer email=\"you@example.com\">you</maintainer>
  <license>Apache-2.0</license>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>
  <test_depend>python3-pytest</test_depend>
  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
")));
            files.push(("setup.py".into(), format!(
"import os
from glob import glob
from setuptools import find_packages, setup

package_name = '{pkg}'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={{
        'console_scripts': [
            '{pkg}_node = {pkg}.{pkg}_node:main',
        ],
    }},
)
")));
            files.push(("setup.cfg".into(), format!("[develop]\nscript_dir=$base/lib/{pkg}\n[install]\ninstall_scripts=$base/lib/{pkg}\n")));
            files.push((format!("resource/{pkg}"), String::new()));
            files.push((format!("{pkg}/__init__.py"), String::new()));
            files.push((format!("{pkg}/{pkg}_node.py"), node_py));
            files.push(("test/test_import.py".into(), format!("def test_import():\n    import {pkg}.{pkg}_node  # noqa: F401\n")));
            files.push((format!("launch/{pkg}.launch.py"), format!(
"from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='{pkg}', executable='{pkg}_node', output='screen'),
    ])
")));
        }
        "ament_cmake" => {
            files.push(("package.xml".into(), format!(
"<?xml version=\"1.0\"?>
<package format=\"3\">
  <name>{pkg}</name>
  <version>0.1.0</version>
  <description>{pkg}</description>
  <maintainer email=\"you@example.com\">you</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
")));
            files.push(("CMakeLists.txt".into(), format!(
"cmake_minimum_required(VERSION 3.8)
project({pkg})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES \"Clang\")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable({pkg}_node src/{pkg}_node.cpp)
ament_target_dependencies({pkg}_node rclcpp std_msgs)

install(TARGETS {pkg}_node DESTINATION lib/${{PROJECT_NAME}})
install(DIRECTORY launch DESTINATION share/${{PROJECT_NAME}})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
")));
            files.push((format!("src/{pkg}_node.cpp"), format!(
"#include <chrono>
#include <memory>
#include <string>

#include \"rclcpp/rclcpp.hpp\"
#include \"std_msgs/msg/string.hpp\"

using namespace std::chrono_literals;

class Talker : public rclcpp::Node {{
public:
  Talker() : Node(\"{pkg}_node\") {{
    pub_ = create_publisher<std_msgs::msg::String>(\"chatter\", 10);
    timer_ = create_wall_timer(1s, [this]() {{
      std_msgs::msg::String msg;
      msg.data = \"{pkg} says hello #\" + std::to_string(count_++);
      RCLCPP_INFO(get_logger(), \"%s\", msg.data.c_str());
      pub_->publish(msg);
    }});
  }}

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_ = 0;
}};

int main(int argc, char ** argv) {{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Talker>());
  rclcpp::shutdown();
  return 0;
}}
")));
            files.push((format!("launch/{pkg}.launch.py"), format!(
"from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='{pkg}', executable='{pkg}_node', output='screen'),
    ])
")));
        }
        _ => {
            files.push(("package.xml".into(), format!(
"<?xml version=\"1.0\"?>
<package format=\"2\">
  <name>{pkg}</name>
  <version>0.1.0</version>
  <description>{pkg}</description>
  <maintainer email=\"you@example.com\">you</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>catkin</buildtool_depend>
  <depend>rospy</depend>
  <depend>std_msgs</depend>
</package>
")));
            files.push(("CMakeLists.txt".into(), format!(
"cmake_minimum_required(VERSION 3.0.2)
project({pkg})
find_package(catkin REQUIRED COMPONENTS rospy std_msgs)
catkin_package()
file(GLOB NODE_SCRIPTS scripts/*.py)
catkin_install_python(PROGRAMS ${{NODE_SCRIPTS}} DESTINATION ${{CATKIN_PACKAGE_BIN_DESTINATION}})
install(DIRECTORY launch DESTINATION ${{CATKIN_PACKAGE_SHARE_DESTINATION}})
")));
            files.push((format!("scripts/{pkg}_node.py"), include_str!("ros1_talker.py").to_string()));
            files.push((format!("launch/{pkg}.launch"), format!(
                "<launch>\n  <node pkg=\"{pkg}\" type=\"{pkg}_node.py\" name=\"{pkg}_node\" output=\"screen\"/>\n</launch>\n")));
        }
    }
    let mut written = Vec::new();
    for (rel, body) in files {
        let full = root.join(&rel);
        if let Some(p) = full.parent() {
            std::fs::create_dir_all(p).map_err(|e| format!("{}: {e}", p.display()))?;
        }
        std::fs::write(&full, body).map_err(|e| format!("{}: {e}", full.display()))?;
        written.push(full.to_string_lossy().to_string());
    }
    Ok(written)
}
