use roboforge_core::catalog::Catalog;
use roboforge_core::paths::Home;
use roboforge_core::plan::{mark_done, plan, UnitKey};
use roboforge_core::project::{wrap, Project, Workspace};
use roboforge_core::resolve::resolve;
use roboforge_core::state::State;
use roboforge_core::modules::Registry;
use roboforge_core::templates::{components_for, generate, package_name, preset_selection, project_components, set_modules, set_workspaces, new_package, adopt, ros_packages, Selection, PRESETS};
use std::path::PathBuf;

fn tmp(tag: &str) -> PathBuf {
    let d = std::env::temp_dir().join(format!("rf-test-{}-{tag}", std::process::id()));
    let _ = std::fs::remove_dir_all(&d);
    std::fs::create_dir_all(&d).unwrap();
    d
}

fn ids(v: &[&str]) -> Vec<String> {
    v.iter().map(|s| s.to_string()).collect()
}

fn sel(ros: Option<&str>, modules: &[&str], extra: &[&str]) -> Selection {
    Selection { ros: ros.map(String::from), modules: ids(modules), components: ids(extra), ros_workspace: None }
}

#[test]
fn embedded_catalog_and_modules_are_valid() {
    let cat = Catalog::embedded();
    cat.validate().unwrap();
    let reg = Registry::builtin();
    reg.validate(&cat).unwrap();
    assert!(reg.modules.len() >= 15);
    for p in PRESETS {
        let s = preset_selection(p.id, "ros2-jazzy").unwrap();
        let comps = components_for(&cat, &reg, &s).unwrap_or_else(|e| panic!("preset {}: {e}", p.id));
        resolve(&cat, &comps).unwrap_or_else(|e| panic!("preset {}: {e}", p.id));
    }
}

#[test]
fn modules_mix_and_bring_their_sdks() {
    let cat = Catalog::embedded();
    let reg = Registry::builtin();
    // MoveIt + Nav2/SLAM + MAVSDK in one selection.
    let comps = components_for(&cat, &reg, &sel(Some("ros2-humble"), &["moveit", "nav2", "slam-toolbox", "mavsdk", "mission"], &[])).unwrap();
    assert_eq!(comps, ids(&["ros2-humble", "moveit", "nav2", "slam-toolbox", "mavsdk"]));
    let env = resolve(&cat, &comps).unwrap().main_env.unwrap();
    assert_eq!(env.components.len(), 5, "one shared environment");

    // Required modules are pulled in; a group request takes its default...
    let comps = components_for(&cat, &reg, &sel(Some("ros2-jazzy"), &["px4-offboard"], &[])).unwrap();
    assert_eq!(comps, ids(&["ros2-jazzy", "px4-1.16", "gazebo", "microxrce-agent"]));
    // ...unless the project already picked a member of that group.
    let comps = components_for(&cat, &reg, &sel(Some("ros2-jazzy"), &["px4-offboard", "gsplat"], &["px4-1.15", "torch-cpu"])).unwrap();
    assert!(comps.contains(&"px4-1.15".to_string()) && !comps.contains(&"px4-1.16".to_string()));
    assert!(comps.contains(&"torch-cpu".to_string()) && !comps.contains(&"torch-cu128".to_string()));

    // ROS-only modules explain themselves without ROS; MAVSDK works without.
    let err = components_for(&cat, &reg, &sel(None, &["moveit"], &[])).unwrap_err();
    assert!(err.contains("MoveIt 2") && err.contains("ROS 2"), "{err}");
    let err = components_for(&cat, &reg, &sel(Some("ros1-noetic"), &["nav2"], &[])).unwrap_err();
    assert!(err.contains("ROS 2"), "{err}");
    assert_eq!(components_for(&cat, &reg, &sel(None, &["mavsdk"], &[])).unwrap(), ids(&["mavsdk"]));
    assert!(components_for(&cat, &reg, &sel(None, &["nope"], &[])).is_err());
}

#[test]
fn modules_can_be_added_and_removed_later() {
    let home = Home { root: tmp("mod-home") };
    let cat = Catalog::embedded();
    let reg = Registry::builtin();
    let dir = tmp("mod-out").join("bot");
    generate(&home, &cat, &reg, &dir, "Bot", &sel(Some("ros2-jazzy"), &["nav2"], &[])).unwrap();
    let pkg = dir.join("ros2_ws/src/bot");
    assert!(pkg.join("bot/nav_goal.py").exists());
    assert!(!std::fs::read_to_string(pkg.join("setup.py")).unwrap().contains("moveit_goal"));

    // The user edits a module file, adds a node and a run configuration.
    std::fs::write(pkg.join("bot/nav_goal.py"), "# mine\ndef main():\n    pass\n").unwrap();
    std::fs::write(pkg.join("bot/my_node.py"), "def main():\n    pass\n").unwrap();
    let mut p = Project::load(&dir).unwrap();
    p.run.push(roboforge_core::project::RunConfig { name: "Mine".into(), cmd: "echo mine".into(), workspace: None, env: None });
    p.depends.push("tf2_ros".into());
    p.save(&dir).unwrap();

    let ch = set_modules(&home, &cat, &reg, &dir, &ids(&["nav2", "moveit", "mavsdk", "mission", "slam-toolbox"])).unwrap();
    assert!(ch.written.contains(&"ros2_ws/src/bot/bot/moveit_goal.py".to_string()));
    assert!(ch.written.contains(&"ros2_ws/src/bot/bot/mission.py".to_string()));
    assert!(!ch.written.contains(&"ros2_ws/src/bot/bot/nav_goal.py".to_string()), "existing files are not rewritten");
    assert!(std::fs::read_to_string(pkg.join("bot/nav_goal.py")).unwrap().starts_with("# mine"));
    let setup = std::fs::read_to_string(pkg.join("setup.py")).unwrap();
    for node in ["moveit_goal", "mavsdk_bridge", "mission", "nav_goal", "my_node", "talker"] {
        assert!(setup.contains(&format!("'{node} = bot.{node}:main'")), "{node} registered");
    }
    let xml = std::fs::read_to_string(pkg.join("package.xml")).unwrap();
    assert!(xml.contains("moveit_msgs") && xml.contains("nav2_simple_commander") && xml.contains("tf2_ros"));
    let launch = std::fs::read_to_string(pkg.join("launch/bringup.launch.py")).unwrap();
    assert!(launch.contains("'use_mavsdk'") && launch.contains("'use_slam'") && launch.contains("FindPackageShare('slam_toolbox')"));
    let names: Vec<String> = ch.project.run.iter().map(|r| r.name.clone()).collect();
    assert!(names.contains(&"Mine".to_string()));
    assert!(names.contains(&"Nav2 + SLAM: TurtleBot3 simulation".to_string()), "cross-module config appears");
    assert_eq!(project_components(&cat, &reg, &ch.project).unwrap(), ids(&["ros2-jazzy", "nav2", "moveit", "mavsdk", "slam-toolbox"]));

    // Removing a module drops its run configs and deps but leaves its files.
    let ch = set_modules(&home, &cat, &reg, &dir, &ids(&["nav2", "mavsdk"])).unwrap();
    let names: Vec<String> = ch.project.run.iter().map(|r| r.name.clone()).collect();
    assert!(!names.iter().any(|n| n.starts_with("MoveIt")) && !names.iter().any(|n| n.contains("SLAM")));
    assert!(names.contains(&"Mine".to_string()) && names.contains(&"Nav2: send goal".to_string()));
    assert!(ch.left.contains(&"ros2_ws/src/bot/bot/moveit_goal.py".to_string()));
    assert!(pkg.join("bot/moveit_goal.py").exists());
    assert!(!std::fs::read_to_string(pkg.join("package.xml")).unwrap().contains("moveit_msgs"));
    // An impossible change is refused and leaves the project untouched.
    let before = std::fs::read_to_string(dir.join("roboforge.json")).unwrap();
    assert!(set_modules(&home, &cat, &reg, &dir, &ids(&["nope"])).is_err());
    assert_eq!(before, std::fs::read_to_string(dir.join("roboforge.json")).unwrap());
    std::fs::remove_dir_all(&home.root).unwrap();
    std::fs::remove_dir_all(dir.parent().unwrap()).unwrap();
}

#[test]
fn user_modules_load_from_home_and_are_validated() {
    let home = Home { root: tmp("user-mod") };
    let cat = Catalog::embedded();
    let d = home.modules_dir().join("lidar-driver");
    std::fs::create_dir_all(d.join("files")).unwrap();
    std::fs::write(d.join("files/driver.py"), "NAME = '@NAME@'\n\ndef main():\n    pass\n").unwrap();
    std::fs::write(d.join("module.json"), r#"{"id":"lidar-driver","name":"Lidar driver","category":"drivers","ros":"ros2","components":["open3d"],
        "depends":["sensor_msgs"],"files":[{"src":"driver.py","dest":"@PKGDIR@/@PKG@/lidar_driver.py"}],
        "run":[{"name":"Lidar: driver","cmd":"ros2 run @PKG@ lidar_driver"}]}"#).unwrap();
    let reg = Registry::load(&home, &cat).unwrap();
    assert!(!reg.get("lidar-driver").unwrap().def.builtin);
    let out = tmp("user-mod-out").join("p");
    let proj = generate(&home, &cat, &reg, &out, "Scanner", &sel(Some("ros2-jazzy"), &["lidar-driver", "nav2"], &[])).unwrap();
    assert!(std::fs::read_to_string(out.join("ros2_ws/src/scanner/scanner/lidar_driver.py")).unwrap().contains("NAME = 'Scanner'"));
    assert!(proj.run.iter().any(|r| r.cmd == "ros2 run scanner lidar_driver"));
    // A broken user module is reported, not silently ignored.
    std::fs::write(d.join("module.json"), r#"{"id":"lidar-driver","name":"x","category":"drivers","components":["no-such-sdk"]}"#).unwrap();
    let err = Registry::load(&home, &cat).unwrap_err();
    assert!(err.contains("lidar-driver") && err.contains("no-such-sdk"), "{err}");
    std::fs::remove_dir_all(&home.root).unwrap();
    std::fs::remove_dir_all(out.parent().unwrap()).unwrap();
}

#[test]
fn two_ros_distros_cannot_share_an_env() {
    let cat = Catalog::embedded();
    let err = resolve(&cat, &ids(&["ros2-humble", "ros2-jazzy"])).unwrap_err();
    assert!(err.contains("cannot share"), "{err}");
    let err = resolve(&cat, &ids(&["ros1-noetic", "ros2-jazzy"])).unwrap_err();
    assert!(err.contains("cannot share"), "{err}");
}

#[test]
fn group_requirement_must_be_satisfied() {
    let cat = Catalog::embedded();
    let err = resolve(&cat, &ids(&["gsplat"])).unwrap_err();
    assert!(err.contains("torch-cu128") && err.contains("torch-cpu"), "{err}");
    let res = resolve(&cat, &ids(&["gsplat", "torch-cpu"])).unwrap();
    assert_eq!(res.components, ids(&["torch-cpu", "gsplat"]));
    assert!(resolve(&cat, &ids(&["nope"])).is_err());
}

#[test]
fn different_selections_get_different_envs() {
    let cat = Catalog::embedded();
    let a = resolve(&cat, &ids(&["ros2-humble", "mujoco"])).unwrap().main_env.unwrap();
    let b = resolve(&cat, &ids(&["ros2-jazzy", "mujoco"])).unwrap().main_env.unwrap();
    let a2 = resolve(&cat, &ids(&["mujoco", "ros2-humble"])).unwrap().main_env.unwrap();
    assert_ne!(a.name, b.name);
    assert_eq!(a.name, a2.name, "order of selection must not matter");
    assert_eq!(a.ros_distro.as_deref(), Some("humble"));
}

#[test]
fn isaac_gets_its_own_env_and_python_pin() {
    let cat = Catalog::embedded();
    let res = resolve(&cat, &ids(&["ros2-jazzy", "isaac-lab"])).unwrap();
    let main = res.main_env.as_ref().unwrap();
    assert_eq!(main.components, ids(&["ros2-jazzy"]));
    assert_eq!(main.python, None, "ROS decides Python in its env");
    let own = res.own_env("isaac").unwrap();
    assert_eq!(own.components, ids(&["isaac-sim", "isaac-lab"]));
    assert_eq!(own.python.as_deref(), Some("3.11"));
}

#[test]
fn plan_is_resumable_and_expands_ros_distro() {
    let home = Home { root: tmp("plan") };
    let cat = Catalog::embedded();
    let res = resolve(&cat, &ids(&["ros2-humble", "px4-1.16", "microxrce-agent", "gazebo"])).unwrap();
    let units = plan(&home, &cat, &res, &State::load(&home));
    let all: String = units.iter().flat_map(|u| u.steps.iter()).map(|s| s.cmd.clone()).collect::<Vec<_>>().join("\n");
    assert!(matches!(units[0].key, UnitKey::Tool));
    assert!(all.contains("ros-humble-ros-gz"), "gazebo follows the ROS distro");
    assert!(!all.contains("gz-sim8"));
    assert!(all.contains("-c robostack-humble"));
    assert!(all.contains("--branch v1.16.0"));
    assert!(all.contains("Tools/setup/requirements.txt"));
    assert!(!all.contains("{sdk}") && !all.contains("{ros_distro}"));
    assert!(all.contains(&home.root.to_string_lossy().to_string()), "everything stays under home");

    // Completing units shrinks the plan; completing all empties it.
    let before = units.len();
    for u in &units[..3] {
        mark_done(&home, &u.key).unwrap();
    }
    std::fs::create_dir_all(home.root.join("tools")).unwrap();
    std::fs::write(home.micromamba(), "").unwrap();
    let rest = plan(&home, &cat, &res, &State::load(&home));
    assert_eq!(rest.len(), before - 3);
    for u in &rest {
        mark_done(&home, &u.key).unwrap();
    }
    assert!(plan(&home, &cat, &res, &State::load(&home)).is_empty());

    // A non-ROS env gets standalone Gazebo and the Python pin.
    let res = resolve(&cat, &ids(&["gazebo", "torch-cpu"])).unwrap();
    let all: String = plan(&home, &cat, &res, &State::default()).iter().flat_map(|u| u.steps.iter()).map(|s| s.cmd.clone()).collect();
    assert!(all.contains("gz-sim8") && all.contains("python=3.11"));
    std::fs::remove_dir_all(&home.root).unwrap();
}

#[test]
fn every_preset_generates_a_loadable_project() {
    let home = Home { root: tmp("gen-home") };
    let cat = Catalog::embedded();
    let reg = Registry::builtin();
    let out = tmp("gen-out");
    for p in PRESETS {
        let dir = out.join(p.id);
        let s = preset_selection(p.id, "ros2-jazzy").unwrap();
        let proj = generate(&home, &cat, &reg, &dir, &format!("Demo {}", p.id), &s).unwrap();
        let loaded = Project::load(&dir).unwrap();
        assert_eq!(loaded.run.len(), proj.run.len());
        assert!(p.modules.iter().all(|m| loaded.modules.iter().any(|x| x == m)));
        assert!(dir.join(".roboforge/activate.sh").exists());
        assert!(dir.join("README.md").exists());
        for text in walk(&dir) {
            assert!(!text.1.contains("@PKG") && !text.1.contains("@NAME@"), "placeholder left in {}", text.0);
        }
    }
    assert!(out.join("full/ros2_ws/src/demo_full/demo_full/offboard.py").exists());
    assert!(out.join("full/ros2_ws/src/demo_full/demo_full/mujoco_bridge.py").exists());
    assert!(out.join("full/ros2_ws/src/demo_full/worlds/empty.sdf").exists());
    assert!(out.join("px4/mav_ws/apps/offboard_mavsdk.py").exists());
    assert!(out.join("mujoco/sim/models/scene.xml").exists());
    assert!(out.join("isaac/.roboforge/activate-isaac.sh").exists());
    assert!(out.join("gs-slam/ros2_ws/src/demo_gs_slam/demo_gs_slam/gs_slam.py").exists());
    assert!(out.join("ros1/catkin_ws/src/demo_ros1/CMakeLists.txt").exists());
    for f in ["moveit_goal.py", "nav_goal.py", "mavsdk_bridge.py", "mission.py"] {
        assert!(out.join("mixed/ros2_ws/src/demo_mixed/demo_mixed").join(f).exists(), "{f}");
    }
    // Refuses to write into a non-empty directory.
    assert!(generate(&home, &cat, &reg, &out.join("full"), "x", &sel(None, &["mujoco"], &[])).is_err());
    if std::env::var_os("RF_KEEP").is_none() {
        std::fs::remove_dir_all(&out).unwrap();
    } else {
        println!("kept {}", out.display());
    }
    std::fs::remove_dir_all(&home.root).unwrap();
}

fn walk(dir: &std::path::Path) -> Vec<(String, String)> {
    let mut out = Vec::new();
    for e in std::fs::read_dir(dir).unwrap().flatten() {
        let p = e.path();
        if p.is_dir() {
            out.extend(walk(&p));
        } else if let Ok(t) = std::fs::read_to_string(&p) {
            out.push((p.display().to_string(), t));
        }
    }
    out
}

fn kinds(p: &Project) -> Vec<(String, String)> {
    p.workspaces.iter().map(|w| (w.kind.clone(), w.path.clone())).collect()
}

fn pair(a: &str, b: &str) -> (String, String) {
    (a.to_string(), b.to_string())
}

#[test]
fn each_kind_of_work_gets_its_own_workspace() {
    let home = Home { root: tmp("ws-home") };
    let cat = Catalog::embedded();
    let reg = Registry::builtin();
    let out = tmp("ws-out");

    // ROS 2 nodes, a standalone MAVSDK app, a MuJoCo viewer, ML and Isaac: five folders.
    let dir = out.join("mix");
    let p = generate(&home, &cat, &reg, &dir, "Mix", &sel(Some("ros2-jazzy"), &["mavsdk", "mujoco", "torch", "isaac"], &[])).unwrap();
    assert_eq!(kinds(&p), vec![pair("ros2", "ros2_ws"), pair("mav", "mav_ws"), pair("sim", "sim"), pair("ml", "ml"), pair("isaac", "isaac")]);
    assert!(dir.join("ros2_ws/src/mix/mix/mavsdk_bridge.py").exists(), "the ROS bridge is a ROS node");
    assert!(dir.join("mav_ws/apps/offboard_mavsdk.py").exists(), "the standalone app is not in the ROS workspace");
    assert!(dir.join("ros2_ws/src/mix/models/scene.xml").exists(), "with ROS 2 the model ships in the package");
    assert!(dir.join("sim/run_mujoco.py").exists() && dir.join("ml/train.py").exists() && dir.join("isaac/standalone_scene.py").exists());
    // Nothing but ROS packages under the colcon workspace.
    let top: Vec<String> = std::fs::read_dir(dir.join("ros2_ws")).unwrap().flatten().map(|e| e.file_name().to_string_lossy().to_string()).collect();
    assert_eq!(top, vec!["src".to_string()]);
    let cfg = |n: &str| p.run.iter().find(|r| r.name == n).unwrap_or_else(|| panic!("{n}")).clone();
    assert_eq!(cfg("Build workspace (colcon)").workspace.as_deref(), Some("ros2"));
    assert_eq!(cfg("MAVSDK: offboard demo").workspace.as_deref(), Some("mav"));
    assert_eq!(cfg("ML: train (PyTorch)").workspace.as_deref(), Some("ml"));
    assert_eq!(p.run_dir(&dir, &cfg("Build workspace (colcon)")), dir.join("ros2_ws"));
    assert_eq!(p.run_dir(&dir, &cfg("Isaac Sim: scene")), dir.join("isaac"));
    assert!(cfg("MuJoCo: viewer").cmd.contains("$RF_WS_ROS/src/mix/models/scene.xml"));
    let act = std::fs::read_to_string(dir.join(".roboforge/activate.sh")).unwrap();
    assert!(act.contains("export RF_WS_ROS=\"$RF_PROJECT/ros2_ws\"") && act.contains("export RF_WS_MAV=\"$RF_PROJECT/mav_ws\""));
    assert!(act.contains("for _rf in \"$RF_PROJECT/ros2_ws\"; do"));
    assert!(wrap(&dir, "true", None).contains(&dir.join(".roboforge/activate.sh").to_string_lossy().to_string()));

    // Without ROS there is no ROS workspace at all, and assets go to sim/.
    let dir = out.join("solo");
    let p = generate(&home, &cat, &reg, &dir, "Solo", &sel(None, &["mavsdk", "px4-sitl", "mujoco", "gazebo"], &[])).unwrap();
    assert_eq!(kinds(&p), vec![pair("mav", "mav_ws"), pair("sim", "sim")]);
    assert!(dir.join("sim/models/scene.xml").exists() && dir.join("sim/worlds/empty.sdf").exists());
    assert!(!dir.join("src").exists() && !dir.join("ros2_ws").exists());

    // Adding a module later adds its workspace; existing ones are untouched.
    let ch = set_modules(&home, &cat, &reg, &dir, &ids(&["mavsdk", "px4-sitl", "mujoco", "gazebo", "gsplat"])).unwrap();
    assert_eq!(kinds(&ch.project), vec![pair("mav", "mav_ws"), pair("sim", "sim"), pair("ml", "ml")]);
    assert!(ch.written.contains(&"ml/fit_splats.py".to_string()));
    std::fs::remove_dir_all(&home.root).unwrap();
    std::fs::remove_dir_all(&out).unwrap();
}

#[test]
fn an_existing_ros_workspace_can_be_reused() {
    let home = Home { root: tmp("reuse-home") };
    let cat = Catalog::embedded();
    let reg = Registry::builtin();
    let out = tmp("reuse-out");
    // Someone's existing workspace with a package and a build.
    let shared = out.join("shared_ws");
    std::fs::create_dir_all(shared.join("src/their_pkg")).unwrap();
    std::fs::write(shared.join("src/their_pkg/package.xml"), "<package/>").unwrap();
    std::fs::create_dir_all(shared.join("install")).unwrap();
    std::fs::write(shared.join("install/setup.bash"), "export THEIRS=1\n").unwrap();

    // 1. A new project that puts its package into that workspace.
    let dir = out.join("proj");
    let mut s = sel(Some("ros2-humble"), &["nav2", "mavsdk"], &[]);
    s.ros_workspace = Some(shared.to_string_lossy().to_string());
    let p = generate(&home, &cat, &reg, &dir, "Rover", &s).unwrap();
    assert_eq!(p.workspaces[0], Workspace { kind: "ros2".into(), path: shared.to_string_lossy().to_string() });
    assert!(p.workspaces[0].external());
    assert!(shared.join("src/rover/rover/nav_goal.py").exists() && shared.join("src/rover/package.xml").exists());
    assert!(shared.join("src/their_pkg/package.xml").exists(), "other packages untouched");
    assert!(dir.join("mav_ws/apps/offboard_mavsdk.py").exists(), "non-ROS workspaces stay in the project");
    assert!(!dir.join("ros2_ws").exists() && !dir.join("src").exists());
    let build = p.run.iter().find(|r| r.name.starts_with("Build")).unwrap();
    assert_eq!(p.run_dir(&dir, build), shared);
    let act = std::fs::read_to_string(dir.join(".roboforge/activate.sh")).unwrap();
    assert!(act.contains(&format!("export RF_WS_ROS=\"{}\"", shared.display())));
    // Sourcing it really overlays the reused workspace.
    let got = std::process::Command::new("bash").arg("-c").arg(format!("source {}/.roboforge/activate.sh 2>/dev/null; echo \"$THEIRS|$RF_WS_MAV\"", dir.display())).output().unwrap();
    assert_eq!(String::from_utf8_lossy(&got.stdout).trim(), format!("1|{}/mav_ws", dir.display()));
    // Modules added later also land in the reused workspace.
    let ch = set_modules(&home, &cat, &reg, &dir, &ids(&["nav2", "mavsdk", "moveit"])).unwrap();
    assert!(ch.written.iter().any(|f| f == &shared.join("src/rover/rover/moveit_goal.py").to_string_lossy().to_string()));
    // A second project cannot silently clobber the same package name.
    let err = generate(&home, &cat, &reg, &out.join("proj2"), "Rover", &s).unwrap_err();
    assert!(err.contains("already exists"), "{err}");
    // A workspace path makes no sense without ROS.
    let mut bad = sel(None, &["mujoco"], &[]);
    bad.ros_workspace = Some(shared.to_string_lossy().to_string());
    assert!(generate(&home, &cat, &reg, &out.join("proj3"), "X", &bad).is_err());

    // 2. Adopting a workspace in place: only config files are added.
    let ws2 = out.join("legacy_ws");
    std::fs::create_dir_all(ws2.join("src/stack/a_pkg")).unwrap();
    std::fs::write(ws2.join("src/stack/a_pkg/package.xml"), "<package/>").unwrap();
    assert!(adopt(&home, &cat, &reg, &out.join("empty-dir-nope"), "", "ros2-jazzy").is_err());
    let p = adopt(&home, &cat, &reg, &ws2, "", "ros2-jazzy").unwrap();
    assert_eq!(p.name, "legacy_ws");
    assert_eq!(kinds(&p), vec![pair("ros2", ".")]);
    assert_eq!(ros_packages(&ws2), vec!["a_pkg".to_string()]);
    let mut names: Vec<String> = std::fs::read_dir(&ws2).unwrap().flatten().map(|e| e.file_name().to_string_lossy().to_string()).collect();
    names.sort();
    assert_eq!(names, vec![".roboforge", "roboforge.json", "src"]);
    assert_eq!(p.run_dir(&ws2, &p.run[0]), ws2);
    assert!(adopt(&home, &cat, &reg, &ws2, "", "ros2-jazzy").is_err(), "already set up");
    // Mixing a module in afterwards adds the project's package beside theirs.
    let ch = set_modules(&home, &cat, &reg, &ws2, &ids(&["mavsdk"])).unwrap();
    assert!(ws2.join("src/legacy_ws/legacy_ws/mavsdk_bridge.py").exists() && ws2.join("src/stack/a_pkg/package.xml").exists());
    assert!(ws2.join("mav_ws/COLCON_IGNORE").exists(), "colcon must not crawl the non-ROS folder");
    assert!(ch.project.run.iter().any(|r| r.name == "Launch: bringup"));

    // 3. A second ROS workspace listed in the config is overlaid too.
    let mut p = Project::load(&dir).unwrap();
    p.workspaces.push(Workspace { kind: "ros2".into(), path: ws2.to_string_lossy().to_string() });
    p.save(&dir).unwrap();
    std::fs::create_dir_all(ws2.join("install")).unwrap();
    std::fs::write(ws2.join("install/setup.bash"), "export SECOND=2\n").unwrap();
    let comps = project_components(&cat, &reg, &p).unwrap();
    roboforge_core::project::write_activation(&home, &cat, &dir, &p, &comps).unwrap();
    let got = std::process::Command::new("bash").arg("-c").arg(format!("source {}/.roboforge/activate.sh 2>/dev/null; echo \"$THEIRS$SECOND\"", dir.display())).output().unwrap();
    assert_eq!(String::from_utf8_lossy(&got.stdout).trim(), "12");
    std::fs::remove_dir_all(&home.root).unwrap();
    std::fs::remove_dir_all(&out).unwrap();
}

#[test]
fn workspaces_and_packages_can_be_added_later() {
    let home = Home { root: tmp("wsp-home") };
    let cat = Catalog::embedded();
    let reg = Registry::builtin();
    let out = tmp("wsp-out");
    let dir = out.join("p");
    let p = generate(&home, &cat, &reg, &dir, "Base", &sel(Some("ros2-jazzy"), &[], &[])).unwrap();
    assert_eq!(p.env.get("ROS_DOMAIN_ID").map(String::as_str), Some("0"));

    // Add a second ROS workspace (new folder) and an ml workspace.
    let mut list = p.workspaces.clone();
    list.push(Workspace { kind: "ros2".into(), path: "drivers_ws".into() });
    list.push(Workspace { kind: "ml".into(), path: "ml".into() });
    let p = set_workspaces(&home, &cat, &reg, &dir, &list).unwrap();
    assert!(dir.join("drivers_ws").is_dir() && dir.join("ml").is_dir());
    let act = std::fs::read_to_string(dir.join(".roboforge/activate.sh")).unwrap();
    assert!(act.contains("\"$RF_PROJECT/ros2_ws\" \"$RF_PROJECT/drivers_ws\""), "both ROS workspaces overlaid in order");
    assert!(act.contains("export ROS_DOMAIN_ID=\"0\""));
    // ROS 1 workspace in a ROS 2 project, or moving the package's workspace, is refused.
    assert!(set_workspaces(&home, &cat, &reg, &dir, &[Workspace { kind: "ros1".into(), path: "x".into() }]).is_err());
    let err = set_workspaces(&home, &cat, &reg, &dir, &[Workspace { kind: "ros2".into(), path: "drivers_ws".into() }]).unwrap_err();
    assert!(err.contains("package lives"), "{err}");

    // New packages of each flavour, in the second workspace.
    let w = new_package(&dir, 1, "Lidar Driver", "ament_cmake").unwrap();
    assert!(w.iter().any(|f| f.ends_with("drivers_ws/src/lidar_driver/src/lidar_driver_node.cpp")));
    assert!(dir.join("drivers_ws/src/lidar_driver/CMakeLists.txt").exists());
    new_package(&dir, 1, "py tools", "ament_python").unwrap();
    assert!(dir.join("drivers_ws/src/py_tools/py_tools/py_tools_node.py").exists());
    assert!(std::fs::read_to_string(dir.join("drivers_ws/src/py_tools/setup.py")).unwrap().contains("py_tools_node = py_tools.py_tools_node:main"));
    assert_eq!(ros_packages(&dir.join("drivers_ws")), vec!["lidar_driver".to_string(), "py_tools".to_string()]);
    assert!(new_package(&dir, 1, "py tools", "ament_python").is_err(), "no clobbering");
    assert!(new_package(&dir, 2, "x", "ament_python").is_err(), "ml is not a ROS workspace");
    // Catkin flavour on a ROS 1 project.
    let d1 = out.join("p1");
    generate(&home, &cat, &reg, &d1, "Old", &sel(Some("ros1-noetic"), &[], &[])).unwrap();
    new_package(&d1, 0, "legacy", "ament_python").unwrap();
    assert!(d1.join("catkin_ws/src/legacy/scripts/legacy_node.py").exists() && d1.join("catkin_ws/src/legacy/CMakeLists.txt").exists());
    let _ = p;
    std::fs::remove_dir_all(&home.root).unwrap();
    std::fs::remove_dir_all(&out).unwrap();
}

#[test]
fn package_names_are_valid_identifiers() {
    assert_eq!(package_name("My Drone-Stack"), "my_drone_stack");
    assert_eq!(package_name("3d vision"), "pkg_3d_vision");
    assert_eq!(package_name("--"), "pkg_");
}

#[test]
fn tools_follow_the_project_contents() {
    use roboforge_core::project::{Project, Workspace};
    let d = std::env::temp_dir().join(format!("rf-tools-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&d);
    std::fs::create_dir_all(d.join("ws/src/pkg/worlds")).unwrap();
    std::fs::create_dir_all(d.join("ws/src/pkg/rviz")).unwrap();
    std::fs::create_dir_all(d.join("ws/build/pkg/worlds")).unwrap();
    std::fs::write(d.join("ws/src/pkg/worlds/arena.sdf"), "<sdf/>").unwrap();
    std::fs::write(d.join("ws/build/pkg/worlds/stale.sdf"), "<sdf/>").unwrap();
    std::fs::write(d.join("ws/src/pkg/rviz/view.rviz"), "").unwrap();
    let mut p = Project::default();
    p.ros = Some("ros2-jazzy".into());
    p.workspaces = vec![Workspace { kind: "ros2".into(), path: "ws".into() }];
    let t = roboforge_core::tools::candidates(&d, &p);
    let ids: Vec<&str> = t.iter().map(|t| t.id.as_str()).collect();
    assert_eq!(ids, ["gazebo", "rviz", "rqt_graph"]);
    assert!(t[0].cmd.contains("arena.sdf") && !t[0].cmd.contains("stale"), "{}", t[0].cmd);
    assert!(t[1].cmd.starts_with("rviz2 -d ") && t[1].cmd.contains("view.rviz"));
    // No worlds and no gazebo module: no Gazebo tool.
    std::fs::remove_dir_all(d.join("ws/src/pkg/worlds")).unwrap();
    let ids: Vec<String> = roboforge_core::tools::candidates(&d, &p).into_iter().map(|t| t.id).collect();
    assert_eq!(ids, ["rviz", "rqt_graph"]);
    let _ = std::fs::remove_dir_all(&d);
}

#[test]
fn setup_overrides_reach_the_activation_script() {
    use roboforge_core::project::{write_activation, Project, Setup};
    let d = std::env::temp_dir().join(format!("rf-setup-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&d);
    std::fs::create_dir_all(d.join("scripts")).unwrap();
    std::fs::write(d.join("fake_ros.bash"), "export FAKE_ROS=1\n").unwrap();
    std::fs::write(d.join("scripts/extra.bash"), "export EXTRA=from-bash\n").unwrap();
    std::fs::write(d.join("scripts/extra.zsh"), "this is not bash ((\n").unwrap();
    let mut p = Project::default();
    p.name = "t".into();
    p.setup = Setup {
        ros_setup: Some(d.join("fake_ros.bash").to_string_lossy().into()),
        source: vec!["scripts/extra.zsh".into(), "scripts/missing.sh".into()],
        path: vec!["tools/bin".into()],
        pythonpath: vec!["py".into()],
        python: None,
    };
    let home = roboforge_core::paths::Home { root: d.join("home") };
    let cat = roboforge_core::catalog::Catalog::embedded();
    write_activation(&home, &cat, &d, &p, &[]).unwrap();
    let out = std::process::Command::new("bash")
        .arg("-c")
        .arg(format!("source {}/.roboforge/activate.sh; echo \"$FAKE_ROS|$EXTRA|${{PATH%%:*}}|${{PYTHONPATH%%:*}}\"", d.display()))
        .output()
        .unwrap();
    let line = String::from_utf8_lossy(&out.stdout).trim().to_string();
    let err = String::from_utf8_lossy(&out.stderr).to_string();
    assert_eq!(line, format!("1|from-bash|{0}/tools/bin|{0}/py", d.display()), "stderr: {err}");
    assert!(err.contains("setup script not found") && err.contains("missing.sh"), "{err}");
    let _ = std::fs::remove_dir_all(&d);
}

#[test]
fn sdk_stacks_scaffold_and_are_recognised() {
    use roboforge_core::project::{Project, Workspace};
    use roboforge_core::sdk::{scaffold, Stack};
    let st = |sim: &str, ros: bool, mav: &str, robot: &str| Stack { sim: sim.into(), ros, mavlink: mav.into(), robot: robot.into(), model: String::new() };
    // Combinations that make no sense are refused with a reason.
    assert!(st("mujoco", false, "vehicle", "arm").validate().unwrap_err().contains("quadrotor"));
    assert!(st("none", false, "none", "").validate().unwrap_err().contains("without a simulator"));
    assert!(st("mujoco", false, "client", "quadrotor").validate().unwrap_err().contains("no simulator"));
    assert!(st("mujoco", false, "none", "custom").validate().unwrap_err().contains("URDF or MJCF"));

    let files: std::collections::BTreeMap<String, String> = scaffold("quad", &st("mujoco", true, "vehicle", "quadrotor")).unwrap().into_iter().collect();
    let yaml = &files["app.yaml"];
    assert!(yaml.contains("\nroboforge: 1\n") && yaml.contains("robot: quadrotor") && yaml.contains("mode: vehicle") && yaml.contains("- {type: ros2}"));
    assert!(yaml.contains("type: multirotor") && !yaml.contains("Patrol"), "MAVLink decides where to fly, so no patrol controller");
    assert!(files.contains_key("mission.py") && !files.contains_key("controller.py"));
    let arm = scaffold("arm", &st("mujoco", true, "none", "arm")).unwrap();
    assert!(arm.iter().any(|(f, b)| f == "controller.py" && b.contains("class Wave")));
    let bridge = scaffold("b", &st("none", true, "client", "")).unwrap();
    assert!(bridge[0].1.contains("physics: {type: none") && bridge[0].1.contains("mode: client") && bridge.len() == 1);
    assert_eq!(st("mujoco", false, "hil", "quadrotor").components(), ["roboforge-sdk", "mujoco", "pymavlink"]);

    // The scanner finds the app and its mission script and proposes running them together.
    let d = std::env::temp_dir().join(format!("rf-sdk-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&d);
    std::fs::create_dir_all(d.join("app/models")).unwrap();
    for (f, body) in &files {
        std::fs::write(d.join("app").join(f), body).unwrap();
    }
    std::fs::write(d.join("app/models/thing.xml"), "<mujoco model=\"t\"/>").unwrap();
    std::fs::write(d.join("app/other.yaml"), "name: not an app\n").unwrap();
    let mut p = Project::default();
    p.name = "quad".into();
    p.workspaces = vec![Workspace { kind: "app".into(), path: "app".into() }];
    let scan = roboforge_core::tools::scan(&d, &p, &Default::default());
    let app = scan.tools.iter().find(|t| t.id == "app:app.yaml").expect("app.yaml not detected");
    assert_eq!((app.name.as_str(), app.role.as_str(), app.cmd.as_str()), ("App: quad", "sim", "python3 -m roboforge run app.yaml"));
    assert!(app.detail.contains("MuJoCo + ROS 2 + MAVLink") && app.primary);
    assert!(scan.tools.iter().any(|t| t.id == "mjcf:models/thing.xml" && t.cmd.contains("mujoco.viewer")));
    assert!(!scan.tools.iter().any(|t| t.id.contains("other.yaml")));
    assert!(scan.kind.contains("RoboForge SDK app"));
    let cfg = scan.suggested.first().expect("no launch configuration suggested");
    let cmds: Vec<&str> = cfg.steps.iter().map(|s| s.cmd.as_str()).collect();
    assert_eq!(cmds, ["python3 -m roboforge run app.yaml", "python3 mission.py"]);
    assert!(cfg.steps[1].wait > 0 && !cfg.steps[1].after_exit, "the app keeps running, so the mission starts after a pause");
    let _ = std::fs::remove_dir_all(&d);
}
