use crate::catalog::Catalog;
use crate::paths::Home;
use crate::plan::expand;
use crate::resolve::{resolve, EnvSpec};
use crate::sh_quote;
use serde::{Deserialize, Serialize};
use std::path::{Path, PathBuf};

pub const MANIFEST: &str = "roboforge.json";

/// A folder with one job inside a project, like a root in a VS Code
/// multi-root workspace. `kind` says how it is built and what goes in it:
///
/// - `ros2` / `ros1`: a colcon / catkin workspace (`src/`, `build/`, ...)
/// - `mav`: standalone MAVSDK / PX4 applications, no ROS
/// - `sim`: simulator assets and scripts outside ROS
/// - `ml`: training and research code
/// - `isaac`: Isaac Sim apps (run in the dedicated Isaac environment)
///
/// `path` is relative to the project, or absolute / `~/...` to reuse a
/// workspace that lives elsewhere (for example an existing `~/ros2_ws`).
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Workspace {
    pub kind: String,
    pub path: String,
}

pub fn default_ws_path(kind: &str) -> String {
    match kind {
        "ros2" => "ros2_ws".into(),
        "ros1" => "catkin_ws".into(),
        "mav" => "mav_ws".into(),
        other => other.to_string(),
    }
}

pub fn expand_tilde(p: &str) -> PathBuf {
    let home = std::env::var("HOME").unwrap_or_default();
    if p == "~" {
        PathBuf::from(home)
    } else if let Some(rest) = p.strip_prefix("~/") {
        Path::new(&home).join(rest)
    } else {
        PathBuf::from(p)
    }
}

impl Workspace {
    pub fn is_ros(&self) -> bool {
        self.kind == "ros2" || self.kind == "ros1"
    }
    /// True when the workspace lives outside the project folder.
    pub fn external(&self) -> bool {
        expand_tilde(&self.path).is_absolute()
    }
    pub fn abs(&self, project_dir: &Path) -> PathBuf {
        let p = expand_tilde(&self.path);
        if p.is_absolute() {
            p
        } else if self.path.is_empty() || self.path == "." {
            project_dir.to_path_buf()
        } else {
            project_dir.join(p)
        }
    }
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct RunConfig {
    pub name: String,
    pub cmd: String,
    /// Workspace kind whose folder the command runs in; project root if absent.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub workspace: Option<String>,
    /// `own_env` key to run in (for example `isaac`); main env when absent.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub env: Option<String>,
}

/// `roboforge.json`: the project's ROS distro, its modules, its workspaces,
/// any extra SDK components, and its run configurations.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Project {
    pub name: String,
    /// ROS distro component id (for example `ros2-jazzy`), if any.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub ros: Option<String>,
    /// Modules mixed into this project (see `roboforge modules`).
    #[serde(default)]
    pub modules: Vec<String>,
    /// Workspace folders. The first ROS workspace hosts the project's own
    /// package; further ROS workspaces are sourced too, in the order listed.
    #[serde(default)]
    pub workspaces: Vec<Workspace>,
    /// SDK components beyond what the modules already bring.
    #[serde(default)]
    pub components: Vec<String>,
    /// Extra ROS package dependencies to keep in the generated package.xml.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub depends: Vec<String>,
    /// Environment variables exported on activation, after the SDK and ROS
    /// overlays, so they win: ROS_DOMAIN_ID, RMW_IMPLEMENTATION,
    /// ROS_LOCALHOST_ONLY, GZ_SIM_RESOURCE_PATH and so on.
    #[serde(default, skip_serializing_if = "std::collections::BTreeMap::is_empty")]
    pub env: std::collections::BTreeMap<String, String>,
    #[serde(default)]
    pub run: Vec<RunConfig>,
    /// User overrides for how the environment is set up: scripts to source,
    /// a different ROS setup file, PATH / PYTHONPATH additions, the Python
    /// used for builds.
    #[serde(default, skip_serializing_if = "Setup::is_empty")]
    pub setup: Setup,
    /// Launch configurations: named, ordered lists of commands (a world,
    /// then SLAM, then a teleop terminal...). `None` until the IDE has
    /// offered an initial one, so a list the user emptied stays empty.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub launch: Option<Vec<LaunchConfig>>,
}

/// Environment customisation, applied on every activation (jobs, launch
/// steps and terminals). Paths may be absolute, start with `~/`, or be
/// relative to the project folder.
#[derive(Debug, Clone, Default, Serialize, Deserialize, PartialEq)]
pub struct Setup {
    /// ROS setup file to source instead of the project SDK environment,
    /// for example `/opt/ros/jazzy/setup.bash` or a custom underlay.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub ros_setup: Option<String>,
    /// Extra scripts sourced after ROS and the workspace overlays, in order.
    /// A `.zsh` file with a `.bash` sibling uses the sibling (activation
    /// runs in bash).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub source: Vec<String>,
    /// Folders put in front of PATH.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub path: Vec<String>,
    /// Folders put in front of PYTHONPATH.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub pythonpath: Vec<String>,
    /// Python interpreter builds should use.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub python: Option<String>,
}

impl Setup {
    pub fn is_empty(&self) -> bool {
        *self == Setup::default()
    }
}

#[derive(Debug, Clone, Default, Serialize, Deserialize, PartialEq)]
pub struct LaunchConfig {
    pub name: String,
    #[serde(default)]
    pub steps: Vec<LaunchStep>,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize, PartialEq)]
pub struct LaunchStep {
    pub name: String,
    pub cmd: String,
    /// Workspace kind whose folder the command runs in; project root if absent.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub workspace: Option<String>,
    /// Run in an integrated terminal (keyboard input) instead of as a job.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub terminal: bool,
    /// Seconds to wait before starting this step (let a simulator come up).
    #[serde(default, skip_serializing_if = "is_zero")]
    pub wait: u32,    /// Start only once the previous step has finished successfully (for a
    /// "start the simulator" command that returns when it is up).
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub after_exit: bool,
}

fn is_zero(n: &u32) -> bool {
    *n == 0
}

impl Project {
    pub fn load(dir: &Path) -> Result<Project, String> {
        let f = dir.join(MANIFEST);
        let text = std::fs::read_to_string(&f).map_err(|e| format!("{}: {e}", f.display()))?;
        serde_json::from_str(&text).map_err(|e| format!("{}: {e}", f.display()))
    }
    pub fn save(&self, dir: &Path) -> Result<(), String> {
        let f = dir.join(MANIFEST);
        std::fs::write(&f, serde_json::to_string_pretty(self).unwrap() + "\n")
            .map_err(|e| format!("{}: {e}", f.display()))
    }
    /// First workspace of a kind. `ros` matches either ROS version.
    pub fn workspace(&self, kind: &str) -> Option<&Workspace> {
        find_ws(&self.workspaces, kind)
    }
    /// Folder a run configuration executes in.
    pub fn run_dir(&self, dir: &Path, rc: &RunConfig) -> PathBuf {
        rc.workspace
            .as_deref()
            .and_then(|k| self.workspace(k))
            .map(|w| w.abs(dir))
            .unwrap_or_else(|| dir.to_path_buf())
    }
}

pub fn find_ws<'a>(list: &'a [Workspace], kind: &str) -> Option<&'a Workspace> {
    list.iter().find(|w| w.kind == kind || (kind == "ros" && w.is_ros()))
}

pub fn activate_file(env: Option<&str>) -> String {
    match env {
        Some(k) => format!(".roboforge/activate-{k}.sh"),
        None => ".roboforge/activate.sh".to_string(),
    }
}

/// A path as a double-quoted bash word; project-relative paths stay
/// relative to `$RF_PROJECT` so the project folder can be moved.
fn ws_shell(ws: &Workspace) -> String {
    let esc = |s: &str| s.replace('\\', "\\\\").replace('"', "\\\"").replace('$', "\\$").replace('`', "\\`");
    let p = expand_tilde(&ws.path);
    if p.is_absolute() {
        format!("\"{}\"", esc(&p.to_string_lossy()))
    } else if ws.path.is_empty() || ws.path == "." {
        "\"$RF_PROJECT\"".into()
    } else {
        format!("\"$RF_PROJECT/{}\"", esc(&ws.path))
    }
}

/// A user-given path as a double-quoted bash word (`~/x`, absolute, or
/// relative to the project).
fn path_shell(p: &str) -> String {
    ws_shell(&Workspace { kind: String::new(), path: p.trim().to_string() })
}

fn script(home: &Home, cat: &Catalog, proj: &Project, env: Option<&EnvSpec>, all: &[String]) -> String {
    let mut s = String::from("# Generated by RoboForge from roboforge.json. Do not edit; use Sync SDKs.\n");
    s += "# Usage: source this file from bash.\n";
    s += &format!("export ROBOFORGE_HOME={}\n", sh_quote(&home.root.to_string_lossy()));
    s += &format!("export MAMBA_ROOT_PREFIX={}\n", sh_quote(&home.mamba_root().to_string_lossy()));
    // Jobs start from a bare `bash -c`, and the IDE may itself have been
    // launched from a menu with a minimal PATH: pick up the user's Rust
    // toolchain and ~/.local/bin so cargo and friends are found either way.
    s += "[ -f \"$HOME/.cargo/env\" ] && . \"$HOME/.cargo/env\"\n";
    s += "case \":$PATH:\" in *\":$HOME/.local/bin:\"*) ;; *) [ -d \"$HOME/.local/bin\" ] && export PATH=\"$HOME/.local/bin:$PATH\" ;; esac\n";
    s += "export RF_PROJECT=\"$(cd \"$(dirname \"${BASH_SOURCE[0]}\")/..\" && pwd)\"\n";
    s += &format!("export RF_PROJECT_NAME={}\n", sh_quote(&proj.name));
    // Source a user script. Activation runs in bash, so a .zsh file with a
    // .bash sibling (ROS ships both) uses the sibling.
    s += "_rf_src() {\n  local f=\"$1\"\n  case \"$f\" in *.zsh) [ -f \"${f%.zsh}.bash\" ] && f=\"${f%.zsh}.bash\" ;; esac\n  if [ -f \"$f\" ]; then . \"$f\"; else echo \"roboforge: setup script not found: $1\" >&2; fi\n}\n";
    let main = env.map_or(true, |e| !e.name.starts_with("own-"));
    let ros_override = proj.setup.ros_setup.as_deref().map(str::trim).filter(|p| !p.is_empty() && main);
    if let Some(p) = ros_override {
        s += &format!("_rf_src {}\n", path_shell(p));
    } else if let Some(e) = env {
        let mm = sh_quote(&home.micromamba().to_string_lossy());
        let prefix = sh_quote(&home.env_prefix(&e.name).to_string_lossy());
        // Fall back to a system-wide ROS install (/opt/ros/<distro>) when the
        // RoboStack environment has not been synced.
        let sys = match &e.ros_distro {
            Some(d) if d.chars().all(|c| c.is_ascii_alphanumeric() || c == '_') => format!(
                "elif [ -f /opt/ros/{d}/setup.bash ]; then\n  echo 'roboforge: SDK environment {} not synced; using system ROS /opt/ros/{d}.' >&2\n  source /opt/ros/{d}/setup.bash\n",
                e.name
            ),
            _ => String::new(),
        };
        s += &format!(
            "if [ -x {mm} ] && [ -d {prefix} ]; then\n  eval \"$({mm} shell hook -s bash)\"\n  micromamba activate {prefix}\n{sys}else\n  echo 'roboforge: SDK environment {} is not installed yet. Run \"Sync SDKs\" (or: roboforge sync).' >&2\nfi\n",
            e.name
        );
    }
    // Exports come after activation so SDK paths win over the env's own.
    for id in all {
        if let Some(c) = cat.get(id) {
            for (k, v) in &c.env {
                s += &format!("export {k}=\"{}\"\n", expand(v, home, c, env).replace('"', "\\\""));
            }
        }
    }
    // One variable per workspace kind: RF_WS_ROS, RF_WS_MAV, RF_WS_SIM, ...
    let mut seen = std::collections::BTreeSet::new();
    for w in &proj.workspaces {
        let mut names = vec![w.kind.clone()];
        if w.is_ros() {
            names.push("ros".into());
        }
        for n in names {
            let var: String = n.chars().map(|c| if c.is_ascii_alphanumeric() { c.to_ascii_uppercase() } else { '_' }).collect();
            if seen.insert(var.clone()) {
                s += &format!("export RF_WS_{var}={}\n", ws_shell(w));
            }
        }
    }
    if env.map_or(true, |e| !e.name.starts_with("own-")) {
        // Overlay every ROS workspace, in the order listed in roboforge.json.
        let ros: Vec<String> = proj.workspaces.iter().filter(|w| w.is_ros()).map(ws_shell).collect();
        let roots = if ros.is_empty() { "\"$RF_PROJECT\"".to_string() } else { ros.join(" ") };
        s += &format!("for _rf in {roots}; do\n  if [ -f \"$_rf/install/setup.bash\" ]; then source \"$_rf/install/setup.bash\"\n  elif [ -f \"$_rf/devel/setup.bash\" ]; then source \"$_rf/devel/setup.bash\"; fi\ndone\nunset _rf\n");
        // Simulators find the package's models and worlds without a build.
        let sim_dirs: Vec<String> = proj.workspaces.iter().filter(|w| w.kind == "sim").map(|w| format!("{}/models", ws_shell(w).trim_matches('"'))).collect();
        if !sim_dirs.is_empty() {
            s += &format!("export GZ_SIM_RESOURCE_PATH=\"{}${{GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}}\"\n", sim_dirs.join(":"));
        }
    }
    if main {
        for p in proj.setup.source.iter().map(|p| p.trim()).filter(|p| !p.is_empty()) {
            s += &format!("_rf_src {}\n", path_shell(p));
        }
        for p in proj.setup.path.iter().rev().map(|p| p.trim()).filter(|p| !p.is_empty()) {
            s += &format!("export PATH={}\":$PATH\"\n", path_shell(p));
        }
        for p in proj.setup.pythonpath.iter().rev().map(|p| p.trim()).filter(|p| !p.is_empty()) {
            s += &format!("export PYTHONPATH={}\"${{PYTHONPATH:+:$PYTHONPATH}}\"\n", path_shell(p));
        }
        if let Some(py) = proj.setup.python.as_deref().map(str::trim).filter(|p| !p.is_empty()) {
            // colcon, CMake's FindPython3 and plain `python3` all follow this interpreter.
            let q = path_shell(py);
            s += &format!("if [ -x {q} ]; then\n  export RF_PYTHON={q}\n  export COLCON_PYTHON_EXECUTABLE=\"$RF_PYTHON\"\n  export Python3_ROOT_DIR=\"$(dirname \"$(dirname \"$RF_PYTHON\")\")\"\n  export PATH=\"$(dirname \"$RF_PYTHON\"):$PATH\"\nelse\n  echo \"roboforge: Python interpreter not found: \"{q} >&2\nfi\n");
        }
    }
    s += "unset -f _rf_src\n";
    for (k, v) in &proj.env {
        if k.chars().all(|c| c.is_ascii_alphanumeric() || c == '_') && !k.is_empty() {
            s += &format!("export {k}=\"{}\"\n", v.replace('\\', "\\\\").replace('"', "\\\""));
        }
    }
    // Keep builds from pinning every core: leave two free by default. Set
    // RF_JOBS in the project's env to change it; variables the user already
    // exported are left alone. The -l load limit also holds back make when
    // colcon builds several packages at once.
    s += "if [ -z \"${RF_JOBS:-}\" ]; then RF_JOBS=$(( $(nproc 2>/dev/null || echo 4) - 2 )); [ \"$RF_JOBS\" -lt 1 ] && RF_JOBS=1; fi\nexport RF_JOBS\n";
    s += "export MAKEFLAGS=\"${MAKEFLAGS:--j$RF_JOBS -l$RF_JOBS}\"\n";
    s += "export CMAKE_BUILD_PARALLEL_LEVEL=\"${CMAKE_BUILD_PARALLEL_LEVEL:-$RF_JOBS}\"\n";
    s += "export CARGO_BUILD_JOBS=\"${CARGO_BUILD_JOBS:-$RF_JOBS}\"\n";
    s
}

/// (Re)write `.roboforge/activate*.sh` for the project's environments.
/// `components` is the project's full component list (see
/// `templates::project_components`).
pub fn write_activation(home: &Home, cat: &Catalog, dir: &Path, proj: &Project, components: &[String]) -> Result<(), String> {
    let res = resolve(cat, components)?;
    let d = dir.join(".roboforge");
    std::fs::create_dir_all(&d).map_err(|e| e.to_string())?;
    let w = |name: String, body: String| std::fs::write(dir.join(name), body).map_err(|e| e.to_string());
    w(activate_file(None), script(home, cat, proj, res.main_env.as_ref(), &res.components))?;
    for e in &res.own_envs {
        let key = e.name.trim_start_matches("own-");
        w(activate_file(Some(key)), script(home, cat, proj, Some(e), &res.components))?;
    }
    Ok(())
}

/// Wrap a command so it runs inside the right SDK environment. The caller
/// picks the working directory (see `Project::run_dir`).
pub fn wrap(project_dir: &Path, cmd: &str, env: Option<&str>) -> String {
    let act = project_dir.join(activate_file(env));
    format!("source {} && {{ {cmd}\n}}", sh_quote(&act.to_string_lossy()))
}
