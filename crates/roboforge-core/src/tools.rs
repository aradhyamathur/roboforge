//! What a project can launch, worked out from its directory tree: launch
//! files in the workspace packages, companion packages of the same family
//! installed in the environment (for example `turtlebot3_cartographer` next
//! to a `turtlebot3_gazebo` checkout), teleop executables, RViz configs and
//! worlds. Each entry gets a role (simulation, SLAM, navigation, control,
//! visualization) so the IDE can start a whole stack in order.
//!
//! Filesystem inspection only. The caller passes the project environment
//! (`env` after activation) so executables and variables can be resolved.

use crate::project::{LaunchConfig, LaunchStep, Project, Setup};
use crate::sh_quote;
use serde::Serialize;
use std::collections::{BTreeMap, BTreeSet, HashMap};
use std::path::{Path, PathBuf};

#[derive(Debug, Clone, Serialize, PartialEq, Default)]
pub struct Tool {
    pub id: String,
    pub name: String,
    pub cmd: String,
    /// Workspace kind the command runs in (project root when absent).
    pub workspace: Option<String>,
    /// Executables that must resolve inside the project environment.
    pub needs: Vec<String>,
    pub missing: Vec<String>,
    pub available: bool,
    /// sim, slam, nav, control, viz, manip, bringup, other.
    pub role: String,
    /// Launch order for "launch stack": 0 simulators, 1 everything that
    /// attaches to them. Stage 2 entries are only started by hand.
    pub stage: u8,
    /// Needs a keyboard: runs in an integrated terminal, not as a job.
    pub terminal: bool,
    /// Included by another launch file of the same package.
    pub helper: bool,
    /// The default pick for its role.
    pub primary: bool,
    /// Shown when the tool cannot run.
    pub hint: String,
    /// What the command was derived from, shown in the UI.
    pub detail: String,
    /// Environment variables it reads without a default (directly or through included launch files).
    pub env: Vec<String>,
    /// Starts RViz itself, so a separate viewer is not needed.
    pub opens_rviz: bool,
    /// Placeholders the command still needs filled in (`CHECKPOINT`).
    pub args: Vec<String>,
    /// A "start" command that returns once the thing is up (it has a
    /// matching stop command), so the next step can simply follow it.
    pub returns: bool,
}

/// An environment variable the launch files read without a default.
#[derive(Debug, Clone, Serialize, PartialEq)]
pub struct EnvNeed {
    pub name: String,
    pub value: Option<String>,
    pub choices: Vec<String>,
}

#[derive(Debug, Clone, Serialize, Default)]
pub struct Scan {
    pub tools: Vec<Tool>,
    pub env: Vec<EnvNeed>,
    /// One line for the "project loaded" notice; empty when nothing special was found.
    pub summary: String,
    /// First launch configurations, built from the default pick of each role.
    pub suggested: Vec<LaunchConfig>,
    /// What kind of project this looks like, for the import summary.
    pub kind: String,
    /// Things worth telling the user (a missing interpreter, ...).
    pub notes: Vec<String>,
    /// Environment settings the project itself points at (its virtualenv).
    pub suggested_setup: Option<Setup>,
}

/// Initial launch configurations.
///
/// Robot stack: world first, then SLAM, a viewer unless SLAM brings its own,
/// and keyboard control in a terminal. Script projects: the simulator start
/// command followed by the main run command, and by training if there is one.
fn suggest(tools: &[Tool]) -> Vec<LaunchConfig> {
    let pick = |role: &str| tools.iter().find(|t| t.role == role && t.primary && !t.helper && t.args.is_empty());
    let step = |t: &Tool, wait: u32, after_exit: bool| LaunchStep { name: t.name.clone(), cmd: t.cmd.clone(), workspace: t.workspace.clone(), terminal: t.terminal, wait, after_exit };
    let mut out = Vec::new();
    let Some(sim) = pick("sim") else { return out };
    // What follows the simulator: right after it returns, or after a pause.
    let follow = |t: &Tool| if sim.returns { step(t, 0, true) } else { step(t, 7, false) };

    let mut steps = vec![step(sim, 0, false)];
    let mut labels = vec!["World"];
    let slam = pick("slam");
    if let Some(t) = slam {
        steps.push(follow(t));
        labels.push("SLAM");
    }
    if let Some(t) = pick("viz").filter(|_| !slam.map_or(false, |s| s.opens_rviz)) {
        steps.push(if steps.len() == 1 { follow(t) } else { step(t, 0, false) });
        labels.push("RViz");
    }
    if let Some(t) = pick("control").filter(|t| t.terminal) {
        steps.push(if steps.len() == 1 { follow(t) } else { step(t, 0, false) });
        labels.push("teleop");
    }
    if steps.len() >= 2 && (slam.is_some() || labels.contains(&"teleop") || labels.contains(&"RViz")) {
        out.push(LaunchConfig { name: labels.join(" + "), steps });
    }
    let short = |t: &Tool| t.name.split(": ").nth(1).unwrap_or(&t.name).to_string();
    for role in ["run", "train"] {
        if let Some(t) = pick(role) {
            out.push(LaunchConfig { name: format!("{} → {}", short(sim), short(t)), steps: vec![step(sim, 0, false), follow(t)] });
        }
    }
    out
}

const SKIP: &[&str] = &["build", "install", "devel", "log", ".git", "node_modules", "target", "__pycache__", ".roboforge", "venv", ".venv"];
/// First name tokens that are too generic to treat as a package family.
const COMMON: &[&str] = &["ros", "ros2", "nav2", "rclcpp", "rclpy", "moveit", "gz", "rviz", "rqt", "tf2", "std", "sensor", "geometry", "nav", "launch", "ament", "robot", "my", "test", "demo", "example", "examples", "common", "control", "controller", "gazebo", "joint", "image", "camera", "laser", "slam", "navigation", "the"];

fn walk(root: &Path, depth: usize, out: &mut Vec<PathBuf>) {
    let Ok(rd) = std::fs::read_dir(root) else { return };
    let mut entries: Vec<_> = rd.flatten().collect();
    entries.sort_by_key(|e| e.file_name());
    for e in entries {
        let p = e.path();
        let name = e.file_name().to_string_lossy().to_string();
        if p.is_dir() {
            if depth > 0 && !SKIP.contains(&name.as_str()) && !name.starts_with('.') {
                walk(&p, depth - 1, out);
            }
        } else if p.is_file() {
            out.push(p);
        }
    }
}

fn has_ext(p: &Path, ext: &str) -> bool {
    p.extension().map_or(false, |e| e.eq_ignore_ascii_case(ext))
}

fn in_dir_named(p: &Path, dir: &str) -> bool {
    p.parent().and_then(|d| d.file_name()).map_or(false, |n| n == dir)
}

fn file_name(p: &Path) -> String {
    p.file_name().map(|n| n.to_string_lossy().to_string()).unwrap_or_default()
}

fn read_small(p: &Path) -> String {
    match std::fs::metadata(p) {
        Ok(m) if m.len() <= 512 * 1024 => std::fs::read_to_string(p).unwrap_or_default(),
        _ => String::new(),
    }
}

/// `<name>` of a package.xml, falling back to the folder name.
fn package_name(dir: &Path) -> String {
    let xml = read_small(&dir.join("package.xml"));
    xml.split("<name>").nth(1).and_then(|r| r.split("</name>").next()).map(|n| n.trim().to_string()).filter(|n| !n.is_empty()).unwrap_or_else(|| file_name(dir))
}

/// Source packages under a workspace: (name, folder).
fn source_packages(root: &Path, depth: usize, out: &mut Vec<(String, PathBuf)>) {
    if root.join("package.xml").is_file() {
        out.push((package_name(root), root.to_path_buf()));
        return;
    }
    if depth == 0 {
        return;
    }
    let Ok(rd) = std::fs::read_dir(root) else { return };
    let mut dirs: Vec<PathBuf> = rd.flatten().map(|e| e.path()).filter(|p| p.is_dir()).collect();
    dirs.sort();
    for d in dirs {
        let n = file_name(&d);
        if !SKIP.contains(&n.as_str()) && !n.starts_with('.') {
            source_packages(&d, depth - 1, out);
        }
    }
}

fn is_launch_file(p: &Path) -> bool {
    let n = file_name(p).to_ascii_lowercase();
    if n.starts_with('_') || n.starts_with('.') {
        return false;
    }
    n.ends_with(".launch.py") || n.ends_with("_launch.py") || n.ends_with(".launch.xml") || n.ends_with(".launch.yaml") || n.ends_with(".launch")
        || ((n.ends_with(".py") || n.ends_with(".xml")) && in_dir_named(p, "launch"))
}

fn stem(file: &str) -> String {
    let mut s = file.to_string();
    for suf in [".launch.py", "_launch.py", ".launch.xml", ".launch.yaml", ".launch", ".py", ".xml"] {
        if let Some(x) = s.strip_suffix(suf) {
            s = x.to_string();
            break;
        }
    }
    s
}

struct Entry {
    pkg: String,
    file: String,
    content: String,
    /// Workspace kind when the package is a checkout in the project.
    ws: Option<String>,
    built: bool,
}

fn launch_entries(pkg: &str, launch_dir: &Path, ws: Option<&str>, built: bool, out: &mut Vec<Entry>) {
    let mut files = Vec::new();
    walk(launch_dir, 2, &mut files);
    for f in files.into_iter().filter(|f| is_launch_file(f)) {
        out.push(Entry { pkg: pkg.to_string(), file: file_name(&f), content: read_small(&f), ws: ws.map(String::from), built });
    }
}

fn contains_any(hay: &str, needles: &[&str]) -> bool {
    needles.iter().any(|n| hay.contains(n))
}

fn role_of(e: &Entry) -> &'static str {
    let f = e.file.to_ascii_lowercase();
    let both = format!("{} {}", e.pkg.to_ascii_lowercase(), f);
    if contains_any(&f, &["rviz", "rqt", "foxglove"]) {
        "viz"
    } else if contains_any(&both, &["cartographer", "slam", "gmapping", "rtabmap", "mapping", "lio_", "orb_", "hector"]) {
        "slam"
    } else if contains_any(&both, &["navigation", "nav2", "amcl", "localization"]) {
        "nav"
    } else if contains_any(&both, &["teleop", "joy", "keyboard"]) {
        "control"
    } else if contains_any(&both, &["moveit", "move_group"]) {
        "manip"
    } else if contains_any(&e.content, &["gz_sim", "ros_gz_sim", "gazebo_ros", "gzserver", "gz sim", "ign gazebo", "webots_ros2", "isaacsim", "mujoco"]) || contains_any(&f, &["world", "gazebo", "sim", "fake"]) {
        "sim"
    } else if contains_any(&both, &["bringup", "robot"]) {
        "bringup"
    } else {
        "other"
    }
}

fn role_label(role: &str) -> &'static str {
    match role {
        "sim" => "World",
        "slam" => "SLAM",
        "nav" => "Navigation",
        "control" => "Control",
        "viz" => "View",
        "manip" => "Manipulation",
        "bringup" => "Bringup",
        "run" => "Run",
        "train" => "Train",
        "test" => "Test",
        "docs" => "Docs",
        "stop" => "Stop",
        "tool" => "Tool",
        _ => "Launch",
    }
}

/// `os.environ['NAME']` reads: variables a launch file cannot run without.
fn required_env(content: &str, out: &mut BTreeSet<String>) {
    for part in content.split("os.environ[").skip(1) {
        let mut ch = part.chars();
        let Some(q) = ch.next().filter(|c| *c == '\'' || *c == '"') else { continue };
        let name: String = ch.take_while(|c| *c != q).collect();
        if !name.is_empty() && name.chars().all(|c| c.is_ascii_alphanumeric() || c == '_') {
            out.insert(name);
        }
    }
}

fn on_path(env: &HashMap<String, String>, bin: &str) -> bool {
    env.get("PATH").map_or(false, |p| p.split(':').any(|d| !d.is_empty() && Path::new(d).join(bin).is_file()))
}

/// Role of a script subcommand, Makefile target or script name.
fn role_of_word(word: &str) -> &'static str {
    let w = word.to_ascii_lowercase();
    let is = |list: &[&str]| list.iter().any(|x| w == *x || w.starts_with(&format!("{x}_")) || w.starts_with(&format!("{x}-")));
    if is(&["sim", "simulator", "simulation", "start", "up", "world", "gazebo", "sitl"]) {
        "sim"
    } else if is(&["stop", "down", "kill", "shutdown"]) {
        "stop"
    } else if is(&["train", "gate", "resume", "fit", "finetune", "learn"]) {
        "train"
    } else if is(&["test", "tests", "check", "lint", "pytest"]) {
        "test"
    } else if is(&["docs", "doc", "html"]) {
        "docs"
    } else if is(&["show", "status", "info", "help", "notebook", "install", "setup", "build", "clean", "all", "format", "fmt", "strict"]) {
        "tool"
    } else {
        "run"
    }
}

fn script_label(role: &str) -> &'static str {
    if role == "sim" { "Simulator" } else { role_label(role) }
}

fn is_executable(p: &Path) -> bool {
    use std::os::unix::fs::PermissionsExt;
    std::fs::metadata(p).map_or(false, |m| m.permissions().mode() & 0o111 != 0)
}

/// Subcommands of a dispatcher script (`./run.sh sim`, `./run.sh train ...`):
/// (name, placeholder arguments, description), in the order the script documents them.
fn script_subcommands(file: &str, text: &str) -> Vec<(String, Vec<String>, String)> {
    let mut out: Vec<(String, Vec<String>, String)> = Vec::new();
    let ok = |n: &str| !n.is_empty() && n.len() <= 24 && n.chars().all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '_' || c == '-');
    // Usage lines in comments:  #   ./run.sh fly CHECKPOINT    fly a trained policy
    for line in text.lines() {
        let Some(rest) = line.trim_start().strip_prefix('#') else { continue };
        let rest = rest.trim_start();
        let Some(rest) = rest.strip_prefix(&format!("./{file} ")).or_else(|| rest.strip_prefix(&format!("{file} "))) else { continue };
        let (head, desc) = match rest.find("  ") {
            Some(i) => (&rest[..i], rest[i..].trim()),
            None => (rest.trim(), ""),
        };
        let mut words = head.split_whitespace();
        let Some(name) = words.next().filter(|n| ok(n)) else { continue };
        let args: Vec<String> = words.filter(|w| w.chars().all(|c| c.is_ascii_uppercase() || c == '_' || c.is_ascii_digit()) && w.chars().any(|c| c.is_ascii_uppercase())).map(String::from).collect();
        match out.iter_mut().find(|(n, _, _)| n == name) {
            Some(e) => {
                if e.2.is_empty() { e.2 = desc.to_string(); }
                if e.1.is_empty() { e.1 = args; }
            }
            None if !desc.is_empty() || !args.is_empty() => out.push((name.to_string(), args, desc.to_string())),
            None => {}
        }
    }
    // Handlers:  cmd_sim() { ... }   and   case "$1" in  sim) ...
    let mut in_case = false;
    for line in text.lines() {
        let t = line.trim();
        let mut found = None;
        if let Some(r) = t.strip_prefix("cmd_").or_else(|| t.strip_prefix("do_")) {
            if let Some(i) = r.find("()") {
                found = Some(r[..i].trim().to_string());
            }
        }
        if t.starts_with("case ") && (t.contains("$1") || t.contains("${1") || t.contains("$cmd") || t.contains("$command") || t.contains("$sub")) {
            in_case = true;
        } else if t == "esac" {
            in_case = false;
        } else if in_case && found.is_none() {
            if let Some(i) = t.find(')') {
                let arm = &t[..i];
                if !arm.contains('*') && !arm.contains(' ') && !arm.contains('"') {
                    found = arm.split('|').next().map(String::from);
                }
            }
        }
        if let Some(n) = found.filter(|n| ok(n)) {
            if !out.iter().any(|(x, _, _)| *x == n) {
                out.push((n, vec![], String::new()));
            }
        }
    }
    out
}

/// First line of a Python module docstring that says something.
fn py_summary(text: &str, file: &str) -> String {
    let Some(i) = text.find("\"\"\"") else { return String::new() };
    if i > 400 {
        return String::new();
    }
    let body = &text[i + 3..];
    let body = &body[..body.find("\"\"\"").unwrap_or(body.len())];
    body.lines().map(str::trim).find(|l| !l.is_empty() && *l != file && !l.starts_with(file)).map(|l| l.chars().take(90).collect()).unwrap_or_default()
}

/// Launchable things in a plain project folder: dispatcher scripts, Python
/// entry points, a Makefile, tests, docs and Gazebo worlds. Looks at the
/// folder itself only (not its subfolders), so it is cheap and predictable.
fn folder_tools(root: &Path, ws: Option<&str>, tools: &mut Vec<Tool>, notes: &mut Vec<String>, python: &mut Option<String>, kinds: &mut Vec<String>) {
    let Ok(rd) = std::fs::read_dir(root) else { return };
    let mut names: Vec<String> = rd.flatten().filter(|e| e.path().is_file()).map(|e| e.file_name().to_string_lossy().to_string()).collect();
    names.sort();
    let wsk = ws.map(String::from);
    let mk = |id: String, name: String, cmd: String, role: &str, detail: String| Tool {
        id, name, cmd, workspace: wsk.clone(), role: role.into(), stage: if role == "sim" { 0 } else { 2 }, available: true, detail,
        hint: "Not available in this environment.".into(), ..Default::default()
    };

    // --- dispatcher shell scripts ---
    let mut dispatcher = false;
    for f in names.iter().filter(|n| n.ends_with(".sh")).take(12) {
        let path = root.join(f);
        let text = read_small(&path);
        let run = if is_executable(&path) { format!("./{f}") } else { format!("bash {f}") };
        // PYTHON="${PYTHON:-$HOME/venvs/x/bin/python}": the interpreter the project expects.
        for line in text.lines() {
            let t = line.trim();
            if let Some(v) = t.strip_prefix("PYTHON=").or_else(|| t.strip_prefix("export PYTHON=")) {
                let v = v.trim_matches('"').trim_matches('\'');
                let v = v.strip_prefix("${PYTHON:-").and_then(|x| x.strip_suffix('}')).unwrap_or(v);
                if v.contains('/') && !v.contains("$(") {
                    let home = std::env::var("HOME").unwrap_or_default();
                    let abs = v.replace("${HOME}", &home).replace("$HOME", &home);
                    let abs = if let Some(r) = abs.strip_prefix("~/") { format!("{home}/{r}") } else { abs };
                    if Path::new(&abs).is_file() {
                        python.get_or_insert(abs);
                    } else {
                        notes.push(format!("{f} expects the Python interpreter {v}, which does not exist on this machine. Create that environment, or set PYTHON under Environment ▸ Variables."));
                    }
                }
            }
        }
        let subs = script_subcommands(f, &text);
        if subs.is_empty() {
            let base = f.trim_end_matches(".sh");
            let role = role_of_word(base);
            tools.push(mk(f.clone(), format!("{}: {f}", script_label(role)), run, role, "shell script".into()));
            continue;
        }
        dispatcher = true;
        let has_stop = subs.iter().any(|(n, _, _)| role_of_word(n) == "stop");
        for (name, args, desc) in subs {
            let role = role_of_word(&name);
            let mut t = mk(format!("{f}:{name}"), format!("{}: {f} {name}", script_label(role)), [vec![run.clone(), name.clone()], args.clone()].concat().join(" "), role, if desc.is_empty() { format!("{f} subcommand") } else { desc });
            t.returns = role == "sim" && has_stop;
            if !args.is_empty() {
                t.hint = format!("Needs {}: add it to a launch configuration and fill it in.", args.join(", "));
            }
            t.args = args;
            tools.push(t);
        }
    }
    if dispatcher {
        kinds.push("command script".into());
    }

    // --- Makefile targets ---
    if names.iter().any(|n| n == "Makefile" || n == "makefile") {
        let text = read_small(&root.join(if names.iter().any(|n| n == "Makefile") { "Makefile" } else { "makefile" }));
        let mut n = 0;
        for line in text.lines() {
            let Some((target, _)) = line.split_once(':') else { continue };
            if line.starts_with(|c: char| c.is_whitespace()) || line.contains(":=") || target.starts_with('.') || target.is_empty() || !target.chars().all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '-') {
                continue;
            }
            let role = role_of_word(target);
            tools.push(mk(format!("make:{target}"), format!("{}: make {target}", script_label(role)), format!("make {target}"), role, "Makefile target".into()));
            n += 1;
            if n >= 15 { break; }
        }
        if n > 0 { kinds.push("Makefile".into()); }
    }

    // --- Python entry points ---
    let mut py = 0;
    for f in names.iter().filter(|n| n.ends_with(".py") && !n.starts_with("test_") && !n.starts_with('_') && !["setup.py", "conf.py", "conftest.py"].contains(&n.as_str())) {
        let text = read_small(&root.join(f));
        if !text.contains("__main__") {
            continue;
        }
        let base = f.trim_end_matches(".py");
        let role = if base.contains("train") { "train" } else if contains_any(base, &["make_", "peek", "show", "sweep", "util", "convert", "export", "control"]) { "tool" } else { "run" };
        let mut t = mk(f.clone(), format!("{}: {f}", role_label(role)), format!("python3 {f}"), role, { let d = py_summary(&text, f); if d.is_empty() { "Python script".into() } else { d } });
        t.needs = vec!["python3".into()];
        tools.push(t);
        py += 1;
        if py >= 40 { break; }
    }
    if py > 0 || names.iter().any(|n| n == "pyproject.toml" || n == "requirements.txt" || n == "setup.py") {
        kinds.push("Python".into());
    }
    for v in [".venv/bin/python", "venv/bin/python", "env/bin/python"] {
        if python.is_none() && root.join(v).is_file() {
            *python = Some(root.join(v).to_string_lossy().to_string());
        }
    }

    // --- tests and docs ---
    let pyproject = read_small(&root.join("pyproject.toml"));
    if names.iter().any(|n| n == "pytest.ini" || n == "conftest.py") || pyproject.contains("[tool.pytest") || (py > 0 && root.join("tests").is_dir()) {
        let mut t = mk("pytest".into(), "Test: pytest".into(), "python3 -m pytest -q".into(), "test", "the test suite".into());
        t.needs = vec!["python3".into()];
        tools.push(t);
    }
    if root.join("docs/conf.py").is_file() && root.join("docs/Makefile").is_file() {
        tools.push(mk("docs".into(), "Docs: build html".into(), "make -C docs html".into(), "docs", "Sphinx site in docs/".into()));
    } else if names.iter().any(|n| n == "mkdocs.yml") {
        tools.push(mk("docs".into(), "Docs: mkdocs serve".into(), "mkdocs serve".into(), "docs", "MkDocs site".into()));
    }

    // --- Gazebo worlds in the folder or its worlds/ ---
    let mut worlds: Vec<PathBuf> = names.iter().map(|n| root.join(n)).collect();
    if let Ok(rd) = std::fs::read_dir(root.join("worlds")) {
        let mut more: Vec<PathBuf> = rd.flatten().map(|e| e.path()).filter(|p| p.is_file()).collect();
        more.sort();
        worlds.extend(more);
    }
    let mut n = 0;
    for w in worlds.into_iter().filter(|p| has_ext(p, "world") || (has_ext(p, "sdf") && read_small(p).contains("<world"))) {
        let relp = w.strip_prefix(root).unwrap_or(&w).to_string_lossy().to_string();
        let mut t = mk(format!("world:{relp}"), format!("World: {}", stem(&file_name(&w)).trim_end_matches(".sdf").trim_end_matches(".world")), format!("gz sim {}", sh_quote(&relp)), "sim", format!("Gazebo world {relp}"));
        t.needs = vec!["gz".into()];
        t.hint = "Gazebo (gz) is not in this environment.".into();
        tools.push(t);
        n += 1;
        if n >= 12 { break; }
    }
    if n > 0 { kinds.push("Gazebo worlds".into()); }
}

/// Inspect the project. `env` is the activated project environment; pass an
/// empty map to skip everything that depends on it.
pub fn scan(dir: &Path, proj: &Project, env: &HashMap<String, String>) -> Scan {
    let ros2 = proj.ros.as_deref().map_or(false, |r| r.starts_with("ros2")) || proj.workspaces.iter().any(|w| w.kind == "ros2");
    let ros1 = !ros2 && (proj.ros.as_deref().map_or(false, |r| r.starts_with("ros1")) || proj.workspaces.iter().any(|w| w.kind == "ros1"));
    let ros_ws = proj.workspaces.iter().find(|w| w.is_ros()).map(|w| w.kind.clone());
    let distro = proj.ros.as_deref().and_then(|r| r.split('-').nth(1)).unwrap_or("<distro>").to_string();
    let rel = |p: &Path| p.strip_prefix(dir).map(|r| r.display().to_string()).unwrap_or_else(|_| p.display().to_string());
    let launcher = if ros1 { "roslaunch" } else { "ros2" };

    // --- loose files: RViz configs and worlds ---
    let mut files = Vec::new();
    for w in &proj.workspaces {
        walk(&w.abs(dir), 6, &mut files);
    }
    if proj.workspaces.is_empty() {
        walk(dir, 5, &mut files);
    }
    files.sort();
    files.dedup();
    let rviz_cfg = files.iter().find(|p| has_ext(p, "rviz")).cloned();
    let world = files.iter().find(|p| has_ext(p, "world") || (has_ext(p, "sdf") && in_dir_named(p, "worlds"))).cloned();

    // --- workspace packages and their launch files ---
    let mut entries: Vec<Entry> = Vec::new();
    let mut ws_pkgs: BTreeMap<String, PathBuf> = BTreeMap::new();
    let mut urdf_dirs: Vec<PathBuf> = Vec::new();
    for w in proj.workspaces.iter().filter(|w| w.is_ros()) {
        let root = w.abs(dir);
        let mut pkgs = Vec::new();
        source_packages(&root.join("src"), 6, &mut pkgs);
        if pkgs.is_empty() {
            source_packages(&root, 3, &mut pkgs);
        }
        for (name, pdir) in pkgs {
            let built = if w.kind == "ros2" { root.join("install").join(&name).is_dir() || root.join("install/share").join(&name).is_dir() } else { root.join("devel").is_dir() };
            launch_entries(&name, &pdir.join("launch"), Some(&w.kind), built, &mut entries);
            urdf_dirs.push(pdir.join("urdf"));
            ws_pkgs.insert(name, pdir);
        }
    }

    // --- companion packages of the same family in the environment ---
    let families: BTreeSet<String> = ws_pkgs.keys().filter_map(|n| n.split('_').next()).filter(|t| t.len() >= 3 && !COMMON.contains(t)).map(String::from).collect();
    let mut prefixes: Vec<PathBuf> = Vec::new();
    for var in ["AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH"] {
        for p in env.get(var).map(String::as_str).unwrap_or("").split(':').filter(|p| !p.is_empty()) {
            let p = PathBuf::from(p);
            let own = proj.workspaces.iter().any(|w| p.starts_with(w.abs(dir)));
            if !own && !prefixes.contains(&p) {
                prefixes.push(p);
            }
        }
    }
    let mut env_pkgs: BTreeMap<String, PathBuf> = BTreeMap::new(); // name -> prefix
    let mut teleop_generic = None;
    let mut map_saver = false;
    for prefix in &prefixes {
        let share = prefix.join("share");
        if teleop_generic.is_none() && share.join("teleop_twist_keyboard").is_dir() {
            teleop_generic = Some(prefix.clone());
        }
        map_saver |= share.join("nav2_map_server").is_dir();
        if families.is_empty() {
            continue;
        }
        let Ok(rd) = std::fs::read_dir(&share) else { continue };
        for e in rd.flatten() {
            let name = e.file_name().to_string_lossy().to_string();
            let fam = name.split('_').next().unwrap_or("");
            if families.contains(fam) && !ws_pkgs.contains_key(&name) && !env_pkgs.contains_key(&name) && e.path().is_dir() {
                env_pkgs.insert(name, prefix.clone());
            }
        }
    }
    let before = entries.len();
    for (name, prefix) in &env_pkgs {
        launch_entries(name, &prefix.join("share").join(name).join("launch"), None, true, &mut entries);
        urdf_dirs.push(prefix.join("share").join(name).join("urdf"));
        if entries.len() - before > 60 {
            break;
        }
    }

    // --- classify ---
    let has_sim = entries.iter().any(|e| role_of(e) == "sim") || world.is_some() || proj.modules.iter().any(|m| m == "gazebo");
    let mut need_env: BTreeSet<String> = BTreeSet::new();
    let mut tools: Vec<Tool> = Vec::new();
    // Variables each launch file needs, including those of the files it includes.
    let mut per: Vec<BTreeSet<String>> = entries.iter().map(|e| { let mut s = BTreeSet::new(); required_env(&e.content, &mut s); s }).collect();
    for _ in 0..3 {
        for i in 0..entries.len() {
            for j in 0..entries.len() {
                if i != j && entries[i].pkg == entries[j].pkg && entries[i].content.contains(&entries[j].file) {
                    let add: Vec<String> = per[j].iter().cloned().collect();
                    per[i].extend(add);
                }
            }
        }
    }
    const STD_ENV: &[&str] = &["HOME", "USER", "PATH", "ROS_DISTRO", "PWD", "DISPLAY"];
    for (i, e) in entries.iter().enumerate() {
        per[i].retain(|n| !STD_ENV.contains(&n.as_str()));
        need_env.extend(per[i].iter().cloned());
        let role = role_of(e);
        let helper = entries.iter().enumerate().any(|(j, o)| j != i && o.pkg == e.pkg && o.content.contains(&e.file));
        let mut cmd = if ros1 { format!("roslaunch {} {}", e.pkg, e.file) } else { format!("ros2 launch {} {}", e.pkg, e.file) };
        if has_sim && role != "sim" && e.content.contains("use_sim_time") {
            cmd += if ros1 { " use_sim_time:=true" } else { " use_sim_time:=True" };
        }
        let saved_map = dir.join("maps/map.yaml");
        if role == "nav" && saved_map.is_file() && e.content.contains("'map'") {
            cmd += &format!(" map:={}", sh_quote(&saved_map.to_string_lossy()));
        }
        let mut needs = vec![launcher.to_string()];
        if contains_any(&e.content, &["gz_sim", "ros_gz_sim"]) {
            needs.push("gz".into());
        }
        let hint = if !e.built {
            format!("{} is not built yet. Run the workspace build first.", e.pkg)
        } else {
            format!("Not available in this environment. Gazebo: sudo apt install ros-{distro}-ros-gz. Otherwise run Sync SDKs.")
        };
        tools.push(Tool {
            id: format!("{}/{}", e.pkg, e.file),
            name: format!("{}: {}", role_label(role), stem(&e.file)),
            cmd,
            workspace: e.ws.clone().or_else(|| ros_ws.clone()),
            needs,
            role: role.into(),
            stage: match role { "sim" => 0, "slam" | "viz" | "control" => 1, _ => 2 },
            helper,
            hint,
            detail: format!("{} · {}", e.pkg, if e.ws.is_some() { "workspace" } else { "environment" }),
            available: e.built, // refined below
            env: per[i].iter().cloned().collect(),
            opens_rviz: e.content.contains("rviz"),
            ..Default::default()
        });
    }

    // --- keyboard teleop executables (need a terminal) ---
    let mut teleops: Vec<(String, String, PathBuf)> = Vec::new(); // pkg, exe, lib dir
    for (name, prefix) in &env_pkgs {
        teleops.push((name.clone(), String::new(), prefix.join("lib").join(name)));
    }
    for w in proj.workspaces.iter().filter(|w| w.kind == "ros2") {
        for name in ws_pkgs.keys() {
            teleops.push((name.clone(), String::new(), w.abs(dir).join("install").join(name).join("lib").join(name)));
        }
    }
    let mut seen = BTreeSet::new();
    for (pkg, _, lib) in teleops {
        let Ok(rd) = std::fs::read_dir(&lib) else { continue };
        let mut exes: Vec<String> = rd.flatten().map(|e| e.file_name().to_string_lossy().to_string()).filter(|n| n.contains("teleop") && !n.contains("joy")).collect();
        exes.sort();
        for exe in exes {
            if seen.insert((pkg.clone(), exe.clone())) {
                tools.push(Tool {
                    id: format!("{pkg}/{exe}"),
                    name: format!("Control: {exe}"),
                    cmd: format!("ros2 run {pkg} {exe}"),
                    needs: vec!["ros2".into()],
                    role: "control".into(),
                    stage: 1,
                    terminal: true,
                    available: true,
                    detail: format!("{pkg} · keyboard, opens in a terminal"),
                    hint: "ros2 is not in this environment. Run Sync SDKs.".into(),
                    ..Default::default()
                });
            }
        }
    }
    if ros2 && !tools.iter().any(|t| t.role == "control" && t.terminal) && teleop_generic.is_some() && !entries.is_empty() {
        tools.push(Tool {
            id: "teleop_twist_keyboard".into(),
            name: "Control: teleop_twist_keyboard".into(),
            cmd: "ros2 run teleop_twist_keyboard teleop_twist_keyboard".into(),
            needs: vec!["ros2".into()],
            role: "control".into(),
            stage: 1,
            terminal: true,
            available: true,
            detail: "generic /cmd_vel keyboard teleop, opens in a terminal".into(),
            hint: "ros2 is not in this environment. Run Sync SDKs.".into(),
            ..Default::default()
        });
    }
    if ros2 && map_saver && tools.iter().any(|t| t.role == "slam") {
        tools.push(Tool {
            id: "save_map".into(),
            name: "Navigation: save map".into(),
            cmd: "mkdir -p \"$RF_PROJECT/maps\" && ros2 run nav2_map_server map_saver_cli -f \"$RF_PROJECT/maps/map\"".into(),
            needs: vec!["ros2".into()],
            role: "nav".into(),
            stage: 2,
            available: true,
            detail: "writes maps/map.yaml in the project; navigation launches pick it up".into(),
            hint: "ros2 is not in this environment. Run Sync SDKs.".into(),
            ..Default::default()
        });
    }

    // --- plain folders: scripts, Python entry points, tests, docs, worlds ---
    let mut notes: Vec<String> = Vec::new();
    let mut python: Option<String> = None;
    let mut kinds: Vec<String> = Vec::new();
    if ros2 { kinds.push("ROS 2 workspace".into()); } else if ros1 { kinds.push("ROS 1 workspace".into()); }
    if proj.workspaces.is_empty() {
        folder_tools(dir, None, &mut tools, &mut notes, &mut python, &mut kinds);
    }
    for w in &proj.workspaces {
        folder_tools(&w.abs(dir), Some(&w.kind), &mut tools, &mut notes, &mut python, &mut kinds);
    }
    kinds.dedup();

    // --- fallbacks that need no launch file ---
    let has_sim_launch = tools.iter().any(|t| t.role == "sim");
    if !has_sim_launch && (proj.modules.iter().any(|m| m == "gazebo") || world.is_some()) {
        let rc = proj.run.iter().find(|r| r.name.to_ascii_lowercase().starts_with("gazebo"));
        let picked = match (rc, &world) {
            (Some(rc), _) => Some((rc.cmd.clone(), rc.workspace.clone(), format!("run configuration \"{}\"", rc.name))),
            (None, Some(w)) => Some((format!("gz sim -r {}", sh_quote(&w.to_string_lossy())), None, format!("world {}", rel(w)))),
            (None, None) => None,
        };
        if let Some((cmd, workspace, detail)) = picked {
            tools.push(Tool {
                id: "gazebo".into(),
                name: "Gazebo".into(),
                cmd,
                workspace,
                needs: vec!["gz".into()],
                role: "sim".into(),
                stage: 0,
                available: true,
                hint: if ros2 { format!("Gazebo (gz) is not in this environment. Install it with: sudo apt install ros-{distro}-ros-gz  (or run Sync SDKs).") } else { "Gazebo (gz) is not in this environment.".into() },
                detail,
                ..Default::default()
            });
        }
    }
    if ros2 || ros1 {
        let (bin, pkg) = if ros2 { ("rviz2", format!("ros-{distro}-rviz2")) } else { ("rviz", format!("ros-{distro}-rviz")) };
        let (mut cmd, detail) = match &rviz_cfg {
            Some(c) => (format!("{bin} -d {}", sh_quote(&c.to_string_lossy())), format!("config {}", rel(c))),
            None => (bin.to_string(), "default configuration".into()),
        };
        if ros2 && has_sim {
            cmd += " --ros-args -p use_sim_time:=true";
        }
        tools.push(Tool {
            id: "rviz".into(),
            name: "RViz".into(),
            cmd,
            workspace: ros_ws.clone(),
            needs: vec![bin.into()],
            role: "viz".into(),
            stage: 1,
            available: true,
            hint: format!("{bin} is not in this environment. Install it with: sudo apt install {pkg}  (or run Sync SDKs)."),
            detail,
            ..Default::default()
        });
        tools.push(Tool {
            id: "rqt_graph".into(),
            name: "rqt graph".into(),
            cmd: "rqt_graph".into(),
            workspace: ros_ws.clone(),
            needs: vec!["rqt_graph".into()],
            role: "viz".into(),
            stage: 2,
            available: true,
            hint: format!("rqt_graph is not in this environment. Install it with: sudo apt install ros-{distro}-rqt-graph"),
            detail: "node and topic graph".into(),
            ..Default::default()
        });
    }

    // --- availability, then one primary per role ---
    for t in &mut tools {
        if !env.is_empty() {
            t.missing = t.needs.iter().filter(|n| !on_path(env, n)).cloned().collect();
        }
        t.available = t.available && t.missing.is_empty();
    }
    let score = |t: &Tool| -> i32 {
        let n = t.id.to_ascii_lowercase();
        let mut s = 0;
        if t.helper { s -= 20; }
        if !t.available { s -= 10; }
        if t.detail.ends_with("workspace") { s += 1; }
        if t.role == "sim" {
            if n.contains("world") { s += 3; }
            if n.contains("empty") { s -= 2; }
            if contains_any(&n, &["dqn", "stage", "test", "autorace", "fake", "spawn"]) { s -= 3; }
        }
        if t.role == "control" && t.terminal { s += 2; }
        if t.role == "viz" && t.id == "rviz" { s += 1; }
        if t.id == "rqt_graph" || t.id == "save_map" { s -= 5; }
        if !t.args.is_empty() { s -= 8; }
        if t.id.contains(".sh:") || t.id.starts_with("make:") { s += 4; }   // the project's own entry points
        if t.id.starts_with("world:") { s -= 1; }
        if t.id.rsplit(':').next() == Some(t.role.as_str()) { s += 3; }      // `run.sh train` is the train command
        s
    };
    let roles: BTreeSet<String> = tools.iter().map(|t| t.role.clone()).collect();
    for role in roles {
        let best = tools.iter().enumerate().filter(|(_, t)| t.role == role).max_by_key(|(i, t)| (score(t), std::cmp::Reverse(*i))).map(|(i, _)| i);
        if let Some(i) = best {
            tools[i].primary = true;
        }
    }
    const ORDER: &[&str] = &["sim", "slam", "nav", "control", "viz", "manip", "bringup", "run", "train", "test", "docs", "stop", "tool", "other"];
    tools.sort_by_key(|t| (ORDER.iter().position(|r| *r == t.role).unwrap_or(9), t.helper, !t.primary));

    // --- required environment variables and sensible choices ---
    let mut env_out = Vec::new();
    for name in need_env {
        let value = proj.env.get(&name).cloned().or_else(|| env.get(&name).cloned()).filter(|v| !v.is_empty());
        let mut choices: BTreeSet<String> = BTreeSet::new();
        if families.iter().any(|f| name.eq_ignore_ascii_case(&format!("{f}_MODEL"))) {
            for d in &urdf_dirs {
                for e in std::fs::read_dir(d).into_iter().flatten().flatten() {
                    let n = e.file_name().to_string_lossy().to_string();
                    let base = n.split('.').next().unwrap_or("");
                    for fam in &families {
                        if let Some(model) = base.strip_prefix(&format!("{fam}_")) {
                            if !model.is_empty() {
                                choices.insert(model.to_string());
                            }
                        }
                    }
                }
            }
        }
        env_out.push(EnvNeed { name, value, choices: choices.into_iter().collect() });
    }

    // --- summary for the "project loaded" notice ---
    let count = |role: &str| tools.iter().filter(|t| t.role == role && !t.helper && t.id != "rviz" && t.id != "rqt_graph" && t.id != "gazebo" && t.id != "save_map").count();
    let mut parts = Vec::new();
    for (role, one, many) in [("sim", "simulation world", "simulation worlds"), ("slam", "SLAM launch", "SLAM launches"), ("nav", "navigation launch", "navigation launches"), ("control", "teleop", "teleop options"), ("manip", "manipulation launch", "manipulation launches"), ("bringup", "bringup launch", "bringup launches"), ("run", "run command", "run commands"), ("train", "training command", "training commands"), ("test", "test command", "test commands"), ("docs", "docs build", "docs builds"), ("tool", "tool", "tools")] {
        match count(role) {
            0 => {}
            1 => parts.push(format!("1 {one}")),
            n => parts.push(format!("{n} {many}")),
        }
    }
    let suggested = suggest(&tools);
    let suggested_setup = python.map(|p| Setup { python: Some(p), ..Default::default() });
    let kind = if kinds.is_empty() { "project folder".to_string() } else { kinds.join(" + ") };
    Scan { tools, env: env_out, summary: parts.join(", "), suggested, kind, notes, suggested_setup }
}

/// Tools detectable without the environment (kept for callers that only
/// need the list).
pub fn candidates(dir: &Path, proj: &Project) -> Vec<Tool> {
    scan(dir, proj, &HashMap::new()).tools
}
