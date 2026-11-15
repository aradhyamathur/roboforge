#![recursion_limit = "512"]
#[cfg(not(unix))]
compile_error!("RoboForge runs on Linux (and probably macOS). On Windows, build and run it inside WSL2: the backend needs bash, Unix process groups and signals, and the SDK recipes (RoboStack, PX4 SITL, Gazebo, Isaac Sim) target Linux.");

mod http;
mod jobs;
mod server;
mod term;

use roboforge_core::catalog::Catalog;
use roboforge_core::modules::Registry;
use roboforge_core::paths::Home;
use roboforge_core::plan::{mark_done, plan};
use roboforge_core::project::{expand_tilde, wrap, write_activation, Project, Workspace};
use roboforge_core::resolve::resolve;
use roboforge_core::state::State;
use roboforge_core::templates::{adopt, generate, preset_selection, project_components, project_selection, ros_packages, set_modules, set_workspaces, new_package, PRESETS};
use std::path::{Path, PathBuf};
use std::process::Command;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

const USAGE: &str = "RoboForge: robotics IDE with isolated, side-by-side SDKs

USAGE
  roboforge [ide] [--port N] [--no-open] [--browser]
                                           start the IDE (default). Desktop builds open a native
                                           window; --browser opens a browser tab instead
  roboforge presets                        list project presets
  roboforge modules                        list modules you can mix into a project
  roboforge new <preset> <dir> [--name N] [--ros DISTRO] [--modules a,b] [--with sdk,sdk]
                [--workspace PATH]         put the ROS package into an existing workspace
  roboforge init [dir] [--ros DISTRO] [--name N]
                                           set up an existing ROS workspace in place
  roboforge add <module>... [--dir D]      mix modules into an existing project
  roboforge remove <module>... [--dir D]   take modules out again
  roboforge ws [--dir D]                   list workspaces
  roboforge ws add <kind> <path> [--dir D] add a workspace (ros2, ros1, mav, sim, ml, isaac, ...)
  roboforge ws rm <path> [--dir D]         drop a workspace from the config (folder stays)
  roboforge pkg <name> [--ws N] [--cmake] [--dir D]   new ROS package in workspace N (default 0)
  roboforge sync [dir]                     install the SDKs a project pins
  roboforge run <config> [dir]             run a project's run configuration
  roboforge sdk list                       catalog and what is installed
  roboforge sdk plan <component>...        show install steps, change nothing
  roboforge sdk install <component>...     install components as one environment

SDKs live under $ROBOFORGE_HOME (default ~/.roboforge). Your own modules go in
$ROBOFORGE_HOME/modules/<id>/ (module.json plus files/).";

static STOP: AtomicBool = AtomicBool::new(false);

extern "C" fn on_signal(_: libc::c_int) {
    STOP.store(true, Ordering::SeqCst);
}

fn die(msg: impl AsRef<str>) -> ! {
    eprintln!("roboforge: {}", msg.as_ref());
    std::process::exit(1)
}

fn bash(script: &str, cwd: &Path) -> i32 {
    Command::new("bash")
        .arg("-c")
        .arg(script)
        .current_dir(cwd)
        .status()
        .ok()
        .and_then(|s| s.code())
        .unwrap_or(130)
}

fn install(home: &Home, cat: &Catalog, comps: &[String], dry: bool) {
    let res = resolve(cat, comps).unwrap_or_else(|e| die(e));
    let units = plan(home, cat, &res, &State::load(home));
    if units.is_empty() {
        println!("Everything is already installed.");
        return;
    }
    for (i, u) in units.iter().enumerate() {
        println!("\n== [{}/{}] {}", i + 1, units.len(), u.label);
        for st in &u.steps {
            println!("-> {}", st.title);
            if dry {
                for l in st.cmd.lines() {
                    println!("     {l}");
                }
                continue;
            }
            std::fs::create_dir_all(&home.root).unwrap_or_else(|e| die(e.to_string()));
            let code = bash(&format!("set -e\n{}", st.cmd), &home.root);
            if code != 0 {
                die(format!("step failed (exit {code}). Finished units are kept; run again to resume."));
            }
        }
        if !dry {
            mark_done(home, &u.key).unwrap_or_else(|e| die(e.to_string()));
        }
    }
}

fn flag(args: &mut Vec<String>, name: &str) -> Option<String> {
    let i = args.iter().position(|a| a == name)?;
    if i + 1 >= args.len() {
        die(format!("{name} needs a value"));
    }
    args.remove(i);
    Some(args.remove(i))
}

fn abs(p: &str) -> PathBuf {
    let p = expand_tilde(p);
    let p = if p.is_absolute() { p } else { std::env::current_dir().unwrap().join(p) };
    // Drop trailing "." / ".." components so paths print and compare cleanly.
    let mut out = PathBuf::new();
    for c in p.components() {
        match c {
            std::path::Component::CurDir => {}
            std::path::Component::ParentDir => { out.pop(); }
            other => out.push(other),
        }
    }
    out
}

/// Show the IDE in a native Tauri window and block until it is closed
/// (or Ctrl+C / SIGTERM arrives). The HTTP server keeps running in-process.
#[cfg(feature = "desktop")]
fn desktop(url: &str, ctx: Arc<server::Ctx>) {
    use tauri::{WebviewUrl, WebviewWindowBuilder};
    let target: tauri::Url = url.parse().expect("valid loopback url");
    let app = tauri::Builder::default()
        .setup(move |app| {
            WebviewWindowBuilder::new(app, "main", WebviewUrl::External(target))
                .title("RoboForge")
                .inner_size(1400.0, 900.0)
                .min_inner_size(900.0, 600.0)
                .build()?;
            let handle = app.handle().clone();
            std::thread::spawn(move || {
                while !STOP.load(Ordering::SeqCst) {
                    std::thread::sleep(std::time::Duration::from_millis(200));
                }
                handle.exit(0);
            });
            Ok(())
        })
        .build(tauri::generate_context!())
        .unwrap_or_else(|e| die(format!("cannot open the window: {e}")));
    // Tauri ends the process itself once the event loop finishes, so the
    // clean-up has to happen on its Exit event, not after `run`.
    app.run(move |_, event| {
        if let tauri::RunEvent::Exit = event {
            STOP.store(true, Ordering::SeqCst);
            eprintln!("\nStopping running jobs...");
            ctx.jobs.shutdown();
            ctx.terms.shutdown();
        }
    });
    STOP.store(true, Ordering::SeqCst);
}

fn main() {
    let mut args: Vec<String> = std::env::args().skip(1).collect();
    let home = Home::discover();
    let cmd = if args.first().map_or(true, |a| a.starts_with("--")) { "ide".to_string() } else { args.remove(0) };
    if args.iter().any(|a| a == "-h" || a == "--help") || cmd == "help" {
        println!("{USAGE}");
        return;
    }
    let cat = Catalog::load(&home).unwrap_or_else(|e| die(e));
    let reg = Registry::load(&home, &cat).unwrap_or_else(|e| die(e));

    match cmd.as_str() {
        "ide" => {
            let asked: Option<u16> = flag(&mut args, "--port").map(|p| p.parse().unwrap_or_else(|_| die("bad --port")));
            let port = asked.unwrap_or(7878);
            let no_open = args.iter().any(|a| a == "--no-open");
            let listener = match std::net::TcpListener::bind(("127.0.0.1", port)) {
                Ok(l) => l,
                // The default port is taken (often another RoboForge): use any free one.
                Err(e) if asked.is_none() => {
                    let l = std::net::TcpListener::bind(("127.0.0.1", 0)).unwrap_or_else(|e| die(format!("cannot listen on 127.0.0.1: {e}")));
                    eprintln!("Port {port} is in use ({e}); another RoboForge may be running. Using port {} instead.", l.local_addr().map(|a| a.port()).unwrap_or(0));
                    l
                }
                Err(e) => die(format!("cannot listen on 127.0.0.1:{port}: {e}")),
            };
            let port = listener.local_addr().unwrap().port();
            let ctx = Arc::new(server::Ctx { home, token: server::random_token(), port, jobs: Default::default(), terms: Default::default(), vscode: Default::default() });
            let url = format!("http://127.0.0.1:{port}/");
            println!("RoboForge is running at {url}\nSDK home: {}\nCtrl+C to quit.", ctx.home.root.display());
            let browser = args.iter().any(|a| a == "--browser");
            #[cfg(feature = "desktop")]
            let window = !browser && !no_open;
            #[cfg(not(feature = "desktop"))]
            let window = {
                let _ = browser;
                false
            };
            if !no_open && !window {
                let _ = Command::new("xdg-open").arg(&url).stderr(std::process::Stdio::null()).spawn();
            }
            unsafe {
                libc::signal(libc::SIGINT, on_signal as *const () as libc::sighandler_t);
                libc::signal(libc::SIGTERM, on_signal as *const () as libc::sighandler_t);
            }
            let srv = Arc::clone(&ctx);
            std::thread::spawn(move || server::serve(srv, listener));
            #[cfg(feature = "desktop")]
            if window {
                desktop(&url, Arc::clone(&ctx));
                return;
            }
            while !STOP.load(Ordering::SeqCst) {
                std::thread::sleep(std::time::Duration::from_millis(200));
            }
            eprintln!("\nStopping running jobs...");
            ctx.jobs.shutdown();
            ctx.terms.shutdown();
        }
        "presets" => {
            for p in PRESETS {
                println!("{:<13} {}\n{:<13} {}", p.id, p.name, "", p.summary);
            }
        }
        "modules" => {
            let mut cats: Vec<&str> = reg.modules.iter().map(|m| m.def.category.as_str()).collect();
            cats.sort_unstable();
            cats.dedup();
            for c in cats {
                println!("{c}");
                for m in reg.modules.iter().filter(|m| m.def.category == c) {
                    let ros = if m.def.ros == "any" { String::new() } else { format!(" [{}]", m.def.ros) };
                    println!("  {:<14} {}{ros}{}\n  {:<14} {}", m.def.id, m.def.name, if m.def.builtin { "" } else { " (yours)" }, "", m.def.summary);
                }
            }
        }
        "new" => {
            let ros = flag(&mut args, "--ros").unwrap_or_else(|| "jazzy".into());
            let name = flag(&mut args, "--name");
            let with = flag(&mut args, "--with");
            let modules = flag(&mut args, "--modules");
            let workspace = flag(&mut args, "--workspace");
            if args.len() != 2 {
                die("usage: roboforge new <preset> <dir> [--name N] [--ros DISTRO] [--modules a,b] [--with sdk,sdk]");
            }
            let mut sel = preset_selection(&args[0], &format!("ros2-{ros}"))
                .unwrap_or_else(|| die(format!("unknown preset {} (see: roboforge presets)", args[0])));
            let add = |list: &mut Vec<String>, extra: Option<String>| {
                for x in extra.iter().flat_map(|w| w.split(',')).filter(|s| !s.is_empty()) {
                    if !list.iter().any(|c| c == x) {
                        list.push(x.to_string());
                    }
                }
            };
            add(&mut sel.modules, modules);
            add(&mut sel.components, with);
            sel.ros_workspace = workspace.map(|w| abs(&w).to_string_lossy().to_string());
            let dir = abs(&args[1]);
            let name = name.unwrap_or_else(|| dir.file_name().map(|n| n.to_string_lossy().to_string()).unwrap_or_default());
            let proj = generate(&home, &cat, &reg, &dir, &name, &sel).unwrap_or_else(|e| die(e));
            let comps = project_components(&cat, &reg, &proj).unwrap_or_else(|e| die(e));
            println!("Created {} in {}\nModules: {}\nSDK components: {}", proj.name, dir.display(),
                if proj.modules.is_empty() { "(none)".into() } else { proj.modules.join(", ") }, comps.join(", "));
            println!("Workspaces:");
            for w in &proj.workspaces {
                println!("  {:<6} {}", w.kind, w.abs(&dir).display());
            }
            println!("Next: cd {} && roboforge sync", dir.display());
        }
        "init" => {
            let ros = flag(&mut args, "--ros").unwrap_or_else(|| "jazzy".into());
            let name = flag(&mut args, "--name").unwrap_or_default();
            let dir = abs(args.first().map(String::as_str).unwrap_or("."));
            let ros = if ros == "noetic" { "ros1-noetic".to_string() } else { format!("ros2-{ros}") };
            let proj = adopt(&home, &cat, &reg, &dir, &name, &ros).unwrap_or_else(|e| die(e));
            let pkgs = ros_packages(&dir);
            println!("Set up {} as project {} on {ros}. Your packages were not touched ({} found{}).", dir.display(), proj.name, pkgs.len(),
                if pkgs.is_empty() { String::new() } else { format!(": {}", pkgs.join(", ")) });
            println!("Added: roboforge.json, .roboforge/\nNext: roboforge sync, then roboforge add <module> to mix modules in.");
        }
        "add" | "remove" => {
            let dir = abs(&flag(&mut args, "--dir").unwrap_or_else(|| ".".into()));
            if args.is_empty() {
                die(format!("usage: roboforge {cmd} <module>... [--dir D]"));
            }
            let proj = Project::load(&dir).unwrap_or_else(|e| die(e));
            let mut modules = project_selection(&cat, &proj).modules;
            if cmd == "add" {
                for m in &args {
                    if !modules.contains(m) {
                        modules.push(m.clone());
                    }
                }
            } else {
                for m in &args {
                    if !modules.contains(m) {
                        die(format!("{m} is not in this project (has: {})", modules.join(", ")));
                    }
                }
                modules.retain(|m| !args.contains(m));
            }
            let ch = set_modules(&home, &cat, &reg, &dir, &modules).unwrap_or_else(|e| die(e));
            println!("Modules: {}", if ch.project.modules.is_empty() { "(none)".into() } else { ch.project.modules.join(", ") });
            for f in &ch.written {
                println!("  created {f}");
            }
            for f in &ch.left {
                println!("  left in place (delete if unwanted): {f}");
            }
            println!("Run `roboforge sync` if the SDK set changed.");
        }
        "ws" => {
            let dir = abs(&flag(&mut args, "--dir").unwrap_or_else(|| ".".into()));
            let proj = Project::load(&dir).unwrap_or_else(|e| die(e));
            let sub = args.first().cloned().unwrap_or_else(|| "list".into());
            let mut list = proj.workspaces.clone();
            match sub.as_str() {
                "add" if args.len() == 3 => list.push(Workspace { kind: args[1].clone(), path: args[2].clone() }),
                "rm" if args.len() == 2 => {
                    let n = list.len();
                    list.retain(|w| w.path != args[1]);
                    if list.len() == n { die(format!("no workspace with path {}", args[1])) }
                }
                "list" => {}
                _ => die("usage: roboforge ws [add <kind> <path> | rm <path>] [--dir D]"),
            }
            let proj = if sub == "list" { proj } else { set_workspaces(&home, &cat, &reg, &dir, &list).unwrap_or_else(|e| die(e)) };
            for w in &proj.workspaces {
                let pk = if w.is_ros() { let p = ros_packages(&w.abs(&dir)); if p.is_empty() { String::new() } else { format!("  [{}]", p.join(", ")) } } else { String::new() };
                println!("{:<6} {}{}{pk}", w.kind, w.abs(&dir).display(), if w.external() { " (external)" } else { "" });
            }
        }
        "pkg" => {
            let dir = abs(&flag(&mut args, "--dir").unwrap_or_else(|| ".".into()));
            let ws: usize = flag(&mut args, "--ws").map(|n| n.parse().unwrap_or_else(|_| die("bad --ws"))).unwrap_or(0);
            let cmake = args.iter().position(|a| a == "--cmake").map(|i| args.remove(i)).is_some();
            if args.len() != 1 { die("usage: roboforge pkg <name> [--ws N] [--cmake] [--dir D]") }
            let written = new_package(&dir, ws, &args[0], if cmake { "ament_cmake" } else { "ament_python" }).unwrap_or_else(|e| die(e));
            for f in written { println!("  created {f}"); }
            println!("Build it with the workspace's build run configuration.");
        }
        "sync" | "run" => {
            let config = if cmd == "run" {
                if args.is_empty() { die("usage: roboforge run <config> [dir]") }
                Some(args.remove(0))
            } else {
                None
            };
            let dir = abs(args.first().map(String::as_str).unwrap_or("."));
            let proj = Project::load(&dir).unwrap_or_else(|e| die(e));
            let comps = project_components(&cat, &reg, &proj).unwrap_or_else(|e| die(e));
            write_activation(&home, &cat, &dir, &proj, &comps).unwrap_or_else(|e| die(e));
            match config {
                None => install(&home, &cat, &comps, false),
                Some(name) => {
                    let rc = proj.run.iter().find(|r| r.name == name).unwrap_or_else(|| {
                        let names: Vec<&str> = proj.run.iter().map(|r| r.name.as_str()).collect();
                        die(format!("no run configuration {name:?}. Available:\n  {}", names.join("\n  ")))
                    });
                    let cwd = proj.run_dir(&dir, rc);
                    std::fs::create_dir_all(&cwd).unwrap_or_else(|e| die(e.to_string()));
                    std::process::exit(bash(&wrap(&dir, &rc.cmd, rc.env.as_deref()), &cwd));
                }
            }
        }
        "sdk" => {
            let sub = if args.is_empty() { "list".to_string() } else { args.remove(0) };
            match sub.as_str() {
                "list" => {
                    let st = State::load(&home);
                    println!("SDK home: {}\n", home.root.display());
                    for c in &cat.components {
                        let envs: Vec<&str> = st.envs.iter().filter(|(_, m)| m.contains(&c.id)).map(|(e, _)| e.as_str()).collect();
                        let mark = if !envs.is_empty() || st.sdks.contains(&c.id) { "*" } else { " " };
                        println!("{mark} {:<16} {:<10} {}", c.id, c.category, c.name);
                        for e in envs {
                            println!("    in env {e}");
                        }
                    }
                }
                "plan" => install(&home, &cat, &args, true),
                "install" => install(&home, &cat, &args, false),
                _ => die("usage: roboforge sdk list|plan|install"),
            }
        }
        _ => die(format!("unknown command {cmd}\n\n{USAGE}")),
    }
}
