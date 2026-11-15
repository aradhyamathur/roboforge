use crate::catalog::{Catalog, Component};
use crate::paths::Home;
use crate::resolve::{EnvSpec, Resolution};
use crate::sh_quote;
use crate::state::{State, ENV_CREATED};
use serde::Serialize;

#[derive(Debug, Clone, Serialize)]
pub struct Step {
    pub title: String,
    /// A bash script, run with `bash -ec`.
    pub cmd: String,
}

#[derive(Debug, Clone, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum UnitKey {
    Tool,
    Sdk { id: String },
    Env { env: String, member: String },
}

/// A group of steps that is recorded in `state.json` once all succeed.
#[derive(Debug, Clone, Serialize)]
pub struct Unit {
    pub key: UnitKey,
    pub label: String,
    pub steps: Vec<Step>,
}

fn p(path: &std::path::Path) -> String {
    sh_quote(&path.to_string_lossy())
}

fn platform() -> &'static str {
    match std::env::consts::ARCH {
        "aarch64" => "linux-aarch64",
        _ => "linux-64",
    }
}

/// Expand `{sdk}`, `{home}`, `{prefix}` and `{ros_distro}`.
pub fn expand(s: &str, home: &Home, c: &Component, env: Option<&EnvSpec>) -> String {
    let mut out = s.replace("{sdk}", &home.sdk_dir(&c.id).to_string_lossy());
    out = out.replace("{home}", &home.root.to_string_lossy());
    if let Some(e) = env {
        out = out.replace("{prefix}", &home.env_prefix(&e.name).to_string_lossy());
        if let Some(d) = &e.ros_distro {
            out = out.replace("{ros_distro}", d);
        }
    }
    out
}

fn conda_specs(c: &Component, env: &EnvSpec, home: &Home) -> (Vec<String>, Vec<String>) {
    let Some(conda) = &c.conda else { return (vec![], vec![]) };
    let mut specs = conda.specs.clone();
    match env.ros_version {
        Some(2) => specs.extend(conda.with_ros2.iter().cloned()),
        Some(_) => specs.extend(conda.with_ros1.iter().cloned()),
        None => specs.extend(conda.without_ros.iter().cloned()),
    }
    let specs = specs.iter().map(|s| expand(s, home, c, Some(env))).collect();
    (conda.channels.clone(), specs)
}

fn push_unique(v: &mut Vec<String>, items: impl IntoIterator<Item = String>) {
    for i in items {
        if !v.contains(&i) {
            v.push(i);
        }
    }
}

fn mm_env(home: &Home) -> String {
    format!("export MAMBA_ROOT_PREFIX={}", p(&home.mamba_root()))
}

fn env_units(home: &Home, cat: &Catalog, env: &EnvSpec, state: &State, out: &mut Vec<Unit>) {
    let mm = p(&home.micromamba());
    let prefix = p(&home.env_prefix(&env.name));
    let comps: Vec<&Component> = env.components.iter().filter_map(|i| cat.get(i)).collect();
    let created = state.env_has(&env.name, ENV_CREATED);

    if !created {
        // One solve for the whole environment: ROS, compilers and simulator
        // packages must agree on Python and library versions.
        let mut channels = vec!["conda-forge".to_string()];
        let mut specs = Vec::new();
        if let Some(py) = &env.python {
            specs.push(format!("python={py}"));
        }
        specs.push("pip".to_string());
        specs.push("git".to_string());
        for c in &comps {
            let (ch, sp) = conda_specs(c, env, home);
            push_unique(&mut channels, ch);
            push_unique(&mut specs, sp);
        }
        let ch: String = channels.iter().map(|c| format!(" -c {}", sh_quote(c))).collect();
        let sp: String = specs.iter().map(|s| format!(" {}", sh_quote(s))).collect();
        out.push(Unit {
            key: UnitKey::Env { env: env.name.clone(), member: ENV_CREATED.into() },
            label: format!("Create environment {}", env.name),
            steps: vec![Step {
                title: format!("Solve and install conda packages into {}", env.name),
                cmd: format!("{}\nrm -rf {prefix}\n{mm} create -y -p {prefix}{ch}{sp}", mm_env(home)),
            }],
        });
    }

    for c in comps {
        if state.env_has(&env.name, &c.id) {
            continue;
        }
        let mut steps = Vec::new();
        if created {
            // Environment already exists (a dedicated env gaining a member).
            let (channels, specs) = conda_specs(c, env, home);
            if !specs.is_empty() {
                let mut all = vec!["conda-forge".to_string()];
                push_unique(&mut all, channels);
                let ch: String = all.iter().map(|c| format!(" -c {}", sh_quote(c))).collect();
                let sp: String = specs.iter().map(|s| format!(" {}", sh_quote(s))).collect();
                steps.push(Step {
                    title: format!("conda packages for {}", c.name),
                    cmd: format!("{}\n{mm} install -y -p {prefix}{ch}{sp}", mm_env(home)),
                });
            }
        }
        for args in &c.pip {
            let a: String = args
                .iter()
                .map(|x| format!(" {}", sh_quote(&expand(x, home, c, Some(env)))))
                .collect();
            steps.push(Step {
                title: format!("pip install{a}"),
                cmd: format!("{}\n{mm} run -p {prefix} python -m pip install{a}", mm_env(home)),
            });
        }
        if let Some(src) = &c.source {
            if !src.post.is_empty() {
                let sdk = home.sdk_dir(&c.id);
                let script = src
                    .post
                    .iter()
                    .map(|l| expand(l, home, c, Some(env)))
                    .collect::<Vec<_>>()
                    .join(" && ");
                steps.push(Step {
                    title: format!("Build {}", c.name),
                    cmd: format!(
                        "{}\ncd {sdk}\nif [ ! -f .roboforge-built ]; then\n  {mm} run -p {prefix} bash -ec {}\n  touch .roboforge-built\nfi",
                        mm_env(home),
                        sh_quote(&script),
                        sdk = p(&sdk),
                    ),
                });
            }
        }
        out.push(Unit {
            key: UnitKey::Env { env: env.name.clone(), member: c.id.clone() },
            label: format!("{} → {}", c.name, env.name),
            steps,
        });
    }
}

/// Everything still missing for `res`, in execution order.
pub fn plan(home: &Home, cat: &Catalog, res: &Resolution, state: &State) -> Vec<Unit> {
    let mut out = Vec::new();
    let needs_env = res.main_env.is_some() || !res.own_envs.is_empty();

    if needs_env && !home.micromamba().exists() {
        let tools = home.root.join("tools");
        out.push(Unit {
            key: UnitKey::Tool,
            label: "Bootstrap micromamba".into(),
            steps: vec![Step {
                title: "Download micromamba (single static binary, no root needed)".into(),
                cmd: format!(
                    "mkdir -p {t}\ncd {t}\ncurl -fSL --retry 2 -o micromamba.tar.bz2 https://micro.mamba.pm/api/micromamba/{}/latest\ntar -xjf micromamba.tar.bz2 --strip-components=1 bin/micromamba\nrm micromamba.tar.bz2\n./micromamba --version",
                    platform(),
                    t = p(&tools)
                ),
            }],
        });
    }

    for id in &res.sources {
        if state.sdks.contains(id) {
            continue;
        }
        let c = cat.get(id).unwrap();
        let src = c.source.as_ref().unwrap();
        let dir = home.sdk_dir(id);
        let mut clone = String::from("git clone --depth 1");
        if let Some(rev) = &src.rev {
            clone.push_str(&format!(" --branch {}", sh_quote(rev)));
        }
        if src.recursive {
            clone.push_str(" --recursive --shallow-submodules");
        }
        clone.push_str(&format!(" {} {}", sh_quote(&src.git), p(&dir)));
        out.push(Unit {
            key: UnitKey::Sdk { id: id.clone() },
            label: format!("Fetch {} {}", c.name, src.rev.clone().unwrap_or_default()),
            steps: vec![Step {
                title: format!("Clone {}", src.git),
                cmd: format!("mkdir -p {}\nrm -rf {d}\n{clone}", p(dir.parent().unwrap()), d = p(&dir)),
            }],
        });
    }

    for env in res.main_env.iter().chain(res.own_envs.iter()) {
        env_units(home, cat, env, state, &mut out);
    }
    out
}

/// Record a finished unit.
pub fn mark_done(home: &Home, key: &UnitKey) -> std::io::Result<()> {
    let mut st = State::load(home);
    match key {
        UnitKey::Tool => return Ok(()),
        UnitKey::Sdk { id } => {
            st.sdks.insert(id.clone());
        }
        UnitKey::Env { env, member } => {
            st.envs.entry(env.clone()).or_default().insert(member.clone());
        }
    }
    st.save(home)
}

/// True when nothing is left to install for `res`.
pub fn is_ready(home: &Home, cat: &Catalog, res: &Resolution) -> bool {
    plan(home, cat, res, &State::load(home)).is_empty()
}
