//! Modules are the mix-and-match unit: a paradigm (MoveIt, Nav2, MAVSDK,
//! MuJoCo, ...) described by a `module.json` plus template files. A module
//! names the SDK components it needs and contributes files, package
//! dependencies, run configurations and launch entries to a project.
//!
//! Built-in modules are embedded at compile time; any folder under
//! `<home>/modules/<id>/` with the same layout is loaded at runtime and can
//! override a built-in one.

use crate::catalog::Catalog;
use crate::paths::Home;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

include!(concat!(env!("OUT_DIR"), "/builtin_modules.rs"));

fn any() -> String {
    "any".into()
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct FileDef {
    /// File under the module's `files/` folder.
    pub src: String,
    /// Workspace kind the file belongs to (`ros`, `mav`, `sim`, `ml`,
    /// `isaac`, or one of your own). Defaults to `ros` when `dest` uses
    /// `@PKGDIR@`, otherwise the project root.
    #[serde(default)]
    pub workspace: Option<String>,
    /// Destination relative to that workspace; placeholders expand.
    pub dest: String,
    #[serde(default)]
    pub when: Vec<String>,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct RunDef {
    pub name: String,
    pub cmd: String,
    /// Workspace kind whose folder the command runs in.
    #[serde(default)]
    pub workspace: Option<String>,
    #[serde(default)]
    pub env: Option<String>,
    #[serde(default)]
    pub when: Vec<String>,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct NodeDef {
    pub package: String,
    pub executable: String,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct IncludeDef {
    pub package: String,
    /// Path inside the package share directory, e.g. `launch/x_launch.py`.
    pub file: String,
    #[serde(default)]
    pub args: BTreeMap<String, String>,
}

/// One optional entry in the generated bringup launch file, switched by the
/// launch argument `use_<name>`.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct LaunchDef {
    pub name: String,
    #[serde(default)]
    pub description: String,
    #[serde(default)]
    pub default: bool,
    #[serde(default)]
    pub node: Option<NodeDef>,
    #[serde(default)]
    pub include: Option<IncludeDef>,
    #[serde(default)]
    pub when: Vec<String>,
}

/// `module.json`.
///
/// Placeholders in `dest`, `cmd`, launch fields and file contents:
/// `@PKG@` package name, `@NAME@` project name, `@PKGDIR@` the package
/// folder inside the ROS workspace (`src/<pkg>`), `@ROS_DISTRO@`,
/// `@PX4_BRANCH@`. At run time `$RF_WS_<KIND>` holds each workspace's path.
///
/// `when` tokens (all must hold): `ros2`, `ros1`, `noros`, `mod:<id>`;
/// prefix any of them with `!` to negate.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ModuleDef {
    pub id: String,
    pub name: String,
    pub category: String,
    #[serde(default)]
    pub summary: String,
    /// `any`, `ros2` or `ros1`.
    #[serde(default = "any")]
    pub ros: String,
    /// Catalog ids, or `group:<g>=<default>` for "any member of the group,
    /// this one if the project has none".
    #[serde(default)]
    pub components: Vec<String>,
    /// Other modules pulled in automatically.
    #[serde(default)]
    pub requires: Vec<String>,
    /// ROS 2 package dependencies added to package.xml.
    #[serde(default)]
    pub depends: Vec<String>,
    /// Folders inside the ROS 2 package to install into its share directory.
    #[serde(default)]
    pub data_dirs: Vec<String>,
    #[serde(default)]
    pub files: Vec<FileDef>,
    #[serde(default)]
    pub run: Vec<RunDef>,
    #[serde(default)]
    pub launch: Vec<LaunchDef>,
    #[serde(default, skip_deserializing)]
    pub builtin: bool,
}

#[derive(Debug, Clone)]
pub struct Module {
    pub def: ModuleDef,
    pub files: BTreeMap<String, String>,
}

#[derive(Debug, Clone, Default)]
pub struct Registry {
    pub modules: Vec<Module>,
}

fn read_tree(dir: &std::path::Path, base: &std::path::Path, out: &mut BTreeMap<String, String>) {
    for e in std::fs::read_dir(dir).into_iter().flatten().flatten() {
        let p = e.path();
        if p.is_dir() {
            read_tree(&p, base, out);
        } else if let Ok(text) = std::fs::read_to_string(&p) {
            out.insert(p.strip_prefix(base).unwrap().to_string_lossy().replace('\\', "/"), text);
        }
    }
}

impl Registry {
    pub fn builtin() -> Registry {
        let modules = BUILTIN
            .iter()
            .map(|(json, files)| {
                let mut def: ModuleDef = serde_json::from_str(json).expect("built-in module.json is valid");
                def.builtin = true;
                Module { def, files: files.iter().map(|(k, v)| (k.to_string(), v.to_string())).collect() }
            })
            .collect();
        Registry { modules }
    }

    /// Built-in modules plus `<home>/modules/*`; a user module with a
    /// built-in id replaces it.
    pub fn load(home: &Home, cat: &Catalog) -> Result<Registry, String> {
        let mut reg = Registry::builtin();
        let mut dirs: Vec<_> = std::fs::read_dir(home.modules_dir()).into_iter().flatten().flatten().map(|e| e.path()).collect();
        dirs.sort();
        for d in dirs {
            let manifest = d.join("module.json");
            if !manifest.exists() {
                continue;
            }
            let text = std::fs::read_to_string(&manifest).map_err(|e| format!("{}: {e}", manifest.display()))?;
            let def: ModuleDef = serde_json::from_str(&text).map_err(|e| format!("{}: {e}", manifest.display()))?;
            let mut files = BTreeMap::new();
            read_tree(&d.join("files"), &d.join("files"), &mut files);
            reg.modules.retain(|m| m.def.id != def.id);
            reg.modules.push(Module { def, files });
        }
        reg.validate(cat)?;
        Ok(reg)
    }

    pub fn get(&self, id: &str) -> Option<&Module> {
        self.modules.iter().find(|m| m.def.id == id)
    }

    pub fn validate(&self, cat: &Catalog) -> Result<(), String> {
        for m in &self.modules {
            let d = &m.def;
            let err = |msg: String| Err(format!("module {}: {msg}", d.id));
            if !matches!(d.ros.as_str(), "any" | "ros2" | "ros1") {
                return err(format!("ros must be any, ros2 or ros1, not {:?}", d.ros));
            }
            for r in &d.requires {
                if self.get(r).is_none() {
                    return err(format!("requires unknown module {r}"));
                }
            }
            for c in &d.components {
                match parse_component(c) {
                    (Some(group), default) => {
                        if !cat.group_members(group).iter().any(|x| x.id == default) {
                            return err(format!("{default} is not a member of group {group}"));
                        }
                    }
                    (None, id) => {
                        if cat.get(id).is_none() {
                            return err(format!("unknown SDK component {id}"));
                        }
                    }
                }
            }
            for f in &d.files {
                if !m.files.contains_key(&f.src) {
                    return err(format!("files/{} is missing", f.src));
                }
            }
            for l in &d.launch {
                if l.node.is_some() == l.include.is_some() {
                    return err(format!("launch entry {} needs exactly one of node or include", l.name));
                }
            }
        }
        Ok(())
    }

    /// The given modules plus everything they require, dependencies first.
    pub fn expand(&self, ids: &[String]) -> Result<Vec<&Module>, String> {
        fn visit<'a>(reg: &'a Registry, id: &str, seen: &mut BTreeSet<String>, trail: &mut Vec<String>, out: &mut Vec<&'a Module>) -> Result<(), String> {
            if seen.contains(id) {
                return Ok(());
            }
            if trail.iter().any(|t| t == id) {
                return Err(format!("module dependency cycle through {id}"));
            }
            let m = reg.get(id).ok_or_else(|| format!("unknown module: {id}"))?;
            trail.push(id.to_string());
            for r in &m.def.requires {
                visit(reg, r, seen, trail, out)?;
            }
            trail.pop();
            seen.insert(id.to_string());
            out.push(m);
            Ok(())
        }
        let mut out = Vec::new();
        let mut seen = BTreeSet::new();
        for id in ids {
            visit(self, id, &mut seen, &mut Vec::new(), &mut out)?;
        }
        Ok(out)
    }
}

/// `group:px4=px4-1.16` -> (Some("px4"), "px4-1.16"); `mujoco` -> (None, "mujoco").
pub fn parse_component(spec: &str) -> (Option<&str>, &str) {
    match spec.strip_prefix("group:").and_then(|r| r.split_once('=')) {
        Some((group, default)) => (Some(group), default),
        None => (None, spec),
    }
}
