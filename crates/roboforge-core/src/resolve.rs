use crate::catalog::{Catalog, Component};
use serde::Serialize;
use std::collections::{BTreeMap, BTreeSet};

#[derive(Debug, Clone, Serialize)]
pub struct EnvSpec {
    pub name: String,
    /// Component ids in dependency order.
    pub components: Vec<String>,
    pub python: Option<String>,
    pub ros_distro: Option<String>,
    pub ros_version: Option<u8>,
}

#[derive(Debug, Clone, Serialize)]
pub struct Resolution {
    /// Every component, dependencies first.
    pub components: Vec<String>,
    /// The project's main environment (absent when nothing needs one).
    pub main_env: Option<EnvSpec>,
    /// Dedicated environments, keyed by `own_env`.
    pub own_envs: Vec<EnvSpec>,
    /// Components with a shared source checkout.
    pub sources: Vec<String>,
}

impl Resolution {
    pub fn env_of(&self, id: &str) -> Option<&EnvSpec> {
        self.main_env
            .iter()
            .chain(self.own_envs.iter())
            .find(|e| e.components.iter().any(|c| c == id))
    }
    pub fn own_env(&self, key: &str) -> Option<&EnvSpec> {
        self.own_envs.iter().find(|e| e.name == own_env_name(key))
    }
}

pub fn own_env_name(key: &str) -> String {
    format!("own-{key}")
}

fn fnv(s: &str) -> u64 {
    let mut h: u64 = 0xcbf29ce484222325;
    for b in s.bytes() {
        h ^= b as u64;
        h = h.wrapping_mul(0x100000001b3);
    }
    h
}

/// Environment names are a pure function of their component set: the same
/// selection always maps to the same environment, a different one never
/// touches it.
pub fn stack_name(ids: &[String]) -> String {
    let mut sorted: Vec<&str> = ids.iter().map(|s| s.as_str()).collect();
    sorted.sort_unstable();
    let joined = sorted.join("__");
    if joined.len() <= 64 {
        joined
    } else {
        format!("{}-{:08x}", &joined[..48], fnv(&joined) as u32)
    }
}

fn visit<'a>(
    cat: &'a Catalog,
    id: &str,
    selected: &BTreeSet<String>,
    order: &mut Vec<String>,
    done: &mut BTreeSet<String>,
    stack: &mut Vec<String>,
) -> Result<(), String> {
    if done.contains(id) {
        return Ok(());
    }
    if stack.iter().any(|s| s == id) {
        return Err(format!("dependency cycle through {id}"));
    }
    let c = cat.get(id).ok_or_else(|| format!("unknown component: {id}"))?;
    stack.push(id.to_string());
    for r in &c.requires {
        if let Some(group) = r.strip_prefix("group:") {
            let members = cat.group_members(group);
            match members.iter().find(|m| selected.contains(&m.id)) {
                Some(m) => visit(cat, &m.id.clone(), selected, order, done, stack)?,
                None => {
                    let names: Vec<&str> = members.iter().map(|m| m.id.as_str()).collect();
                    return Err(format!("{} needs one of: {}", c.name, names.join(", ")));
                }
            }
        } else {
            visit(cat, r, selected, order, done, stack)?;
        }
    }
    stack.pop();
    done.insert(id.to_string());
    order.push(id.to_string());
    Ok(())
}

fn build_env(cat: &Catalog, name: String, ids: Vec<String>) -> Result<EnvSpec, String> {
    let comps: Vec<&Component> = ids.iter().filter_map(|i| cat.get(i)).collect();
    let mut groups: BTreeMap<&str, &str> = BTreeMap::new();
    for c in &comps {
        if let Some(g) = c.group.as_deref() {
            if let Some(prev) = groups.insert(g, &c.name) {
                return Err(format!(
                    "{prev} and {} cannot share an environment (both are \"{g}\"). Pick one per project.",
                    c.name
                ));
            }
        }
    }
    let ros = comps.iter().find(|c| c.distro.is_some());
    let mut python: Option<String> = None;
    if ros.is_none() {
        for c in &comps {
            if let Some(p) = &c.python {
                match &python {
                    Some(q) if q != p => {
                        return Err(format!(
                            "conflicting Python pins in one environment: {q} and {p} ({})",
                            c.name
                        ))
                    }
                    _ => python = Some(p.clone()),
                }
            }
        }
    }
    Ok(EnvSpec {
        name,
        components: ids,
        python,
        ros_distro: ros.and_then(|c| c.distro.clone()),
        ros_version: ros.and_then(|c| c.ros_version),
    })
}

pub fn resolve(cat: &Catalog, selected: &[String]) -> Result<Resolution, String> {
    let sel: BTreeSet<String> = selected.iter().cloned().collect();
    let mut order = Vec::new();
    let mut done = BTreeSet::new();
    for id in selected {
        visit(cat, id, &sel, &mut order, &mut done, &mut Vec::new())?;
    }

    let mut main = Vec::new();
    let mut own: BTreeMap<String, Vec<String>> = BTreeMap::new();
    let mut sources = Vec::new();
    for id in &order {
        let c = cat.get(id).unwrap();
        if c.source.is_some() {
            sources.push(id.clone());
        }
        match &c.own_env {
            Some(k) => own.entry(k.clone()).or_default().push(id.clone()),
            None => main.push(id.clone()),
        }
    }

    let needs_env = |ids: &[String]| {
        ids.iter().filter_map(|i| cat.get(i)).any(|c| {
            c.conda.is_some()
                || !c.pip.is_empty()
                || c.source.as_ref().map_or(false, |s| !s.post.is_empty())
        })
    };
    let main_env = if needs_env(&main) {
        Some(build_env(cat, stack_name(&main), main)?)
    } else {
        None
    };
    let mut own_envs = Vec::new();
    for (k, ids) in own {
        own_envs.push(build_env(cat, own_env_name(&k), ids)?);
    }
    Ok(Resolution { components: order, main_env, own_envs, sources })
}
