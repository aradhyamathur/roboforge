use crate::paths::Home;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

pub const EMBEDDED: &str = include_str!("catalog.json");

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Conda {
    #[serde(default)]
    pub channels: Vec<String>,
    /// Always installed.
    #[serde(default)]
    pub specs: Vec<String>,
    /// Only when the environment holds a ROS 2 distro. `{ros_distro}` expands.
    #[serde(default)]
    pub with_ros2: Vec<String>,
    #[serde(default)]
    pub with_ros1: Vec<String>,
    /// Only when the environment holds no ROS at all.
    #[serde(default)]
    pub without_ros: Vec<String>,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Source {
    pub git: String,
    #[serde(default)]
    pub rev: Option<String>,
    #[serde(default)]
    pub recursive: bool,
    /// Build commands, run once inside the checkout with the env active.
    #[serde(default)]
    pub post: Vec<String>,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Component {
    pub id: String,
    pub name: String,
    pub category: String,
    pub version: String,
    #[serde(default)]
    pub summary: String,
    /// At most one component per group may share an environment.
    #[serde(default)]
    pub group: Option<String>,
    /// ROS distro name, set on ROS components only.
    #[serde(default)]
    pub distro: Option<String>,
    #[serde(default)]
    pub ros_version: Option<u8>,
    /// Components naming the same `own_env` share a dedicated environment
    /// instead of joining the project's main one.
    #[serde(default)]
    pub own_env: Option<String>,
    /// Python pin. Ignored when a ROS distro is in the same environment,
    /// because RoboStack decides the Python version there.
    #[serde(default)]
    pub python: Option<String>,
    #[serde(default)]
    pub conda: Option<Conda>,
    /// Each entry is one `pip install` argument list.
    #[serde(default)]
    pub pip: Vec<Vec<String>>,
    #[serde(default)]
    pub source: Option<Source>,
    /// Component ids, or `group:<name>` for "any one member of this group".
    #[serde(default)]
    pub requires: Vec<String>,
    /// Exported on activation. `{sdk}` expands to this component's checkout.
    #[serde(default)]
    pub env: BTreeMap<String, String>,
    #[serde(default)]
    pub notes: String,
    #[serde(default)]
    pub gpu: bool,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Catalog {
    #[serde(default)]
    pub schema: u32,
    pub components: Vec<Component>,
}

impl Catalog {
    pub fn embedded() -> Catalog {
        serde_json::from_str(EMBEDDED).expect("embedded catalog is valid JSON")
    }

    /// `<home>/catalog.json` overrides the built-in catalog when present, so
    /// versions and recipes can be changed without rebuilding the IDE.
    pub fn load(home: &Home) -> Result<Catalog, String> {
        let p = home.catalog_override();
        let cat = if p.exists() {
            let text = std::fs::read_to_string(&p).map_err(|e| format!("{}: {e}", p.display()))?;
            serde_json::from_str(&text).map_err(|e| format!("{}: {e}", p.display()))?
        } else {
            Catalog::embedded()
        };
        cat.validate()?;
        Ok(cat)
    }

    pub fn get(&self, id: &str) -> Option<&Component> {
        self.components.iter().find(|c| c.id == id)
    }

    pub fn group_members(&self, group: &str) -> Vec<&Component> {
        self.components
            .iter()
            .filter(|c| c.group.as_deref() == Some(group))
            .collect()
    }

    pub fn validate(&self) -> Result<(), String> {
        let mut seen = BTreeSet::new();
        for c in &self.components {
            if c.id.is_empty()
                || !c
                    .id
                    .chars()
                    .all(|ch| ch.is_ascii_lowercase() || ch.is_ascii_digit() || "-._".contains(ch))
            {
                return Err(format!("catalog: invalid component id {:?}", c.id));
            }
            if !seen.insert(c.id.clone()) {
                return Err(format!("catalog: duplicate component id {}", c.id));
            }
        }
        for c in &self.components {
            for r in &c.requires {
                if let Some(g) = r.strip_prefix("group:") {
                    if self.group_members(g).is_empty() {
                        return Err(format!("catalog: {} requires empty group {g}", c.id));
                    }
                } else if !seen.contains(r) {
                    return Err(format!("catalog: {} requires unknown component {r}", c.id));
                }
            }
        }
        Ok(())
    }
}
