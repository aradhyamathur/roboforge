use crate::paths::Home;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

/// Marker stored in an env's set once `micromamba create` has succeeded.
pub const ENV_CREATED: &str = "@created";

/// What is installed. Written after every completed install unit, so an
/// interrupted install resumes where it stopped.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct State {
    #[serde(default)]
    pub sdks: BTreeSet<String>,
    #[serde(default)]
    pub envs: BTreeMap<String, BTreeSet<String>>,
}

impl State {
    pub fn load(home: &Home) -> State {
        std::fs::read_to_string(home.state_file())
            .ok()
            .and_then(|t| serde_json::from_str(&t).ok())
            .unwrap_or_default()
    }
    pub fn save(&self, home: &Home) -> std::io::Result<()> {
        std::fs::create_dir_all(&home.root)?;
        let tmp = home.root.join("state.json.tmp");
        std::fs::write(&tmp, serde_json::to_string_pretty(self).unwrap())?;
        std::fs::rename(tmp, home.state_file())
    }
    pub fn env_has(&self, env: &str, member: &str) -> bool {
        self.envs.get(env).map_or(false, |s| s.contains(member))
    }
}
