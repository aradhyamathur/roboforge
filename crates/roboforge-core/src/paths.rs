use std::path::PathBuf;

/// Everything RoboForge installs lives under one directory (default
/// `~/.roboforge`), so removing it removes every SDK.
#[derive(Debug, Clone)]
pub struct Home {
    pub root: PathBuf,
}

impl Home {
    pub fn discover() -> Home {
        if let Some(p) = std::env::var_os("ROBOFORGE_HOME") {
            return Home { root: PathBuf::from(p) };
        }
        let base = std::env::var_os("HOME").map(PathBuf::from).unwrap_or_else(|| PathBuf::from("."));
        Home { root: base.join(".roboforge") }
    }
    pub fn micromamba(&self) -> PathBuf {
        self.root.join("tools").join("micromamba")
    }
    pub fn mamba_root(&self) -> PathBuf {
        self.root.join("mamba")
    }
    pub fn sdk_dir(&self, id: &str) -> PathBuf {
        self.root.join("sdks").join(id)
    }
    pub fn env_prefix(&self, name: &str) -> PathBuf {
        self.root.join("envs").join(name)
    }
    pub fn modules_dir(&self) -> PathBuf {
        self.root.join("modules")
    }
    pub fn state_file(&self) -> PathBuf {
        self.root.join("state.json")
    }
    pub fn catalog_override(&self) -> PathBuf {
        self.root.join("catalog.json")
    }
    pub fn recent_file(&self) -> PathBuf {
        self.root.join("recent.json")
    }
}
