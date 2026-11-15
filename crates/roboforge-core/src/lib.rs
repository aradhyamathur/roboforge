//! RoboForge core: SDK catalog, environment resolver, install planner and
//! project generator. No I/O beyond the filesystem; process execution lives
//! in the `roboforge` binary.

pub mod catalog;
pub mod modules;
pub mod paths;
pub mod plan;
pub mod project;
pub mod resolve;
pub mod state;
pub mod templates;
pub mod tools;

/// Quote a string for safe use as a single bash word.
pub fn sh_quote(s: &str) -> String {
    if !s.is_empty()
        && s.chars()
            .all(|c| c.is_ascii_alphanumeric() || "-_./=:+@,".contains(c))
    {
        return s.to_string();
    }
    format!("'{}'", s.replace('\'', "'\\''"))
}
