//! Embeds every folder under `modules/` into the library, so adding a
//! built-in module is: create a folder, write module.json, drop in files.

use std::fmt::Write as _;
use std::path::{Path, PathBuf};

fn walk(dir: &Path, base: &Path, out: &mut Vec<(String, PathBuf)>) {
    let mut entries: Vec<PathBuf> = std::fs::read_dir(dir).into_iter().flatten().flatten().map(|e| e.path()).collect();
    entries.sort();
    for p in entries {
        if p.is_dir() {
            walk(&p, base, out);
        } else {
            let rel = p.strip_prefix(base).unwrap().to_string_lossy().replace('\\', "/");
            out.push((rel, p));
        }
    }
}

fn main() {
    println!("cargo:rerun-if-changed=modules");
    let root = Path::new(&std::env::var("CARGO_MANIFEST_DIR").unwrap()).join("modules");
    let mut dirs: Vec<PathBuf> = std::fs::read_dir(&root).unwrap().flatten().map(|e| e.path()).filter(|p| p.join("module.json").exists()).collect();
    dirs.sort();
    let mut code = String::from("pub static BUILTIN: &[(&str, &[(&str, &str)])] = &[\n");
    for d in dirs {
        let mut files = Vec::new();
        walk(&d.join("files"), &d.join("files"), &mut files);
        write!(code, "    (include_str!({:?}), &[", d.join("module.json")).unwrap();
        for (rel, path) in files {
            write!(code, "({rel:?}, include_str!({path:?})), ").unwrap();
        }
        code.push_str("]),\n");
    }
    code.push_str("];\n");
    let out = Path::new(&std::env::var("OUT_DIR").unwrap()).join("builtin_modules.rs");
    std::fs::write(out, code).unwrap();
}
