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

    // The Python SDK ships inside the binary too, so a fresh machine needs
    // nothing but the IDE: it is unpacked into the RoboForge home on first use.
    let sdk = Path::new(&std::env::var("CARGO_MANIFEST_DIR").unwrap()).join("../../sdk/python");
    println!("cargo:rerun-if-changed={}", sdk.join("roboforge").display());
    println!("cargo:rerun-if-changed={}", sdk.join("pyproject.toml").display());
    let mut files = Vec::new();
    walk(&sdk.join("roboforge"), &sdk, &mut files);
    // The examples are the tutorials in docs/sdk.html; they ship too.
    println!("cargo:rerun-if-changed={}", sdk.join("examples").display());
    walk(&sdk.join("examples"), &sdk, &mut files);
    files.retain(|(rel, _)| !rel.contains("__pycache__") && [".py", ".xml", ".yaml", ".urdf"].iter().any(|e| rel.ends_with(e)));
    files.push(("pyproject.toml".into(), sdk.join("pyproject.toml")));
    let mut code = String::from("pub static SDK_FILES: &[(&str, &str)] = &[\n");
    for (rel, path) in files {
        writeln!(code, "    ({rel:?}, include_str!({:?})),", path.canonicalize().unwrap()).unwrap();
    }
    code.push_str("];\n");
    std::fs::write(Path::new(&std::env::var("OUT_DIR").unwrap()).join("sdk_files.rs"), code).unwrap();
}
