//! Background processes with captured, pollable output.

use std::collections::HashMap;
use std::io::Read;
use std::os::unix::process::CommandExt;
use std::path::Path;
use std::process::{Command, Stdio};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

const LOG_CAP: usize = 4 << 20;

#[derive(Clone, Debug, PartialEq)]
pub enum Status {
    Running,
    Done(i32),
}

pub(crate) struct LogBuf {
    /// Absolute offset of `data[0]`; grows when old output is dropped.
    pub base: u64,
    pub data: Vec<u8>,
}

impl LogBuf {
    pub fn new() -> Self {
        LogBuf { base: 0, data: Vec::new() }
    }
    pub fn append(&mut self, bytes: &[u8]) {
        self.data.extend_from_slice(bytes);
        if self.data.len() > LOG_CAP {
            let cut = self.data.len() - LOG_CAP / 2;
            self.data.drain(..cut);
            self.base += cut as u64;
        }
    }
    /// Output from `offset` on, cut at a UTF-8 boundary, plus the next offset.
    pub fn read_from(&self, offset: u64) -> (String, u64) {
        let start = offset.saturating_sub(self.base).min(self.data.len() as u64) as usize;
        let mut slice = &self.data[start..];
        if let Err(e) = std::str::from_utf8(slice) {
            if e.error_len().is_none() {
                slice = &slice[..e.valid_up_to()];
            }
        }
        let next = self.base + (start + slice.len()) as u64;
        (String::from_utf8_lossy(slice).into_owned(), next)
    }
}

pub struct Job {
    pub id: u64,
    pub title: String,
    log: Mutex<LogBuf>,
    status: Mutex<Status>,
    pgid: Mutex<Option<u32>>,
    cancelled: AtomicBool,
}

impl Job {
    pub fn append(&self, bytes: &[u8]) {
        let mut l = self.log.lock().unwrap();
        l.data.extend_from_slice(bytes);
        if l.data.len() > LOG_CAP {
            let cut = l.data.len() - LOG_CAP / 2;
            l.data.drain(..cut);
            l.base += cut as u64;
        }
    }
    pub fn say(&self, line: &str) {
        self.append(line.as_bytes());
        self.append(b"\n");
    }
    /// Output from `offset` on, cut at a UTF-8 boundary, plus the next offset.
    pub fn read_from(&self, offset: u64) -> (String, u64) {
        let l = self.log.lock().unwrap();
        let start = offset.saturating_sub(l.base).min(l.data.len() as u64) as usize;
        let mut slice = &l.data[start..];
        if let Err(e) = std::str::from_utf8(slice) {
            if e.error_len().is_none() {
                slice = &slice[..e.valid_up_to()];
            }
        }
        let next = l.base + (start + slice.len()) as u64;
        (String::from_utf8_lossy(slice).into_owned(), next)
    }
    pub fn status(&self) -> Status {
        self.status.lock().unwrap().clone()
    }
    pub fn finish(&self, code: i32) {
        *self.status.lock().unwrap() = Status::Done(code);
    }
    pub fn cancelled(&self) -> bool {
        self.cancelled.load(Ordering::SeqCst)
    }

    /// Run a bash script to completion, streaming stdout+stderr into the log.
    pub fn run_shell(&self, script: &str, cwd: &Path) -> i32 {
        if self.cancelled() {
            return 130;
        }
        let mut cmd = Command::new("bash");
        cmd.arg("-c")
            .arg(format!("exec 2>&1\n{script}"))
            .current_dir(cwd)
            .env("PYTHONUNBUFFERED", "1")
            .stdin(Stdio::null())
            .stdout(Stdio::piped())
            .process_group(0);
        let mut child = match cmd.spawn() {
            Ok(c) => c,
            Err(e) => {
                self.say(&format!("roboforge: cannot start bash in {}: {e}", cwd.display()));
                return 127;
            }
        };
        *self.pgid.lock().unwrap() = Some(child.id());
        let mut out = child.stdout.take().unwrap();
        let mut buf = [0u8; 8192];
        loop {
            match out.read(&mut buf) {
                Ok(0) | Err(_) => break,
                Ok(n) => self.append(&buf[..n]),
            }
        }
        let code = child.wait().ok().and_then(|s| s.code()).unwrap_or(130);
        *self.pgid.lock().unwrap() = None;
        code
    }

    /// Stop the whole process group: TERM first, KILL two seconds later.
    pub fn kill(self: &Arc<Self>) {
        self.cancelled.store(true, Ordering::SeqCst);
        let Some(pgid) = *self.pgid.lock().unwrap() else { return };
        let _ = Command::new("kill").args(["-TERM", "--", &format!("-{pgid}")]).status();
        let me = Arc::clone(self);
        std::thread::spawn(move || {
            std::thread::sleep(std::time::Duration::from_secs(2));
            if *me.pgid.lock().unwrap() == Some(pgid) {
                let _ = Command::new("kill").args(["-KILL", "--", &format!("-{pgid}")]).status();
            }
        });
    }
}

#[derive(Default)]
pub struct Jobs {
    map: Mutex<HashMap<u64, Arc<Job>>>,
    next: AtomicU64,
}

impl Jobs {
    /// Start `work` on a new thread; its return value is the exit code.
    pub fn start<F>(&self, title: &str, work: F) -> Arc<Job>
    where
        F: FnOnce(&Job) -> i32 + Send + 'static,
    {
        let id = self.next.fetch_add(1, Ordering::SeqCst) + 1;
        let job = Arc::new(Job {
            id,
            title: title.to_string(),
            log: Mutex::new(LogBuf { base: 0, data: Vec::new() }),
            status: Mutex::new(Status::Running),
            pgid: Mutex::new(None),
            cancelled: AtomicBool::new(false),
        });
        self.map.lock().unwrap().insert(id, Arc::clone(&job));
        let j = Arc::clone(&job);
        std::thread::spawn(move || {
            let code = work(&j);
            j.finish(code);
        });
        job
    }
    pub fn get(&self, id: u64) -> Option<Arc<Job>> {
        self.map.lock().unwrap().get(&id).cloned()
    }
    pub fn forget(&self, id: u64) {
        self.map.lock().unwrap().remove(&id);
    }
    /// Stop every running job; called when the IDE itself is shutting down
    /// so simulators and SITL instances do not outlive it.
    pub fn shutdown(&self) {
        let jobs: Vec<Arc<Job>> = self.map.lock().unwrap().values().cloned().collect();
        let groups: Vec<u32> = jobs
            .iter()
            .filter_map(|j| {
                j.cancelled.store(true, Ordering::SeqCst);
                *j.pgid.lock().unwrap()
            })
            .collect();
        if groups.is_empty() {
            return;
        }
        for g in &groups {
            let _ = Command::new("kill").args(["-TERM", "--", &format!("-{g}")]).status();
        }
        std::thread::sleep(std::time::Duration::from_millis(1500));
        for j in &jobs {
            if let Some(g) = *j.pgid.lock().unwrap() {
                let _ = Command::new("kill").args(["-KILL", "--", &format!("-{g}")]).status();
            }
        }
    }
}
