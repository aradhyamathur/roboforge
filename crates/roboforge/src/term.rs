//! Interactive terminals: a bash on a pseudo-terminal, polled by the UI.

use crate::jobs::LogBuf;
use std::collections::HashMap;
use std::io::{Read, Write};
use std::os::fd::{AsRawFd, FromRawFd, OwnedFd};
use std::os::unix::process::CommandExt;
use std::path::Path;
use std::process::{Command, Stdio};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

pub struct Term {
    pub id: u64,
    log: Mutex<LogBuf>,
    master: Mutex<std::fs::File>,
    pid: u32,
    alive: AtomicBool,
    code: Mutex<Option<i32>>,
}

impl Term {
    pub fn read_from(&self, offset: u64) -> (String, u64) {
        self.log.lock().unwrap().read_from(offset)
    }
    pub fn status(&self) -> (bool, Option<i32>) {
        (self.alive.load(Ordering::SeqCst), *self.code.lock().unwrap())
    }
    pub fn write(&self, data: &[u8]) -> std::io::Result<()> {
        self.master.lock().unwrap().write_all(data)
    }
    pub fn resize(&self, cols: u16, rows: u16) {
        let ws = libc::winsize { ws_row: rows, ws_col: cols, ws_xpixel: 0, ws_ypixel: 0 };
        let fd = self.master.lock().unwrap().as_raw_fd();
        unsafe { libc::ioctl(fd, libc::TIOCSWINSZ, &ws) };
    }
    /// HUP the session like a closing terminal window, KILL if it lingers.
    pub fn kill(&self) {
        let pgid = self.pid as i32;
        unsafe { libc::kill(-pgid, libc::SIGHUP) };
        let pid = self.pid;
        std::thread::spawn(move || {
            std::thread::sleep(std::time::Duration::from_secs(2));
            unsafe { libc::kill(-(pid as i32), libc::SIGKILL) };
        });
    }
}

#[derive(Default)]
pub struct Terms {
    map: Mutex<HashMap<u64, Arc<Term>>>,
    next: AtomicU64,
}

impl Terms {
    /// Start an interactive bash in `cwd`. `rc` is sourced after ~/.bashrc.
    pub fn open(&self, cwd: &Path, rc: &Path, cols: u16, rows: u16) -> Result<Arc<Term>, String> {
        let (mut master_fd, mut slave_fd) = (0, 0);
        let mut ws = libc::winsize { ws_row: rows, ws_col: cols, ws_xpixel: 0, ws_ypixel: 0 };
        if unsafe { libc::openpty(&mut master_fd, &mut slave_fd, std::ptr::null_mut(), std::ptr::null_mut(), &mut ws) } != 0 {
            return Err(format!("openpty failed: {}", std::io::Error::last_os_error()));
        }
        let master = unsafe { OwnedFd::from_raw_fd(master_fd) };
        let slave = unsafe { OwnedFd::from_raw_fd(slave_fd) };
        let dup = |fd: &OwnedFd| fd.try_clone().map_err(|e| e.to_string());
        let mut cmd = Command::new("bash");
        cmd.args(["--rcfile"]).arg(rc).arg("-i")
            .current_dir(cwd)
            .env("TERM", "xterm-256color")
            .env("COLORTERM", "truecolor")
            .stdin(Stdio::from(dup(&slave)?))
            .stdout(Stdio::from(dup(&slave)?))
            .stderr(Stdio::from(slave));
        unsafe {
            cmd.pre_exec(|| {
                libc::setsid();
                libc::ioctl(0, libc::TIOCSCTTY as _, 0);
                Ok(())
            });
        }
        let child = cmd.spawn().map_err(|e| format!("cannot start bash: {e}"))?;
        drop(cmd); // closes the parent's copies of the slave
        let pid = child.id();
        let master_file = std::fs::File::from(master);
        let mut reader = master_file.try_clone().map_err(|e| e.to_string())?;
        let id = self.next.fetch_add(1, Ordering::SeqCst) + 1;
        let term = Arc::new(Term {
            id,
            log: Mutex::new(LogBuf::new()),
            master: Mutex::new(master_file),
            pid,
            alive: AtomicBool::new(true),
            code: Mutex::new(None),
        });
        self.map.lock().unwrap().insert(id, Arc::clone(&term));
        let t = Arc::clone(&term);
        std::thread::spawn(move || {
            let mut child = child;
            let mut buf = [0u8; 8192];
            loop {
                match reader.read(&mut buf) {
                    Ok(0) | Err(_) => break, // EIO once the shell and its children are gone
                    Ok(n) => t.log.lock().unwrap().append(&buf[..n]),
                }
            }
            let code = child.wait().ok().and_then(|s| s.code()).unwrap_or(130);
            *t.code.lock().unwrap() = Some(code);
            t.alive.store(false, Ordering::SeqCst);
        });
        Ok(term)
    }
    pub fn get(&self, id: u64) -> Option<Arc<Term>> {
        self.map.lock().unwrap().get(&id).cloned()
    }
    pub fn close(&self, id: u64) {
        if let Some(t) = self.map.lock().unwrap().remove(&id) {
            if t.status().0 {
                t.kill();
            }
        }
    }
    pub fn shutdown(&self) {
        for t in self.map.lock().unwrap().values() {
            if t.status().0 {
                unsafe { libc::kill(-(t.pid as i32), libc::SIGHUP) };
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::{Duration, Instant};

    #[test]
    fn interactive_terminal_runs_and_exits() {
        let terms = Terms::default();
        let term = terms.open(&std::env::current_dir().unwrap(), Path::new("/dev/null"), 80, 24).unwrap();
        term.resize(100, 30);
        term.write(b"printf 'roboforge-pty-ok\\n'; exit\n").unwrap();
        let deadline = Instant::now() + Duration::from_secs(5);
        while term.status().0 && Instant::now() < deadline {
            std::thread::sleep(Duration::from_millis(20));
        }
        let status = term.status();
        let (output, _) = term.read_from(0);
        terms.close(term.id);
        assert_eq!(status, (false, Some(0)), "{output}");
        assert!(output.lines().any(|line| line.trim() == "roboforge-pty-ok"), "{output}");
    }
}
