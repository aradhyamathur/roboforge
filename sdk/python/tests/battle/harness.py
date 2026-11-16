"""Battle tests: each scenario runs in its own process with a time limit, so a
hang, a crash in native code or a leak in one cannot hide behind another.

    python tests/battle/harness.py              # all scenarios
    python tests/battle/harness.py config       # those whose name contains "config"
    python tests/battle/harness.py --one NAME   # one scenario, in this process (for debugging)

A scenario is a function in a `battle_*.py` file next to this one, decorated
with `@scenario(timeout=...)`. It passes by returning (optionally a short
string of measurements) and fails by raising.
"""
import importlib
import os
import pathlib
import subprocess
import sys
import time
import traceback

HERE = pathlib.Path(__file__).resolve().parent
SCENARIOS = {}
os.environ.setdefault("ROS_DOMAIN_ID", "86")
os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")


def scenario(timeout=60, needs=()):
    def deco(fn):
        SCENARIOS[fn.__name__] = (fn, timeout, needs)
        return fn
    return deco


UNHELPFUL = (TypeError, AttributeError, KeyError, IndexError, AssertionError, ZeroDivisionError, UnboundLocalError, RecursionError)


def clear_error(fn, *hints, kinds=(ValueError, FileNotFoundError, RuntimeError, ImportError, ConnectionError, TimeoutError, OSError)):
    """`fn()` must fail with an error a user can act on: a sensible exception type
    whose message mentions every hint. Returns the message."""
    try:
        fn()
    except UNHELPFUL as e:
        raise AssertionError(f"unhelpful {type(e).__name__}: {e}") from e
    except kinds as e:
        msg = str(e)
        missing = [h for h in hints if h.lower() not in msg.lower()]
        if missing:
            raise AssertionError(f"error does not mention {missing}: {type(e).__name__}: {msg}") from e
        return f"{type(e).__name__}: {msg}"
    raise AssertionError("no error was raised")


def rss_mb():
    with open("/proc/self/status") as f:
        for line in f:
            if line.startswith("VmRSS"):
                return int(line.split()[1]) / 1024
    return 0.0


def threads():
    import threading
    return threading.active_count()


def fds():
    return len(os.listdir("/proc/self/fd"))


def load():
    sys.path.insert(0, str(HERE))
    for f in sorted(HERE.glob("battle_*.py")):
        importlib.import_module(f.stem)


def main():
    load()
    args = sys.argv[1:]
    if args[:1] == ["--one"]:
        fn, _, _ = SCENARIOS[args[1]]
        try:
            out = fn()
        except Exception:
            traceback.print_exc()
            sys.exit(1)
        print(f"RESULT {out or ''}")
        return
    names = [n for n in SCENARIOS if not args or any(a in n for a in args)]
    failed = []
    for n in names:
        _, timeout, needs = SCENARIOS[n]
        missing = [m for m in needs if importlib.util.find_spec(m) is None]
        if missing:
            print(f"skip  {n}  (no {', '.join(missing)})", flush=True)
            continue
        t0 = time.time()
        try:
            p = subprocess.run([sys.executable, __file__, "--one", n], capture_output=True, text=True, timeout=timeout)
            code, out = p.returncode, p.stdout + p.stderr
        except subprocess.TimeoutExpired as e:
            code, out = "HANG", ((e.stdout or b"").decode() if isinstance(e.stdout, bytes) else (e.stdout or "")) + f"\n(no result within {timeout}s)"
        dt = time.time() - t0
        result = [l[7:] for l in out.splitlines() if l.startswith("RESULT ")]
        if code == 0:
            print(f"ok    {n}  {dt:.1f}s  {result[-1] if result else ''}", flush=True)
        else:
            failed.append(n)
            tail = "\n        ".join(out.strip().splitlines()[-6:])
            print(f"FAIL  {n}  {dt:.1f}s  exit {code}\n        {tail}", flush=True)
    print(f"\n{len(names) - len(failed)} of {len(names)} passed" + (f"; failed: {', '.join(failed)}" if failed else ""))
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    # Scenario files import this module by name; make that the same module object.
    sys.modules.setdefault("harness", sys.modules["__main__"])
    main()
