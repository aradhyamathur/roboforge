"""The documentation builds without warnings, and the code it shows runs."""
import pathlib
import re
import socket
import subprocess
import sys
import textwrap

import pytest

pytest.importorskip("mujoco")
import roboforge as rf  # noqa: E402

DOCS = pathlib.Path(__file__).resolve().parents[1] / "docs"


def python_blocks(page):
    """The `.. code-block:: python` blocks of a page, dedented."""
    text = (DOCS / page).read_text()
    blocks = re.findall(r"\.\. code-block:: python\n\n((?:(?:   .*)?\n)+)", text)
    return [textwrap.dedent(b) for b in blocks]


def test_docs_build_without_warnings(tmp_path):
    pytest.importorskip("sphinx")
    pytest.importorskip("sphinx_rtd_theme")
    out = subprocess.run([sys.executable, "-m", "sphinx", "-W", "--keep-going", "-q", "-b", "html", str(DOCS), str(tmp_path)],
                         capture_output=True, text=True, timeout=600)
    assert out.returncode == 0, out.stderr[-3000:]
    for page in ("index.html", "tutorials/debug_broken_app.html", "debugging/troubleshooting.html", "reference/api.html"):
        assert (tmp_path / page).is_file()


def test_every_example_file_shown_in_the_docs_exists():
    for rst in DOCS.rglob("*.rst"):
        for inc in re.findall(r"\.\. literalinclude:: (\S+)", rst.read_text()):
            assert (rst.parent / inc).resolve().is_file(), f"{rst.name} includes {inc}, which does not exist"


def test_extending_guide_code_runs(capsys):
    ns = {}
    for block in python_blocks("guides/extending.rst"):
        exec(block, ns)                                  # the classes the guide defines, and its bus example
    sink = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sink.bind(("127.0.0.1", 0))
    sink.settimeout(5)
    app = rf.App(robot="diffdrive",
                 controllers=[rf.controller(lambda o, t: rf.velocity_command((0.3, 0, 0)), rate=10), rf.DiffDriveController()],
                 links=[ns["UdpJson"](port=sink.getsockname()[1])], callbacks=[ns["Lap"]()], realtime=False)
    app.bus.publish("state/pose", rf.Pose(position=[-0.05, 0, 0]))   # the base starts just behind x = 0 for the lap counter
    app.run(duration=2.0)
    app.close()
    data, _ = sink.recvfrom(4096)
    assert b'"position"' in data
    assert issubclass(ns["MySim"], rf.Physics)


def test_testing_page_example_passes():
    ns = {}
    for block in python_blocks("testing.rst"):
        exec(block, ns)
    ns["test_arm_reaches_its_target"]()


def test_quickstart_app_runs_as_written(tmp_path):
    text = (DOCS / "quickstart.rst").read_text()
    yamls = [textwrap.dedent(b) for b in re.findall(r"\.\. code-block:: yaml\n\n((?:(?:   .*)?\n)+)", text)]
    code = python_blocks("quickstart.rst")[0]
    (tmp_path / "controller.py").write_text(code)
    for y in yamls[:2]:                                  # the first app, then the one using controller.py
        (tmp_path / "app.yaml").write_text(y)
        out = subprocess.run([sys.executable, "-m", "roboforge", "run", "app.yaml", "--headless", "--duration", "2", "--tap", "--check", "--no-realtime"],
                             capture_output=True, text=True, cwd=tmp_path, timeout=120)
        assert out.returncode == 0, out.stderr[-1500:]
        assert "state/joint" in out.stdout and "roboforge check" not in out.stdout, out.stdout[-1500:]
    assert "cmd/joint" in out.stdout and "Wave" in out.stdout
