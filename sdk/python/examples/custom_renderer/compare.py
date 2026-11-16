"""Render the same moment with MuJoCo's renderer and with the splat renderer,
save both side by side, and check that they agree on where the robot is.
(MuJoCo shades the bottom of the arm dark, so compare the sides and the top.)

    python3 compare.py            # writes frames/compare.png
"""
import pathlib
import sys
import time

import numpy as np

import roboforge as rf
from roboforge.data import write_png

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from splat import SplatRenderer  # noqa: E402


def robot_mask(rgb):
    """Pixels of the arm's orange."""
    r, g, b = (rgb[..., i].astype(int) for i in range(3))
    return (r > g + 50) & (r > b + 90)


def frame(renderer):
    app = rf.App(robot="arm", renderer=renderer, controllers=[rf.JointPD(rate=500, target=[0.6, 0.5, -0.9])], realtime=False)
    app.run(duration=1.5)
    t0 = time.perf_counter()
    img = app.renderer.render("overview")
    ms = 1000 * (time.perf_counter() - t0)
    app.close()
    return img, ms


def main():
    splat, ms_s = frame(SplatRenderer(width=320, height=240))
    try:
        mj, ms_m = frame(rf.MujocoRenderer(width=320, height=240))
    except Exception as e:                       # no OpenGL on this machine: the splat renderer still works
        print(f"MuJoCo's renderer is not available here ({type(e).__name__}); splat frame only")
        mj, ms_m = None, 0.0
    out = pathlib.Path("frames")
    out.mkdir(exist_ok=True)
    side = splat.data if mj is None else np.concatenate([mj.data, splat.data], axis=1)
    write_png(out / "compare.png", side)
    print(f"splat renderer: {ms_s:5.1f} ms per 320x240 frame, pure numpy")
    if mj is not None:
        print(f"MuJoCo renderer: {ms_m:5.1f} ms per frame, OpenGL")
        print("same camera pose in both:", np.allclose(mj.pose, splat.pose))
        for name, img in (("MuJoCo", mj), ("splats", splat)):
            y, x = np.nonzero(robot_mask(img.data))
            print(f"arm in the {name:<6} frame: x {x.min()}-{x.max()}, top at y {y.min()}")
    print(f"-> {out / 'compare.png'} (left: MuJoCo, right: splats)")


if __name__ == "__main__":
    main()
