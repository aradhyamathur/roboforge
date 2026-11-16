"""Change what the camera sees: render passes.

A pass is a function `(image, renderer) -> image` run on every frame before
it is published, so everything downstream (ROS, the recorder, a policy) gets
the changed picture. Here: a depth overlay, and sensor noise. Replacing the
picture entirely works the same way, which is how another renderer or a
learned scene model plugs in.

    python3 render_pass.py          # writes frames/plain.png, frames/depth_tint.png, frames/noisy.png
"""
import pathlib

import numpy as np

import roboforge as rf
from roboforge.data import write_png


def depth_tint(img, renderer):
    """Tint near things red and far things blue, using the depth buffer of the same camera."""
    depth = renderer.render(img.camera, depth=True).data                  # metres
    near = np.clip(1.0 - depth / 6.0, 0.0, 1.0)[..., None]
    tint = near * np.array([255, 60, 40]) + (1 - near) * np.array([40, 80, 255])
    out = (0.55 * img.data + 0.45 * tint).astype(np.uint8)
    return rf.Image(data=out, camera=img.camera, fovy=img.fovy, pose=img.pose, t=img.t)


def sensor_noise(sigma=12.0, seed=0):
    rng = np.random.default_rng(seed)

    def noisy(img, renderer):
        img.data = np.clip(img.data + rng.normal(0, sigma, img.data.shape), 0, 255).astype(np.uint8)    # in place: return nothing
    return noisy


def frame(*passes):
    r = rf.MujocoRenderer(width=320, height=240, fps=10)
    for p in passes:
        r.add_pass(p)
    app = rf.App(robot="quadrotor", renderer=r, controllers=[rf.MultirotorController(rate=250, hover=[0, 0, 1.5])], realtime=False)
    app.run(duration=2.0)
    img = app.bus.latest("sensor/camera/front")
    app.close()
    return img


def main():
    out = pathlib.Path("frames")
    out.mkdir(exist_ok=True)
    plain = frame()
    for name, img in (("plain", plain), ("depth_tint", frame(depth_tint)), ("noisy", frame(sensor_noise()))):
        write_png(out / f"{name}.png", img.data)
        diff = float(np.abs(img.data.astype(int) - plain.data.astype(int)).mean())
        print(f"{name:<11} {img.data.shape[1]}x{img.data.shape[0]}  differs from plain by {diff:5.1f} grey levels  -> {out / (name + '.png')}")


if __name__ == "__main__":
    main()
