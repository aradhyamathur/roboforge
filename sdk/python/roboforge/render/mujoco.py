"""MuJoCo's own renderer as a camera source."""
from __future__ import annotations

import threading

import numpy as np

from .. import msgs
from ..component import Renderer


class MujocoRenderer(Renderer):
    """Renders the robot's cameras and publishes `sensor/camera/<name>`.

    `cameras` defaults to the robot's camera list. `depth=True` also
    publishes `sensor/depth/<name>` in metres. Passes added with `add_pass`
    run on every colour frame before it is published.
    """
    name = "mujoco"

    def __init__(self, cameras=None, width: int = 320, height: int = 240, fps: float = 10.0, depth: bool = False):
        super().__init__()
        self.cameras, self.width, self.height, self.fps, self.depth = cameras, width, height, fps, depth
        self._r = None
        self._thread = None
        self._next = 0.0

    def setup(self, app):
        import mujoco
        physics = app.physics
        if getattr(physics, "model", None) is None:
            raise ValueError("MujocoRenderer needs MujocoPhysics (it renders that model). "
                             "Use another renderer with other physics backends.")
        self.bus, self.model, self.data = app.bus, physics.model, physics.data
        if self.cameras is None:
            self.cameras = list(app.robot.cameras) if app.robot else []
        for c in self.cameras:
            if mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, c) < 0:
                raise ValueError(f"the model has no camera named {c!r}")
        self._mj = mujoco
        self.outputs = [msgs.CAMERA.format(name=c) for c in self.cameras] + ([msgs.DEPTH.format(name=c) for c in self.cameras] if self.depth else [])
        self._next = 0.0

    def _context(self):
        """The MuJoCo renderer, created in the thread that renders: an OpenGL
        context belongs to one thread, and the app may be run from any."""
        me = threading.get_ident()
        if self._r is None or self._thread != me:
            if self._r is not None:
                try:
                    self._r.close()
                except Exception:
                    pass
            self._r = self._mj.Renderer(self.model, self.height, self.width)
            self._thread = me
        return self._r

    def _pose(self, camera: str) -> np.ndarray:
        cid = self._mj.mj_name2id(self.model, self._mj.mjtObj.mjOBJ_CAMERA, camera)
        T = np.eye(4)
        T[:3, :3] = self.data.cam_xmat[cid].reshape(3, 3)     # columns: camera x, y, z axes in the world; looks along -z
        T[:3, 3] = self.data.cam_xpos[cid]
        return T

    def render(self, camera: str, depth: bool = False) -> msgs.Image:
        cid = self._mj.mj_name2id(self.model, self._mj.mjtObj.mjOBJ_CAMERA, camera)
        r = self._context()
        r.update_scene(self.data, camera=camera)
        if depth:
            r.enable_depth_rendering()
            data = r.render().copy()
            r.disable_depth_rendering()
            return msgs.Image(data=data, camera=camera, encoding="depth32f", fovy=float(self.model.cam_fovy[cid]), pose=self._pose(camera), t=float(self.data.time))
        img = msgs.Image(data=r.render().copy(), camera=camera, fovy=float(self.model.cam_fovy[cid]), pose=self._pose(camera), t=float(self.data.time))
        for p in self.passes:
            out = p(img, self)
            if out is not None:
                img = out
        return img

    def step(self, t: float):
        if t + 1e-9 < self._next:
            return
        self._next = max(self._next + 1.0 / self.fps, t)
        for c in self.cameras:
            self.bus.publish(msgs.CAMERA.format(name=c), self.render(c), "mujoco-renderer")
            if self.depth:
                self.bus.publish(msgs.DEPTH.format(name=c), self.render(c, depth=True), "mujoco-renderer")

    def close(self):
        if self._r is not None:
            try:
                self._r.close()
            except Exception:
                pass
            self._r = None

    def describe(self):
        return {**super().describe(), "cameras": list(self.cameras or []), "size": [self.width, self.height], "fps": self.fps}
