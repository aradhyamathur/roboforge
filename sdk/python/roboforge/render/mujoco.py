"""MuJoCo's own renderer as a camera source."""
from __future__ import annotations

import queue
import threading

import numpy as np

from .. import msgs
from ..component import Renderer
from ..errors import ConfigError, positive


class MujocoRenderer(Renderer):
    """Renders the robot's cameras and publishes `sensor/camera/<name>`.

    `cameras` defaults to the robot's camera list. `depth=True` also
    publishes `sensor/depth/<name>` in metres. Passes added with `add_pass`
    run on every colour frame before it is published.
    """
    name = "mujoco"

    def __init__(self, cameras=None, width: int = 320, height: int = 240, fps: float = 10.0, depth: bool = False):
        super().__init__(cameras=cameras)
        positive(width, "renderer width", integer=True)
        positive(height, "renderer height", integer=True)
        positive(fps, "renderer fps")
        if isinstance(cameras, str):
            cameras = [cameras]
        self.cameras, self.width, self.height, self.fps, self.depth = cameras, int(width), int(height), fps, depth
        self._mj = None
        self._thread, self._jobs, self._start = None, None, threading.Lock()
        self._next = 0.0

    def setup(self, app):
        import mujoco
        physics = app.physics
        if getattr(physics, "model", None) is None:
            raise ConfigError("the mujoco renderer needs the mujoco physics backend and a robot (it renders that model). "
                              "Use another renderer with other physics backends.")
        self.bus, self.model, self.data = app.bus, physics.model, physics.data
        if self.cameras is None:
            self.cameras = list(app.robot.cameras) if app.robot else []
        self._mj = mujoco
        for c in self.cameras:
            self._camera_id(c)
        # MuJoCo renders into an off-screen buffer whose size the model fixes.
        vis = self.model.vis.global_
        if self.width > vis.offwidth or self.height > vis.offheight:
            vis.offwidth, vis.offheight = max(vis.offwidth, self.width), max(vis.offheight, self.height)
        self.outputs = [msgs.CAMERA.format(name=c) for c in self.cameras] + ([msgs.DEPTH.format(name=c) for c in self.cameras] if self.depth else [])
        self._next = 0.0

    # An OpenGL context belongs to the thread that created it, and some backends crash
    # if it is used or destroyed from another. So one worker thread per renderer creates
    # the context, renders every frame and closes it; `render()` may be called from anywhere.
    def _worker(self, jobs):
        r = None
        while True:
            job = jobs.get()
            if job is None:
                break
            fn, box, done = job
            try:
                if r is None:
                    r = self._mj.Renderer(self.model, self.height, self.width)
                box["out"] = fn(r)
            except BaseException as e:
                box["err"] = e
            done.set()
        if r is not None:
            try:
                r.close()
            except Exception:
                pass

    def _on_render_thread(self, fn):
        with self._start:
            if self._thread is None or not self._thread.is_alive():
                self._jobs = queue.SimpleQueue()
                self._thread = threading.Thread(target=self._worker, args=(self._jobs,), name="roboforge-render", daemon=True)
                self._thread.start()
            jobs = self._jobs
        box, done = {}, threading.Event()
        jobs.put((fn, box, done))
        done.wait()
        if "err" in box:
            e = box["err"]
            if "context" in str(e).lower() or "egl" in str(e).lower() or "glfw" in str(e).lower():
                raise RuntimeError(f"could not create an OpenGL context for off-screen rendering ({type(e).__name__}: {e}). "
                                   f"On a machine without a display set MUJOCO_GL=egl (GPU) or MUJOCO_GL=osmesa (software) "
                                   f"before starting; `python -m roboforge doctor` shows what is in use.") from e
            raise e
        return box["out"]

    def _pose(self, camera: str) -> np.ndarray:
        cid = self._camera_id(camera)
        T = np.eye(4)
        T[:3, :3] = self.data.cam_xmat[cid].reshape(3, 3)     # columns: camera x, y, z axes in the world; looks along -z
        T[:3, 3] = self.data.cam_xpos[cid]
        return T

    def _camera_id(self, camera: str) -> int:
        cid = self._mj.mj_name2id(self.model, self._mj.mjtObj.mjOBJ_CAMERA, camera) if isinstance(camera, str) else -1
        if cid < 0:
            have = [self._mj.mj_id2name(self.model, self._mj.mjtObj.mjOBJ_CAMERA, i) for i in range(self.model.ncam)]
            raise ConfigError(f"the model has no camera named {camera!r}. Its cameras: {', '.join(h for h in have if h) or 'none'}")
        return cid

    def render(self, camera: str, depth: bool = False) -> msgs.Image:
        """One frame from `camera`, now. Safe to call from any thread."""
        if self._mj is None:
            raise RuntimeError("the renderer is not set up yet: call app.setup() (or app.step()) before render()")
        cid = self._camera_id(camera)

        def draw(r):
            r.update_scene(self.data, camera=camera)
            if not depth:
                return r.render().copy()
            r.enable_depth_rendering()
            try:
                return r.render().copy()
            finally:
                r.disable_depth_rendering()
        data = self._on_render_thread(draw)
        fovy, pose, t = float(self.model.cam_fovy[cid]), self._pose(camera), float(self.data.time)
        if depth:
            return msgs.Image(data=data, camera=camera, encoding="depth32f", fovy=fovy, pose=pose, t=t)
        return self.finish(msgs.Image(data=data, camera=camera, fovy=fovy, pose=pose, t=t))

    def step(self, t: float):
        if t + 1e-9 < self._next:
            return
        self._next = max(self._next + 1.0 / self.fps, t)
        for c in self.cameras:
            self.bus.publish(msgs.CAMERA.format(name=c), self.render(c), "mujoco-renderer")
            if self.depth:
                self.bus.publish(msgs.DEPTH.format(name=c), self.render(c, depth=True), "mujoco-renderer")

    def close(self):
        with self._start:
            th, self._thread = self._thread, None
            if th is not None and th.is_alive():
                self._jobs.put(None)        # the render thread closes its own context
        if th is not None and th is not threading.current_thread():
            th.join(timeout=5.0)

    def describe(self):
        return {**super().describe(), "cameras": list(self.cameras or []), "size": [self.width, self.height], "fps": self.fps}
