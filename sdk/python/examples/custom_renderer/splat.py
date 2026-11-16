"""A renderer written from scratch: Gaussian splats in plain numpy.

Every shape in the model becomes a handful of 3D Gaussians ("splats"). Each
frame they are moved to where physics put their bodies, projected through
the camera, sorted far to near and alpha-blended, the way Gaussian-splatting
renderers draw a scene. Floors are ray-traced as a checkerboard. No OpenGL,
no GPU: it works where MuJoCo's renderer cannot start.

It is an `rf.Renderer`, and the only method it has to provide is
`render(camera)`. The base class publishes `sensor/camera/<name>` at `fps`,
runs render passes, and `app.physics.camera(name)` says where the camera is,
so the ROS link, the recorder, the dataset exports and your policies take its
frames unchanged.

Swapping the splats made from the model for splats from a trained 3D
Gaussian-splatting scene, or the numpy blending for a GPU rasteriser, keeps
the same interface: that is the hook for neural rendering.
"""
import math

import numpy as np

import roboforge as rf

PLANE, SPHERE, CAPSULE, ELLIPSOID, CYLINDER, BOX = 0, 2, 3, 4, 5, 6


class SplatRenderer(rf.Renderer):
    name = "splat"

    def __init__(self, cameras=None, width=320, height=240, fps=10.0, opacity=0.9, density=1.0, sky=(0.62, 0.72, 0.85)):
        super().__init__(cameras=cameras, fps=fps)            # the base class publishes frames at `fps`
        self.width, self.height, self.density = int(width), int(height), float(density)
        self.opacity, self.sky = float(opacity), np.asarray(sky, float)

    # --- setup: turn the model's shapes into splats ---------------------------
    def setup(self, app):
        super().setup(app)                                    # bus, cameras (the robot's by default), outputs
        model = getattr(app.physics, "model", None)
        if model is None:
            raise rf.ConfigError("the splat renderer reads shapes from a MuJoCo model: use it with physics: mujoco")
        self.model, self.data, self.physics = model, app.physics.data, app.physics
        for c in self.cameras:
            app.physics.camera(c)                             # fails now, with the list of cameras, if one is wrong
        geom, local, sigma, color, planes = [], [], [], [], []
        for g in range(model.ngeom):
            kind, size = int(model.geom_type[g]), model.geom_size[g]
            rgba = model.mat_rgba[model.geom_matid[g]] if model.geom_matid[g] >= 0 else model.geom_rgba[g]
            if rgba[3] == 0:
                continue
            if kind == PLANE:
                planes.append((g, rgba[:3]))
                continue
            pts, s = self._sample(kind, size)
            geom += [g] * len(pts)
            local += list(pts)
            sigma += [s] * len(pts)
            color += [rgba[:3]] * len(pts)
        self._geom, self._local = np.array(geom, int), np.array(local, float).reshape(-1, 3)
        self._sigma, self._color = np.array(sigma, float), np.array(color, float).reshape(-1, 3)
        self._planes = planes

    def _sample(self, kind, size):
        """Points in the shape's own frame, and one Gaussian width for all of them."""
        d = self.density
        if kind == SPHERE:
            return [np.zeros(3)], 0.6 * size[0]
        if kind in (CAPSULE, CYLINDER):
            r, half = size[0], size[1]
            n = max(2, int(math.ceil(d * 2 * half / r)) + 1)
            return [np.array([0, 0, z]) for z in np.linspace(-half, half, n)], 0.7 * r
        if kind in (BOX, ELLIPSOID):
            s = 0.7 * float(np.min(size[:3]))
            axes = [np.linspace(-h + s / 2, h - s / 2, max(1, int(math.ceil(d * 2 * h / (2 * s))))) if h > s else [0.0] for h in size[:3]]
            return [np.array([x, y, z]) for x in axes[0] for y in axes[1] for z in axes[2]], s
        return [np.zeros(3)], 0.5 * float(np.mean(size[:3]))

    # --- rendering --------------------------------------------------------------
    def render(self, camera, depth=False):
        pose, fovy = self.physics.camera(camera)              # camera-to-world; columns: right, up, backwards
        R, eye = pose[:3, :3], pose[:3, 3]
        W, H = self.width, self.height
        f = (H / 2) / math.tan(math.radians(fovy) / 2)
        # Background: sky, then each floor plane, ray-traced.
        u, v = np.meshgrid(np.arange(W) + 0.5, np.arange(H) + 0.5)
        rays = np.stack([(u - W / 2) / f, -(v - H / 2) / f, -np.ones_like(u)], axis=-1) @ R.T
        image = np.broadcast_to(self.sky * (0.85 + 0.15 * (v / H))[..., None], (H, W, 3)).copy()
        zbuf = np.full((H, W), np.inf)
        for g, rgba in self._planes:
            n = self.data.geom_xmat[g].reshape(3, 3)[:, 2]
            p0 = self.data.geom_xpos[g]
            denom = rays @ n
            with np.errstate(divide="ignore", invalid="ignore"):
                t = ((p0 - eye) @ n) / denom
            hit = (t > 0) & (t < zbuf) & np.isfinite(t)
            pts = eye + rays * t[..., None]
            half = self.model.geom_size[g][:2]                 # a plane of size 0 is infinite
            local = (pts - p0) @ self.data.geom_xmat[g].reshape(3, 3)
            for k in range(2):
                if half[k] > 0:
                    hit &= np.abs(local[..., k]) <= half[k]
            checker = (np.floor(pts[..., 0] / 0.25) + np.floor(pts[..., 1] / 0.25)) % 2
            shade = rgba * (0.82 + 0.18 * checker)[..., None] * np.exp(-0.04 * np.nan_to_num(t, posinf=0))[..., None]
            image[hit], zbuf[hit] = shade[hit], t[hit]
        # Splats: world position from each shape's body pose, then into the camera.
        xmat = self.data.geom_xmat[self._geom].reshape(-1, 3, 3)
        world = self.data.geom_xpos[self._geom] + np.einsum("nij,nj->ni", xmat, self._local)
        cam = (world - eye) @ R                              # coordinates in the camera frame
        dist = -cam[:, 2]                                    # in front of the camera means negative z
        order = np.argsort(-dist)                            # far to near
        alpha_total = np.zeros((H, W))
        for i in order:
            if dist[i] <= 0.05:
                continue
            cx, cy = W / 2 + f * cam[i, 0] / dist[i], H / 2 - f * cam[i, 1] / dist[i]
            s = f * self._sigma[i] / dist[i]                 # the Gaussian's width on screen, pixels
            x0, x1 = int(max(0, cx - 3 * s)), int(min(W, cx + 3 * s + 1))
            y0, y1 = int(max(0, cy - 3 * s)), int(min(H, cy + 3 * s + 1))
            if x0 >= x1 or y0 >= y1:
                continue
            gx, gy = np.meshgrid(np.arange(x0, x1) + 0.5 - cx, np.arange(y0, y1) + 0.5 - cy)
            a = self.opacity * np.exp(-0.5 * (gx ** 2 + gy ** 2) / max(s, 0.5) ** 2)
            a = np.where(dist[i] < zbuf[y0:y1, x0:x1], a, 0.0)          # hidden behind the floor
            light = 0.75 + 0.25 * np.clip(-gy / max(s, 0.5), -1, 1)    # a little top lighting, for shape
            color = self._color[i] * light[..., None]
            image[y0:y1, x0:x1] = (1 - a[..., None]) * image[y0:y1, x0:x1] + a[..., None] * color
            alpha_total[y0:y1, x0:x1] = np.maximum(alpha_total[y0:y1, x0:x1], a)
        t_now = float(self.data.time)
        if depth:
            return rf.Image(data=np.where(np.isfinite(zbuf), zbuf, 0).astype(np.float32), camera=camera, encoding="depth32f", fovy=fovy, pose=pose, t=t_now)
        img = rf.Image(data=(np.clip(image, 0, 1) * 255).astype(np.uint8), camera=camera, fovy=fovy, pose=pose, t=t_now,
                       extras={"coverage": alpha_total})
        return self.finish(img)                               # runs the render passes

    def describe(self):
        return {**super().describe(), "cameras": list(self.cameras or []), "size": [self.width, self.height], "fps": self.fps,
                "splats": int(len(getattr(self, "_geom", [])))}
