"""Recorded runs, in a plain on-disk layout, convertible to standard formats.

    runs/flight1/
      dataset.yaml                 what was recorded, camera intrinsics
      signals.npz                  time series (pose, twist, joints, imu, ...)
      images/<camera>/000000.png   frames
      images/<camera>/frames.json  per-frame time and camera-to-world pose

    ds = Dataset("runs/flight1")
    ds.export_nerfstudio("out/ns")         # transforms.json + images, for ns-train
    ds.export_tum("out/tum")               # rgb.txt + groundtruth.txt, for SLAM

Poses are simulator ground truth when the run came from a simulator.
"""
from __future__ import annotations

import json
import shutil
import struct
import zlib
from pathlib import Path

import numpy as np
import yaml

from .frames import mat_to_quat


def write_png(path, rgb: np.ndarray) -> None:
    """Write an HxWx3 uint8 array as a PNG. No imaging library needed."""
    rgb = np.ascontiguousarray(rgb[..., :3], dtype=np.uint8)
    h, w = rgb.shape[:2]
    raw = np.concatenate([np.zeros((h, 1), np.uint8), rgb.reshape(h, w * 3)], axis=1).tobytes()

    def chunk(tag, data):
        body = tag + data
        return struct.pack(">I", len(data)) + body + struct.pack(">I", zlib.crc32(body) & 0xFFFFFFFF)

    Path(path).write_bytes(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
                           + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))


class Dataset:
    def __init__(self, path):
        self.path = Path(path)
        meta = self.path / "dataset.yaml"
        if not meta.is_file():
            raise FileNotFoundError(f"{self.path} is not a recorded dataset (no dataset.yaml)")
        self.meta = yaml.safe_load(meta.read_text())

    @property
    def cameras(self) -> list:
        return list(self.meta.get("cameras", {}))

    def signals(self) -> dict:
        """The recorded time series as `{name: array}`."""
        f = self.path / "signals.npz"
        return dict(np.load(f)) if f.is_file() else {}

    def frames(self, camera: str = None) -> list:
        camera = self._camera(camera)
        return json.loads((self.path / "images" / camera / "frames.json").read_text())

    def _camera(self, camera):
        if camera is None:
            if not self.cameras:
                raise ValueError("this dataset has no camera frames")
            camera = self.cameras[0]
        if camera not in self.cameras:
            raise KeyError(f"no camera {camera!r} in this dataset. Recorded: {', '.join(self.cameras)}")
        return camera

    def export_nerfstudio(self, out, camera: str = None) -> Path:
        """nerfstudio layout: `transforms.json` + `images/`. Train with `ns-train splatfacto --data <out>`."""
        camera = self._camera(camera)
        out = Path(out)
        (out / "images").mkdir(parents=True, exist_ok=True)
        intr = self.meta["cameras"][camera]
        frames = []
        for fr in self.frames(camera):
            shutil.copyfile(self.path / "images" / camera / fr["file"], out / "images" / fr["file"])
            frames.append({"file_path": f"images/{fr['file']}", "transform_matrix": fr["pose"], "time": fr["t"]})
        doc = {"camera_model": "OPENCV", **{k: intr[k] for k in ("w", "h", "fl_x", "fl_y", "cx", "cy")},
               "k1": 0.0, "k2": 0.0, "p1": 0.0, "p2": 0.0, "frames": frames}
        (out / "transforms.json").write_text(json.dumps(doc, indent=2))
        return out

    def export_tum(self, out, camera: str = None) -> Path:
        """TUM RGB-D layout: `rgb/`, `rgb.txt` and `groundtruth.txt` (camera poses,
        `t tx ty tz qx qy qz qw`, camera frame z-forward as SLAM systems expect)."""
        camera = self._camera(camera)
        out = Path(out)
        (out / "rgb").mkdir(parents=True, exist_ok=True)
        flip = np.diag([1.0, -1.0, -1.0])       # OpenGL camera axes -> OpenCV camera axes
        rgb, gt = ["# timestamp filename"], ["# timestamp tx ty tz qx qy qz qw"]
        for fr in self.frames(camera):
            shutil.copyfile(self.path / "images" / camera / fr["file"], out / "rgb" / fr["file"])
            T = np.array(fr["pose"])
            w, x, y, z = mat_to_quat(T[:3, :3] @ flip)
            rgb.append(f"{fr['t']:.6f} rgb/{fr['file']}")
            gt.append(f"{fr['t']:.6f} {T[0, 3]:.6f} {T[1, 3]:.6f} {T[2, 3]:.6f} {x:.6f} {y:.6f} {z:.6f} {w:.6f}")
        (out / "rgb.txt").write_text("\n".join(rgb) + "\n")
        (out / "groundtruth.txt").write_text("\n".join(gt) + "\n")
        return out
