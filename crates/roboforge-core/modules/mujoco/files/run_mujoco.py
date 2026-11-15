"""Open a model in MuJoCo's passive viewer and step in real time.

Usage: python run_mujoco.py [model.xml]   (default: models/scene.xml next to this file)
"""
import sys
import time
from pathlib import Path

import mujoco
import mujoco.viewer

DEFAULT = Path(__file__).resolve().parent / 'models' / 'scene.xml'


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else str(DEFAULT)
    model = mujoco.MjModel.from_xml_path(path)
    data = mujoco.MjData(model)
    if model.nkey:
        mujoco.mj_resetDataKeyframe(model, data, 0)
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            start = time.time()
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(max(0.0, model.opt.timestep - (time.time() - start)))


if __name__ == '__main__':
    main()
