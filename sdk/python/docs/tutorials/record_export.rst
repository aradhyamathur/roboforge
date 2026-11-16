8. Record a run and export a dataset
====================================

**You will:** record a flight with a camera and export it for NeRF or
Gaussian splatting and for SLAM, with ground-truth camera poses. **Time:** 5
minutes. **Needs:** MuJoCo.

Record
------

.. literalinclude:: ../../examples/record_and_export.py
   :language: python
   :lines: 9-

.. code-block:: console

   $ python3 record_and_export.py
   recorded 251 frames from camera 'front'
   nerfstudio: runs/flight/nerfstudio
   TUM:        runs/flight/tum

About four seconds, and 36 MB under ``runs/flight``.

``rf.Recorder`` is a callback: add it to any app (``- {type: recorder, dir:
runs/latest}`` in ``app.yaml``) and that run is recorded. A folder holds one
run; recording into it again replaces what was there.

What is in a recording
----------------------

.. code-block:: text

   runs/flight/
     dataset.yaml            what was recorded: robot, duration, signals, cameras with intrinsics
     signals.npz             the time series
     images/front/000000.png ...
     images/front/frames.json   per frame: file, time, camera pose

.. code-block:: python

   ds = rf.Dataset("runs/flight")
   ds.cameras                         # ['front']
   ds.frames()                        # [{'file': '000000.png', 't': 0.0, 'pose': [[...4x4...]]}, ...]
   ds.signals()["state/pose"]         # array (N, 7): position, then quaternion (w, x, y, z)
   ds.times("state/pose")             # array (N,): app time of each sample

Rows of the other signals: ``state/twist`` is linear then angular velocity,
``state/joint`` positions then velocities, ``sensor/imu`` acceleration then
angular rate, ``cmd/actuators`` one value per motor.

Export
------

.. list-table::
   :header-rows: 1
   :widths: 18 40 42

   * - Format
     - Call
     - For
   * - nerfstudio
     - ``ds.export_nerfstudio(out)``
     - ``ns-train nerfacto`` / ``splatfacto --data <out>``
   * - TUM RGB-D
     - ``ds.export_tum(out)``
     - ORB-SLAM3 and other SLAM systems; ``groundtruth.txt`` to score them

.. code-block:: console

   $ python3 -m roboforge data export runs/flight --format tum --out runs/flight/tum

The camera poses are the simulator's ground truth, so there is no COLMAP
step before training, and a SLAM trajectory can be scored against
``groundtruth.txt`` directly.

.. note::

   Training on the export is the heavy step and is not part of this tutorial:
   ``ns-train splatfacto`` needs nerfstudio and a CUDA GPU. For a smaller
   dataset lower ``width``, ``height`` or ``fps`` in the script.

Try next
--------

- :ref:`debugging/toolbox:Compare two runs` uses recordings to answer "did my
  change help?".
- Record depth too: ``rf.MujocoRenderer(depth=True)``.

If something goes wrong
-----------------------

``runs/x has no camera frames: it was recorded without a renderer``
   Signals are always recorded; images only when the app has a renderer.

``no camera 'rear' in this dataset. Recorded: front``
   Pass one of the recorded cameras, or leave ``camera`` out for the first.
