17. Add your own sensors and SLAM
=================================

**You will:** add simulated sensors, run a SLAM method on them while the
robot drives, and score it against ground truth with the standard metrics.
**Time:** 10 minutes. **Needs:** MuJoCo.

What it takes
-------------

Both are callbacks: code that runs after every physics step.

- A **sensor** reads the true state and publishes a noisy measurement:
  ``sensor/odometry`` (wheel speeds with a small turn-rate bias, so dead
  reckoning drifts as real odometry does) and ``sensor/landmarks`` (range and
  bearing to posts within 3 m).
- A **SLAM method** subscribes to those and publishes its estimate as an
  ordinary ``rf.Pose`` on ``slam/pose``.

.. literalinclude:: ../../examples/custom_slam/app.yaml
   :language: yaml
   :lines: 4-

The sensors' messages are dataclasses with a ``signal`` name and an
``as_array()``, which is all the recorder needs to keep them:

.. literalinclude:: ../../examples/custom_slam/sensors.py
   :language: python
   :pyobject: WheelOdometry

The method
----------

``slam.py`` is EKF-SLAM with known landmark identities: odometry predicts,
each sighting corrects the robot and the landmark, an unseen landmark is
added to the map. Its contact with the SDK:

.. code-block:: python

   class EkfSlam(rf.Callback):
       def on_start(self, app):
           app.bus.subscribe("sensor/odometry", self._predict)
           app.bus.subscribe("sensor/landmarks", self._update)

       def _publish(self, t):
           self.bus.publish("slam/pose", rf.Pose(position=..., orientation=..., t=t), self.name)

Score it
--------

``rf.eval`` has the numbers SLAM papers report and the file format their
tools read; ``Dataset.trajectory`` returns any recorded pose signal:

.. literalinclude:: ../../examples/custom_slam/evaluate.py
   :language: python
   :pyobject: main
   :lines: 1-17

.. code-block:: console

   $ cd custom_slam && python3 evaluate.py
   estimate          ATE (m)   RPE over 1 s (m)  final error (m)
   odometry only       0.760              0.289            1.280
   EKF-SLAM            0.052              0.017            0.021

   map: 8 of 8 landmarks found, placed within 0.121 m (mean 0.086 m)
   trajectories in TUM format: runs/slam/groundtruth.txt, slam.txt, odometry.txt

A minute of simulated driving in three seconds. Ground truth comes free from
the simulator, so every run is scored, and runs are deterministic, so a
change to the filter shows up as a change in the numbers and nothing else.

.. list-table::
   :widths: 30 70

   * - ``rf.eval.ate(est, gt)``
     - absolute trajectory error, RMS, metres. ``align_first=True`` (and
       ``scale=True``) for methods with their own origin or unknown scale
   * - ``rf.eval.rpe(est, gt, delta)``
     - drift over ``delta`` samples
   * - ``rf.eval.associate(t_est, t_gt)``
     - match samples by time, for estimates at their own rate
   * - ``rf.eval.write_tum`` / ``read_tum``
     - TUM trajectory files, for evo and the TUM benchmark scripts

.. code-block:: console

   $ evo_ape tum runs/slam/groundtruth.txt runs/slam/slam.txt -p     # if evo is installed

A real SLAM system
------------------

The same two pieces wrap ORB-SLAM3, FAST-LIO, slam_toolbox or a learned
odometry network:

- **Live:** a callback feeds it ``sensor/camera/<name>`` or ``sensor/imu``
  as they are published and publishes its pose as ``slam/pose``. Over ROS 2,
  the ROS link already publishes the camera and IMU topics such systems
  subscribe to.
- **Offline:** record with a renderer, export TUM (:doc:`record_export`),
  run the system on the export, read its trajectory back with
  ``rf.eval.read_tum``.

Either way ``evaluate.py`` does not change.

If something goes wrong
-----------------------

``no signal 'slam/pose' in this dataset``
   The recorder's ``signals`` must list it, and the SLAM callback must come
   before the recorder in ``callbacks``.

``'sensor/landmarks' is not a pose signal``
   ``trajectory`` reads pose signals only; use ``ds.signals()`` for others.

The estimate drifts like odometry
   No landmarks are being seen (``--tap`` shows ``sensor/landmarks`` with
   empty values), or ``range_noise`` in the filter is set far too high.
