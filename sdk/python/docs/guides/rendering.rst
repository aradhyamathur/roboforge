Rendering and recording
=======================

Physics and rendering are separate parts. Physics says where everything is;
a renderer turns that and a camera into an image; anything may then change
the image before it is published.

The renderer
------------

.. code-block:: yaml

   renderer: {type: mujoco, width: 640, height: 480, fps: 30, depth: true, cameras: [front]}

.. list-table::
   :header-rows: 1
   :widths: 18 82

   * - Option
     - Meaning
   * - ``cameras``
     - Camera names from the model. Default: the robot's ``cameras``
   * - ``width``, ``height``
     - Frame size in pixels. Up to full HD has been tested
   * - ``fps``
     - Frames per second of app time. Above the physics rate means one frame
       per step
   * - ``depth``
     - Also publish ``sensor/depth/<name>``, in metres

Each camera is published as ``sensor/camera/<name>``. Without a renderer an
app has no camera signals and costs nothing for them.

Cost: on an 8-core desktop a 320×240 camera at 30 fps roughly halves the
simulation speed (from about ×17 to ×8 real time for the quadrotor).
``examples/debugging/health_check.py`` measures it on your machine.

On demand, from any thread:

.. code-block:: python

   img = app.renderer.render("front")               # rf.Image
   depth = app.renderer.render("front", depth=True)

Each renderer owns one render thread that holds the OpenGL context, so
``render`` is safe to call from anywhere.

Passes
------

``renderer.add_pass(fn)`` appends a function ``(image, renderer) -> image``
run on every colour frame: a post-process, an overlay, sensor noise, or a
replacement picture from another engine. :doc:`../tutorials/render_pass` has
worked examples.

Without a display
-----------------

With no ``DISPLAY`` the SDK selects EGL (GPU, off-screen). To choose
yourself, set ``MUJOCO_GL`` before starting: ``egl``, ``osmesa`` (software)
or ``glfw`` (needs a display).

The viewer
----------

``- {type: viewer}`` opens MuJoCo's interactive window; closing it stops the
app. It is skipped, with a message, when there is no display, and
``roboforge run --headless`` removes it.

Recording
---------

``- {type: recorder, dir: runs/latest, rate: 50}`` records the run:

.. list-table::
   :header-rows: 1
   :widths: 18 82

   * - Option
     - Meaning
   * - ``dir``
     - Where to write. One run per folder: an earlier recording there is
       replaced
   * - ``rate``
     - Samples per second of the signals. Default: every step
   * - ``signals``
     - Which signals. Default: pose, twist, joints, IMU, actuator commands
   * - ``cameras``
     - Which cameras. Default: every frame published

Frames are written as they arrive, so memory does not grow with the length
of the run, and a run that ends in an error still leaves a readable dataset.
Reading and exporting recordings is :doc:`../tutorials/record_export`.
