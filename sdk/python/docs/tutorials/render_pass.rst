12. Change what the camera sees
===============================

**You will:** add render passes that alter every camera frame before
anything else receives it. **Time:** 5 minutes. **Needs:** MuJoCo.

A pass
------

A pass is a function ``(image, renderer) -> image``, run on every frame
before it is published. Everything downstream, the ROS link, the recorder, a
policy, gets the changed picture.

.. literalinclude:: ../../examples/render_pass.py
   :language: python
   :pyobject: depth_tint

.. literalinclude:: ../../examples/render_pass.py
   :language: python
   :pyobject: sensor_noise

A pass either returns a new ``rf.Image`` or changes the one it was given and
returns nothing.

.. code-block:: console

   $ python3 render_pass.py
   plain       320x240  differs from plain by   0.0 grey levels  -> frames/plain.png
   depth_tint  320x240  differs from plain by  48.0 grey levels  -> frames/depth_tint.png
   noisy       320x240  differs from plain by   6.7 grey levels  -> frames/noisy.png

What an image carries
---------------------

.. list-table::
   :widths: 20 80

   * - ``data``
     - ``uint8`` array, height × width × 3 (or ``float32`` metres for depth)
   * - ``camera``
     - the camera's name
   * - ``pose``
     - 4×4 camera-to-world matrix (OpenGL axes: looks along −z)
   * - ``fovy``, ``intrinsics``
     - vertical field of view; ``fl_x``, ``fl_y``, ``cx``, ``cy``, ``w``, ``h``
   * - ``t``
     - app time of the frame

Because the pose and intrinsics travel with the picture, a pass can replace
the picture entirely with one rendered by something else from the same
viewpoint: another engine, a Gaussian-splat scene, a learned model. Physics
and rendering are separate parts for that reason.

``renderer.render("front")`` renders one frame on demand and may be called
from any thread.

If something goes wrong
-----------------------

``render pass 0 (my_pass) returned a str; a pass returns an rf.Image, or None``
   Return the image, or nothing.

An exception inside a pass
   Carries a note naming the pass and the camera:
   ``(in render pass 1, depth_tint, on camera 'front')``.

``could not create an OpenGL context for off-screen rendering``
   See :ref:`debugging/troubleshooting:Rendering and cameras`.
