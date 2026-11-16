Command line
============

.. code-block:: console

   $ python3 -m roboforge <command>

``run``
-------

.. code-block:: console

   $ python3 -m roboforge run CONFIG [path=value ...] [options]

.. list-table::
   :header-rows: 1
   :widths: 28 72

   * - Option
     - Meaning
   * - ``path=value``
     - Overrides, before or after the options (:doc:`../guides/config`)
   * - ``--duration S``
     - Seconds of app time to run. Default: the file's ``run.duration``, else
       until Ctrl+C
   * - ``--headless``
     - Remove the ``viewer`` callback
   * - ``--no-realtime``
     - Run as fast as possible
   * - ``--tap [S]``
     - Print every signal's rate, source and value every *S* seconds
       (default 1)
   * - ``--check``
     - Report common problems while running

Ctrl+C and SIGTERM stop the app cleanly: links are closed and recordings
saved. The last line printed is ``roboforge: stopped at t=...s after N
steps``.

``info``
--------

.. code-block:: console

   $ python3 -m roboforge info CONFIG [path=value ...] [--json] [--steps N]

The app's parts and signal wiring. Runs the app for *N* steps (default 50)
with links and callbacks off: no ports are opened and ROS is not needed.

``doctor``
----------

.. code-block:: console

   $ python3 -m roboforge doctor [--json]

Which optional parts import in this environment, with the fix for each that
does not.

``zoo``
-------

.. code-block:: console

   $ python3 -m roboforge zoo

The bundled robots.

``data export``
---------------

.. code-block:: console

   $ python3 -m roboforge data export DIR --format nerfstudio|tum --out OUT [--camera NAME]

Export a recording (:doc:`../tutorials/record_export`).

``cpp``
-------

.. code-block:: console

   $ python3 -m roboforge cpp include                 # the folder holding roboforge/controller.hpp
   $ python3 -m roboforge cpp new NAME                # write NAME.cpp, a starting controller
   $ python3 -m roboforge cpp build SOURCE [--out D]  # compile it; prints the library's path

See :doc:`../guides/cpp`.

Exit codes
----------

.. list-table::
   :widths: 12 88

   * - ``0``
     - Finished, or stopped by Ctrl+C / SIGTERM
   * - ``1``
     - An exception from user code (its traceback is printed)
   * - ``2``
     - A reported error: a mistake in the app file or the command, a missing
       file or package, a port in use, a peer that never connected, an
       unstable simulation. One line, ``roboforge: error: ...``

Environment variables
---------------------

.. list-table::
   :widths: 28 72

   * - ``MUJOCO_GL``
     - OpenGL backend for rendering: ``egl``, ``osmesa`` or ``glfw``.
       Default: ``egl`` when there is no display
   * - ``ROS_DOMAIN_ID``, ``ROS_LOCALHOST_ONLY``
     - As for any ROS 2 program
