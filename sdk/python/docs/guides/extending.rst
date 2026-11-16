Extending it
============

Every part is a small class. Yours are named in an app file as
``module:Class`` and behave exactly like the built-in ones. Four worked
examples, each a tutorial with a runnable folder:

.. list-table::
   :header-rows: 1
   :widths: 26 20 54

   * - You write
     - You implement
     - The SDK does
   * - A learned policy (:doc:`../tutorials/custom_policy`)
     - ``observe``, ``act``
     - the Gymnasium environment and its spaces, conversion of numpy, PyTorch
       or JAX output to messages, waiting for inputs, deployment as a
       controller
   * - A renderer (:doc:`../tutorials/custom_renderer`)
     - ``render(camera)``
     - choosing cameras, publishing at ``fps``, render passes, the camera's
       pose from ``physics.camera``
   * - A physics solver (:doc:`../tutorials/custom_solver`)
     - ``step``, ``reset``
     - the clock and time stamps (``publish_state``); every controller, link
       and callback works on top
   * - Sensors and a SLAM method (:doc:`../tutorials/custom_slam`)
     - callbacks that publish
     - recording any message type, ATE / RPE / alignment, TUM files
   * - A controller in C++ (:doc:`../tutorials/cpp_controller`)
     - ``step`` in a class, ``RF_EXPORT_CONTROLLER``
     - compiling, caching, loading, passing data, parameters with typo
       checks, catching exceptions

.. list-table::
   :header-rows: 1
   :widths: 16 44 40

   * - Part
     - Must provide
     - May provide
   * - ``Controller``
     - ``step(obs, t)``
     - ``setup(app)``, ``reset()``, ``close()``, ``rate``
   * - ``Callback``
     - nothing
     - ``on_start(app)``, ``on_step(app, t)``, ``on_stop(app)``
   * - ``Policy``
     - ``observe(obs)``, ``act(x)``, ``output``
     - ``make_env``, ``action_space``, ``command``
   * - ``Renderer``
     - ``render(camera) -> Image``
     - ``setup`` (call ``super().setup(app)``), ``finish(img)`` (passes),
       ``step(t)`` is provided
   * - ``Link``
     - nothing
     - ``setup(app)``, ``before_step(t)``, ``after_step(t)``, ``close()``
   * - ``Physics``
     - ``dt``, ``time``, ``step()``, ``reset()``
     - ``publish_state(...)``, ``camera(name)``, ``raw``, ``joint_names``,
       ``actuator_names``, ``ctrl_range``, ``joint_range``, ``allocation``

All have ``setup(app)`` and ``close()``, and ``inputs`` / ``outputs`` lists
of signal names, which is what ``roboforge info`` and the IDE's Stack view
show.

A callback
----------

.. code-block:: python

   import roboforge as rf

   class Lap(rf.Callback):
       """Print the time whenever the base crosses x = 0 going forward."""
       def on_start(self, app):
           self.last = None

       def on_step(self, app, t):
           x = app.bus.latest("state/pose").position[0]
           if self.last is not None and self.last < 0 <= x:
               print(f"lap at t={t:.2f}s")
           self.last = x

.. code-block:: yaml

   callbacks:
     - {type: "lap:Lap"}

A link
------

A link moves signals between the bus and something outside. Subscribe to
what you send; publish what you receive.

.. code-block:: python

   import json, socket
   import roboforge as rf

   class UdpJson(rf.Link):
       """Send the pose as JSON over UDP."""
       name = "udp_json"
       inputs = ["state/pose"]

       def __init__(self, host="127.0.0.1", port=9870, rate=20.0):
           self.addr, self.period, self.next = (host, port), 1.0 / rate, 0.0

       def setup(self, app):
           self.bus = app.bus
           self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

       def after_step(self, t):
           if t < self.next:
               return
           self.next = t + self.period
           p = self.bus.latest("state/pose")
           self.sock.sendto(json.dumps({"t": t, "position": p.position.tolist()}).encode(), self.addr)

       def close(self):
           self.sock.close()

For data arriving from outside, check it before publishing and call
``self.reject(what, why)`` for anything that cannot be right; ``self.finite``
tests for ``nan`` and ``inf``. That is the rule the built-in links follow:
bad input is dropped and reported, never passed on.

Your own message types
----------------------

A message is any object; a dataclass with a ``signal`` class attribute can be
returned from a controller and lands on that signal. Give it ``as_array()``
and the recorder keeps it too:

.. code-block:: python

   from dataclasses import dataclass
   from typing import ClassVar

   @dataclass
   class WheelOdometry:
       signal: ClassVar[str] = "sensor/odometry"
       v: float = 0.0
       w: float = 0.0
       def as_array(self):
           return [self.v, self.w]

A physics backend
-----------------

A backend for another simulator implements ``Physics``: publish ``clock``,
``state/pose``, ``state/twist``, ``state/joint`` and ``sensor/imu`` after
each step, and apply ``cmd/actuators``. Then every link, controller and
callback works with it unchanged.

.. code-block:: python

   class MySim(rf.Physics):
       name = "mysim"
       inputs = ["cmd/actuators"]

       def setup(self, app):
           self.bus, self.dt, self.time = app.bus, 0.002, 0.0
           self.bus.subscribe("cmd/actuators", lambda name, msg: self.apply(msg.values))
           ...

       def step(self):
           ...                                  # advance the simulator by dt
           self.time += self.dt
           self.publish_state(pose=rf.Pose(position=..., orientation=...))    # also publishes the clock

       def reset(self): ...

The optional attributes buy features: ``joint_names`` and ``actuator_names``
let ``joint_pd`` and the links check sizes, ``ctrl_range`` gives the Gym
action space, ``allocation`` lets the ``multirotor`` controller and the PX4
link fly it.

A backend for a simulator that runs as its own process (Gazebo, Isaac)
steps that process and waits for it in ``step``. None is bundled yet.

Registering names
-----------------

To use a short name instead of ``module:Class``:

.. code-block:: python

   from roboforge.registry import PHYSICS, LINKS, CONTROLLERS, CALLBACKS, RENDERERS

   PHYSICS.register("mysim", "my_package.sim:MySim")       # imported only when used

A package can do this for everyone who installs it through the
``roboforge.plugins`` entry-point group:

.. code-block:: toml

   [project.entry-points."roboforge.plugins"]
   my_package = "my_package.register"

The bus by itself
-----------------

The bus is a small thread-safe object and needs no app:

.. code-block:: python

   bus = rf.Bus()
   stop = bus.subscribe("sensor/*", lambda name, msg: print(name))     # patterns allowed
   bus.publish("sensor/imu", rf.Imu())
   bus.latest("sensor/imu"); bus.count("sensor/imu"); bus.names(); bus.describe()
   stop()

Subscribers run in the publisher's thread, so keep them short. If one
raises, the others still run and the exception then reaches the publisher,
with a note naming the signal.
