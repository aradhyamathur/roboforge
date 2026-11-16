18. Write a controller in C++
=============================

**You will:** write a controller in C++, run it under a Python one in the
same app, check it against the built-in controller, and build it with CMake.
**Time:** 10 minutes. **Needs:** MuJoCo and a C++ compiler (``g++`` or
``clang++``). No CMake needed unless you want it.

What it takes
-------------

A class with a constructor and ``step``, one line to export it, one line in
``app.yaml``. No bindings to write, no build files, no Python:

.. code-block:: cpp

   #include <roboforge/controller.hpp>

   class Hold : public rf::Controller {
    public:
     explicit Hold(const rf::Params& p) : kp_(p.number("kp", 40.0)) {}
     rf::Action step(const rf::Obs& obs) override {
       rf::Vec u(obs.joints());
       for (int i = 0; i < obs.joints(); ++i) u[i] = -kp_ * obs.q(i) - 4.0 * obs.qd(i);
       return rf::Action::actuators(u);
     }
    private:
     double kp_;
   };
   RF_EXPORT_CONTROLLER(Hold)

.. code-block:: yaml

   controllers:
     - {type: cpp, source: hold.cpp, rate: 500, kp: 60}

The first ``run`` compiles ``hold.cpp`` into ``build/``; later runs reuse
the library until the source changes. ``kp: 60`` reaches the constructor
through ``rf::Params``. Start from a template with:

.. code-block:: console

   $ python3 -m roboforge cpp new hold
   wrote hold.cpp
   use it in app.yaml:  - {type: cpp, source: hold.cpp, rate: 500}

The example: Python above, C++ below
------------------------------------

``examples/cpp_controller`` splits the work: a Python controller decides
where the joints should go, a C++ PID loop gets them there.

.. literalinclude:: ../../examples/cpp_controller/app.yaml
   :language: yaml
   :lines: 6-

``inputs: [cmd/joint]`` hands the Python controller's targets to the C++ one,
which reads them with ``obs.signal("cmd/joint")``. Any signal can be listed:
a goal, an estimate, your own message type with ``as_array()``.

.. literalinclude:: ../../examples/cpp_controller/arm_pid.cpp
   :language: cpp
   :lines: 7-

.. code-block:: console

   $ cd cpp_controller
   $ python3 -m roboforge run app.yaml --headless --duration 5 --tap
   roboforge: compiling arm_pid.cpp ...
     cmd/actuators    500.0 Hz  ArmPID    u [-0.02 -0.63 0.22]
     cmd/joint         50.0 Hz  Wave      q* [0.55 0.22 -0.90]

Check it against the built-in controller
----------------------------------------

``arm_pid.cpp`` implements the SDK's Python ``joint_pd`` line for line, so
``compare.py`` runs the app with each and compares the joints:

.. code-block:: console

   $ python3 compare.py
   low-level controller     per call
   C++ (ArmPID)               23.5 us   of which 2.9 us in C++; the rest is handing the data across
   Python (JointPD)           16.3 us

   final joints: C++ [ 0.1632  0.253  -0.9089]  Python [ 0.1632  0.253  -0.9089]
   largest difference over 500 samples: 0.00e+00 rad -> identical

Identical to the last bit. Notice the timing: handing the observations to C++
and the command back costs about ten microseconds, more than this tiny PID
itself. C++ is worth it when the control law is heavy (an MPC, a filter, an
inverse-kinematics solver, a network in libtorch) or when it already exists
in C++. For a three-joint PID, Python is just as good.

Build it with CMake instead
---------------------------

For several files, libraries to link (Eigen, a QP solver, libtorch) or an
existing CMake project, build the library yourself and point ``library:`` at
it:

.. literalinclude:: ../../examples/cpp_controller/CMakeLists.txt
   :language: cmake
   :lines: 6-

.. code-block:: console

   $ cmake -S . -B build/cmake && cmake --build build/cmake
   $ python3 -m roboforge run app_cmake.yaml --headless --duration 5

.. literalinclude:: ../../examples/cpp_controller/app_cmake.yaml
   :language: yaml
   :lines: 8-

In the IDE
----------

**New ▸ Build on the RoboForge SDK ▸ Your controller in: C++** generates
``app/controller.cpp`` instead of ``controller.py`` (a joint-target, setpoint,
velocity or PID controller, depending on the robot), a run configuration
*C++: build controller*, and a ``.clangd`` file so VS Code's C++ support
finds the SDK header.

If something goes wrong
-----------------------

``arm_pid.cpp did not compile:`` followed by the compiler's errors
   Fix them and run again; the command used is printed at the end.

``C++ controller ArmPID does not read parameter 'kpp'. Did you mean 'kp'? It reads: kp, ki, kd, integral_limit.``
   Every key in the controller's line must be read by the constructor:
   typos are caught, not ignored.

``C++ controller ArmPID threw: joint index 7 out of range (the robot has 3)``
   An exception from ``step`` is caught at the boundary and reported with the
   controller's name. A crash in C++ (a bad pointer) cannot be caught: it ends
   the process, as it would anywhere.

``libx.so is not a RoboForge controller (no rf_create ...)``
   ``RF_EXPORT_CONTROLLER(YourClass)`` is missing at the end of the source.

``no C++ compiler found``
   Install ``g++`` (``sudo apt install g++``), set ``CXX``, or build the
   library elsewhere and give ``library:``.
