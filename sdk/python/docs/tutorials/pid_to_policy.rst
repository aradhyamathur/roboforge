7. From a PID loop to a slow policy
===================================

**You will:** run a slow model above a fast control loop without slowing the
loop down, and see the fallback that covers for the model when it is late.
**Time:** 10 minutes. **Needs:** MuJoCo. No GPU and no model download: the
"model" is a ``time.sleep``.

The problem
-----------

A vision-language-action model answers a few times a second. A motor needs a
command hundreds of times a second. Put the model in the control loop and the
robot is controlled at the model's pace.

The structure
-------------

Three controllers share the arm, each at its own rate:

.. literalinclude:: ../../examples/pid_to_policy.py
   :language: python
   :lines: 3-11

.. literalinclude:: ../../examples/pid_to_policy.py
   :language: python
   :pyobject: SlowPolicy

``rf.Async`` runs the wrapped controller in a worker thread on a frozen copy
of the observations. Its own ``step`` returns at once with the next buffered
action. A ``rf.Chunk`` is several future actions from one inference, played
out at the control rate. When nothing fresh has arrived for ``timeout``
seconds, the ``fallback`` applies.

Run it
------

.. code-block:: console

   $ python3 pid_to_policy.py 20 --headless
   roboforge: t=19.7s  x0.98 real time  async(SlowPolicy) 50 Hz  JointPD 500 Hz
   policy: 62 inferences, 300 ms each, 16 late steps; PID loop ran at 500 Hz throughout

The PID loop held 500 Hz while each inference took 300 ms. The 16 late steps
are the fallback holding a safe pose while the very first inference was still
running.

Swap in a real model
--------------------

Only ``SlowPolicy.step`` changes: build the model in ``setup``, run it in
``step``, return a ``Chunk`` (or a single message). Images are
``obs.camera("front")``; a language instruction is just another signal,
``obs["cmd/instruction"]``.

What ``Async`` tells you afterwards:

.. list-table::
   :widths: 24 76

   * - ``inferences``
     - completed calls of the model
   * - ``latency``
     - seconds the last call took
   * - ``late``
     - steps answered by the fallback
   * - ``errors``, ``last_error``
     - calls that raised. A crashing model does not stop the loop: the
       fallback applies and the exception is kept here

Try next
--------

- Raise the sleep in ``SlowPolicy`` beyond the chunk length and watch ``late``
  grow while the PID rate stays at 500 Hz.
- Make the policy raise now and then; check ``errors`` afterwards.

If something goes wrong
-----------------------

The robot freezes between inferences
   The chunk is shorter than the time between inferences. Return more actions
   per chunk, or choose a fallback that keeps moving.

``roboforge: async(SlowPolicy) raised RuntimeError: ...``
   The model failed; the loop carried on with the fallback. The full
   exception is in ``async_controller.last_error``.
