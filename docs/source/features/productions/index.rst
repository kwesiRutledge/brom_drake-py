Productions
============

Example Productions
-------------------

In case you want to "dive right in" to using Productions, we'll list a few here that you might find interesting/useful:

.. toctree::
   :maxdepth: 1
   :glob:

   show_me_this_model

What is a Production?
---------------------

The short answer
^^^^^^^^^^^^^^^^

A Production is a partially built Drake diagram that is waiting for your code (aka the "star,"" or the "lead actor") to start the show.

We use the term Production to evoke the language of the theatre. You can add a cast to your production (most cast members will not need to be specified). The only cast member you need to add is **the algorithm you want to test**.

Once you add this cast member (and the supporting cast), then simulate the scene (using Drake's normal Simulator object) and enjoy the show!

The long answer
^^^^^^^^^^^^^^^

Brom's :code:`Productions` were created after noticing a frustrating part of robot simulation: In most cases, we start to use a robot simulation to test a *specific algorithm*.

For example, we might want to test a new motion planning algorithm. In order to test that algorithm, however, we are asked to build a ton of OTHER things in the simulated world, including:

- A robot model,
- A set of objects to populate the world,
- A joint controller, and
- A finite state machine to start and begin tracking the motion plan.

Then the :code:`Production` will wait for you to add your planning algorithm using the :code:`fill_role` method. From there, you can simulate the scene!

In more concrete terms, by doing the following operations you can simulate the scene:

1. Create your Production (e.g., :code:`production = ChemLab1()`)
2. Add the supporting cast (e.g., :code:`production.add_supporting_cast()`)
3. Add YOUR roles (e.g., use :code:`production.add_main_cast()` or :code:`production.fill_role()`)
4. Build the production (e.g., use :code:`self.build_production()`)
5. Create a Drake simulator for the production (e.g., use :code:`simulator = Simulator(diagram, diagram_context)`)
6. Use your own approach to run the simulator (one way is to use :code:`simulator.AdvanceTo(desired_end_time)`)

To give you a taste of what a :code:`Production` can be, we include many examples for how to use the scenes in the examples directory.

Coming soon...