Scenarios
=========

A production's ``Scenario`` lists the initial conditions that a caller may
change from outside (for example, a table's height or a shelf's pose), with
each value's type and allowed range. Tools such as Saphira read it to run one
production under several checked sets of values.

Values arrive as plain JSON. :py:meth:`Scenario.validate
<brom_drake.productions.types.base.scenario.Scenario.validate>` checks them
against the declaration and reports every problem at once in a
:py:class:`~brom_drake.productions.types.base.scenario.ScenarioError`.

Declaring a Scenario
--------------------

Declare the scenario as a dataclass nested in the production, and declare each
field with ``param``:

.. code-block:: python

    from dataclasses import dataclass

    from brom_drake.productions.types.base import FloatParam, PoseParam, param


    class ChemLab1(KinematicMotionPlanningProduction):
        @dataclass
        class Scenario(KinematicMotionPlanningProduction.Scenario):
            table_height: float = param(FloatParam(0.05, 0.5, "m"), default=0.1)
            shelf_pose: dict = param(
                PoseParam(xyz_min=(-1.0, 0.0, 0.0), xyz_max=(1.0, 1.5, 1.5)),
                default=None,
            )

Rules:

- Every field name must be a keyword argument of the production's constructor
  (or of a parent class's constructor). Callers apply a value by passing it as
  that keyword argument.
- A default of ``None`` means "use the constructor's own default".
- Use ``mutually_exclusive`` for arguments that are alternatives, for example
  ``mutually_exclusive = (("start_configuration", "start_pose"),)``.

Parameter types
---------------

.. list-table::
   :header-rows: 1

   * - Type
     - JSON value
     - Bounds
   * - ``FloatParam(min, max, units="")``
     - a number
     - ``min`` to ``max``
   * - ``VectorParam(length, min, max, units="")``
     - a list of ``length`` numbers
     - ``min[i]`` to ``max[i]`` for each entry
   * - ``PoseParam(xyz_min, xyz_max, rpy_min, rpy_max)``
     - ``{"xyz": [...], "rpy": [...]}`` or ``{"xyz": [...], "wxyz": [...]}``
     - per component; see below

``true`` and ``false``, NaN and infinity are never accepted as numbers.

.. _pose-orientation-convention:

Pose orientation convention
---------------------------

A pose is ``"xyz"`` (metres) plus **exactly one** orientation:

- ``"rpy"``: ``[roll, pitch, yaw]`` in radians, or
- ``"wxyz"``: a unit quaternion ``[w, x, y, z]``, **scalar first**, the same
  order as Drake's ``Quaternion(wxyz)`` (not Eigen's internal ``x, y, z, w``).
  Its norm must be within 0.001 of 1; such values are renormalized, so a
  hand-typed ``[0.7071, 0, 0, 0.7071]`` is fine.

.. code-block:: text

    {"xyz": [0.0, 0.75, 0.66], "rpy": [0.0, 0.0, -1.5708]}
    {"xyz": [0.0, 0.75, 0.66], "wxyz": [0.7071, 0.0, 0.0, -0.7071]}

Both describe the same pose. ``validate`` converts a quaternion to
roll-pitch-yaw with Drake's ``RollPitchYaw`` and always returns poses in the
``rpy`` form.

**Every angle is expressed in [-π, π].** Drake's conversion returns roll and
yaw in [-π, π] and pitch in [-π/2, π/2]. Because both input forms are checked
against the same ``rpy_min`` and ``rpy_max``, those bounds must lie inside
[-π, π]:

- A ``PoseParam`` whose ``rpy_min`` or ``rpy_max`` falls outside [-π, π] is
  refused when it is declared.
- An ``rpy`` value outside [-π, π] is refused, and the error gives the same
  angle in [-π, π]. For example, write a yaw of 4.71 rad (270°) as -1.57 rad.
- A range that wraps past π, such as yaw from 270° to 90° through 0°, cannot be
  written as one interval. Use the interval in [-π, π] that covers what you
  need.

Keep pitch bounds inside [-π/2, π/2] when you can. A pitch beyond ±π/2 given as
``rpy`` describes the same rotation as a quaternion whose converted pitch lies
inside ±π/2 with roll and yaw shifted by π, so it can pass as ``rpy`` and fail
as ``wxyz``.

Reading the schema
------------------

``Scenario.schema()`` returns JSON with one entry per field (``name``,
``kind``, ``default``, ``units``, ``description`` and the bounds) and the
``mutually_exclusive`` groups. A pose entry also lists
``"orientation_keys": ["rpy", "wxyz"]``.
