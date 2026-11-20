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
   * - ``VectorParam(None, min, max, units="")``
     - a non-empty list of any length
     - ``min`` to ``max`` (single numbers) for every entry
   * - ``PoseParam(xyz_min, xyz_max, rpy_min, rpy_max)``
     - ``{"xyz": [...], "rpy": [...]}`` or ``{"xyz": [...], "wxyz": [...]}``
     - per component; see below

``true`` and ``false``, NaN and infinity are never accepted as numbers.

The base production types (``KinematicMotionPlanningProduction``,
``OfflineDynamicMotionPlanningProduction`` and
``MotionPlanningAndGraspingProduction``) declare their start and goal fields
with loose bounds and ``VectorParam(None, ...)``, because the joint count
depends on the arm. A production should re-declare those fields with its
arm's joint count and limits and its own workspace.

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

Shared presets
--------------

``brom_drake.productions.scenario_presets`` holds the specs Brom's own
productions use, so productions with the same arm share the same numbers:

.. list-table::
   :header-rows: 1

   * - Name
     - What it allows
   * - ``UR10E_CONFIGURATION``
     - 6 joints, each within +/-2*pi except the elbow (+/-pi), from the UR10e URDF
   * - ``UR10E_TARGET_POSE``
     - x and y within +/-1.3 m, z from -0.2 to 1.5 m (the UR10e's reach)
   * - ``KINOVA_GEN3_CONFIGURATION``
     - any number of joints (6 or 7, following ``arm_variant``), each within +/-pi
   * - ``KINOVA_GEN3_TARGET_POSE``
     - x and y within +/-0.9 m, z from -0.2 to 1.2 m (the Gen3's reach)
   * - ``placed_object_pose(xyz_min, xyz_max, roll=0.0)``
     - an object that may turn about the vertical axis but not tip over
       (roll and pitch within 0.05 rad of ``roll`` and 0)
   * - ``SHELF_POSE``, ``TABLE_LENGTH``, ``TABLE_WIDTH``, ``TABLE_HEIGHT``
     - the shelf and table ranges of the ChemLab productions

Joint limits are copied by hand from the arm models, since reading them from
the plant would mean building it. Workspace boxes are in the world frame, whose
origin is at the arm's base.

To declare a Scenario for a new production, extend its production type's
Scenario and re-declare the start and goal fields with your arm's preset:

.. code-block:: python

    from dataclasses import dataclass

    from brom_drake.productions.scenario_presets import (
        UR10E_CONFIGURATION,
        UR10E_TARGET_POSE,
        placed_object_pose,
    )
    from brom_drake.productions.types.base import param


    class MyLab(KinematicMotionPlanningProduction):
        @dataclass
        class Scenario(KinematicMotionPlanningProduction.Scenario):
            start_configuration: list = param(UR10E_CONFIGURATION)
            start_pose: dict = param(UR10E_TARGET_POSE)
            goal_configuration: list = param(UR10E_CONFIGURATION)
            goal_pose: dict = param(UR10E_TARGET_POSE)
            box_pose: dict = param(
                placed_object_pose(xyz_min=(-1.0, -1.0, 0.0), xyz_max=(1.0, 1.0, 1.0))
            )

        def __init__(self, box_pose=None, **kwargs):
            ...

A test in Brom (``tests/productions/scenario_contract_test.py``) checks every
production with a Scenario: each field must be a constructor keyword argument
with the same default.

Reading the schema
------------------

``Scenario.schema()`` returns JSON with one entry per field (``name``,
``kind``, ``default``, ``units``, ``description`` and the bounds) and the
``mutually_exclusive`` groups. A pose entry also lists
``"orientation_keys": ["rpy", "wxyz"]``.

Building a production from scenario values
------------------------------------------

``Scenario.constructor_kwargs(values)`` validates the values and converts them
to constructor arguments: vectors become numpy arrays and poses become Drake
``RigidTransform`` objects.

.. code-block:: python

    kwargs = ShelfPlanning1.Scenario.constructor_kwargs(
        {"shelf_pose": {"xyz": [0.4, 1.2, 0.6], "rpy": [0.0, 0.0, 0.52]}}
    )
    production = ShelfPlanning1(**kwargs)
