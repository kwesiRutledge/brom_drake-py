"""
Typed scenario declarations for productions.

A production's ``Scenario`` lists the initial conditions that an outside caller
(for example, a Saphira fanout case) may change, with each parameter's type and
allowed range. Values arrive as plain JSON and are checked with
:meth:`Scenario.validate` before they are used.

Every ``Scenario`` field must be declared with :func:`param` and must have the
same name as a keyword argument of the production's constructor.

Wire format (plain JSON):

- ``float``: a number.
- ``vector``: a list of numbers.
- ``pose``: ``{"xyz": [x, y, z], "rpy": [roll, pitch, yaw]}`` in metres and radians,
  or ``{"xyz": [x, y, z], "wxyz": [w, x, y, z]}`` with a unit quaternion. Validated
  poses are always returned in the ``rpy`` form.
"""

import copy
import dataclasses
import math
from dataclasses import dataclass
from numbers import Real
from typing import Any, ClassVar, Dict, List, Optional, Sequence, Tuple, Union

import numpy as np

PARAM_METADATA_KEY = "brom_param"

# How far a quaternion's norm may be from 1 (hand-typed values such as
# [0.7071, 0, 0, 0.7071] are accepted and renormalized; [1, 0, 0, 1] is not).
QUATERNION_NORM_TOLERANCE = 1e-3

# Author documentation for the pose orientation convention; linked from errors.
SCENARIO_DOCS_URL = (
    "https://kwesirutledge.github.io/brom_drake-py/features/productions/scenarios.html"
)
POSE_CONVENTION_DOCS_URL = SCENARIO_DOCS_URL + "#pose-orientation-convention"

ANGLE_NAMES = ("roll", "pitch", "yaw")

# Slack for bounds written as rounded pi (e.g. 3.14159265) or np.pi arithmetic.
_ANGLE_TOLERANCE = 1e-9


def _angle_in_convention(a: float) -> bool:
    return -math.pi - _ANGLE_TOLERANCE <= a <= math.pi + _ANGLE_TOLERANCE


def _wrap_angle(a: float) -> float:
    """The same angle expressed in [-pi, pi)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _fmt_angles(angles: Sequence[float]) -> str:
    return "(" + ", ".join(f"{a:.6g}" for a in angles) + ")"


def _suggest_rpy_bounds(lo: Sequence[float], hi: Sequence[float]) -> str:
    """
    Explain how to rewrite rpy bounds that fall outside [-pi, pi]: wrap each
    range into [-pi, pi] when it stays one interval, or say that it cannot.
    """
    new_lo, new_hi, crossing = [], [], []
    for i, (a, b) in enumerate(zip(lo, hi)):
        if _angle_in_convention(a) and _angle_in_convention(b):
            new_lo.append(max(a, -math.pi))
            new_hi.append(min(b, math.pi))
        elif b - a >= 2.0 * math.pi:
            # Covers every angle anyway.
            new_lo.append(-math.pi)
            new_hi.append(math.pi)
        else:
            wa, wb = _wrap_angle(a), _wrap_angle(b)
            if wb == -math.pi:
                wb = math.pi
            if wa <= wb:
                new_lo.append(wa)
                new_hi.append(wb)
            else:
                crossing.append(
                    f"{ANGLE_NAMES[i]} from {a:.6g} to {b:.6g} is {wa:.6g} to "
                    f"{wb:.6g} in [-pi, pi], which crosses +/-pi"
                )
                new_lo.append(-math.pi)
                new_hi.append(math.pi)

    if crossing:
        return (
            "This range cannot be written as one interval inside [-pi, pi]: "
            + "; ".join(crossing)
            + ". Use the full range (-pi, pi) for that angle, or a narrower "
            "interval on one side of pi, for example "
            f"rpy_min={_fmt_angles(new_lo)}, rpy_max={_fmt_angles(new_hi)}."
        )
    return (
        "The same range expressed in [-pi, pi] is "
        f"rpy_min={_fmt_angles(new_lo)}, rpy_max={_fmt_angles(new_hi)}."
    )


class ScenarioError(ValueError):
    """
    *Description*

    Raised when scenario values (or a ``Scenario`` declaration) are invalid.

    *Attributes*

    problems: list[str]
        Every problem found, not just the first.
    """

    def __init__(self, problems: List[str]):
        self.problems = list(problems)
        super().__init__(
            "Invalid scenario values:\n" + "\n".join(f"- {p}" for p in self.problems)
        )


# ---------------------------------------------------------------------------
# Parameter specifications
# ---------------------------------------------------------------------------


def _is_number(value: Any) -> bool:
    # bool is a subclass of int (and so of Real); it is never a valid number here.
    return isinstance(value, Real) and not isinstance(value, (bool, np.bool_))


def _as_float_tuple(values: Sequence[float], name: str) -> Tuple[float, ...]:
    out = tuple(float(v) for v in values)
    if not all(math.isfinite(v) for v in out):
        raise ValueError(f"{name} must contain only finite numbers, got {values}")
    return out


def _check_bounds(lo: Sequence[float], hi: Sequence[float], name: str):
    for i, (a, b) in enumerate(zip(lo, hi)):
        if a > b:
            raise ValueError(f"{name}: min[{i}]={a} is greater than max[{i}]={b}")


def _check_number_list(
    value: Any,
    length: int,
    lo: Sequence[float],
    hi: Sequence[float],
    label: str,
    problems: List[str],
) -> Optional[List[float]]:
    """Check a list of numbers against a length and per-element bounds."""
    if not isinstance(value, (list, tuple, np.ndarray)) or (
        isinstance(value, np.ndarray) and value.ndim != 1
    ):
        problems.append(
            f"{label}: expected a list of {length} numbers, got {type(value).__name__}"
        )
        return None
    if len(value) != length:
        problems.append(f"{label}: expected {length} numbers, got {len(value)}")
        return None

    out = []
    ok = True
    for i, v in enumerate(value):
        if not _is_number(v):
            problems.append(f"{label}[{i}]: expected a number, got {type(v).__name__}")
            ok = False
            continue
        f = float(v)
        if not math.isfinite(f):
            problems.append(f"{label}[{i}]: expected a finite number, got {f}")
            ok = False
            continue
        if not (lo[i] <= f <= hi[i]):
            problems.append(
                f"{label}[{i}]: {f} is outside the allowed range [{lo[i]}, {hi[i]}]"
            )
            ok = False
            continue
        out.append(f)
    return out if ok else None


@dataclass(frozen=True)
class FloatParam:
    """
    *Description*

    A single real number in ``[min, max]``.
    """

    min: float
    max: float
    units: str = ""

    kind: ClassVar[str] = "float"

    def __post_init__(self):
        lo, hi = _as_float_tuple((self.min, self.max), "FloatParam bounds")
        object.__setattr__(self, "min", lo)
        object.__setattr__(self, "max", hi)
        _check_bounds((lo,), (hi,), "FloatParam")

    def to_python(self, value: float) -> float:
        """The constructor argument for a validated value."""
        return value

    def bounds(self) -> Dict[str, Any]:
        return {"min": self.min, "max": self.max}

    def check(self, value: Any, label: str, problems: List[str]) -> Optional[float]:
        if not _is_number(value):
            problems.append(f"{label}: expected a number, got {type(value).__name__}")
            return None
        f = float(value)
        if not math.isfinite(f):
            problems.append(f"{label}: expected a finite number, got {f}")
            return None
        if not (self.min <= f <= self.max):
            problems.append(
                f"{label}: {f} is outside the allowed range [{self.min}, {self.max}]"
            )
            return None
        return f


@dataclass(frozen=True)
class VectorParam:
    """
    *Description*

    A list of real numbers.

    - With a ``length``, ``min`` and ``max`` are sequences of that length and
      entry ``i`` must lie in ``[min[i], max[i]]``.
    - With ``length=None``, any non-empty length is accepted and ``min`` and
      ``max`` are single numbers that bound every entry. Base production
      classes use this for joint vectors, whose length depends on the arm;
      a concrete production should re-declare the field with its exact length
      and per-joint bounds.
    """

    length: Optional[int]
    min: Union[float, Tuple[float, ...]]
    max: Union[float, Tuple[float, ...]]
    units: str = ""

    kind: ClassVar[str] = "vector"

    def __post_init__(self):
        if self.length is None:
            if not (_is_number(self.min) and _is_number(self.max)):
                raise TypeError(
                    "VectorParam with length=None needs single numbers for min "
                    f"and max (they bound every entry), got {self.min!r} and "
                    f"{self.max!r}"
                )
            lo, hi = _as_float_tuple((self.min, self.max), "VectorParam bounds")
            _check_bounds((lo,), (hi,), "VectorParam")
            object.__setattr__(self, "min", lo)
            object.__setattr__(self, "max", hi)
            return

        if isinstance(self.length, bool) or not isinstance(self.length, int):
            raise TypeError(
                f"VectorParam length must be an int or None, got {self.length!r}"
            )
        if self.length < 1:
            raise ValueError(f"VectorParam length must be positive, got {self.length}")
        lo = _as_float_tuple(self.min, "VectorParam min")
        hi = _as_float_tuple(self.max, "VectorParam max")
        if len(lo) != self.length or len(hi) != self.length:
            raise ValueError(
                f"VectorParam min and max must have length {self.length}, "
                f"got {len(lo)} and {len(hi)}"
            )
        _check_bounds(lo, hi, "VectorParam")
        object.__setattr__(self, "min", lo)
        object.__setattr__(self, "max", hi)

    def to_python(self, value: List[float]) -> np.ndarray:
        """The constructor argument for a validated value: a numpy array."""
        return np.array(value, dtype=float)

    def bounds(self) -> Dict[str, Any]:
        if self.length is None:
            return {"length": None, "min": self.min, "max": self.max}
        return {"length": self.length, "min": list(self.min), "max": list(self.max)}

    def check(
        self, value: Any, label: str, problems: List[str]
    ) -> Optional[List[float]]:
        if self.length is None:
            if not isinstance(value, (list, tuple, np.ndarray)):
                problems.append(
                    f"{label}: expected a list of numbers, got {type(value).__name__}"
                )
                return None
            if len(value) == 0:
                problems.append(f"{label}: expected a non-empty list of numbers")
                return None
            n = len(value)
            return _check_number_list(
                value, n, (self.min,) * n, (self.max,) * n, label, problems
            )
        return _check_number_list(
            value, self.length, self.min, self.max, label, problems
        )


@dataclass(frozen=True)
class PoseParam:
    """
    *Description*

    A rigid pose with per-component bounds on position and roll-pitch-yaw.

    The position is ``"xyz"`` in metres. The orientation is given as exactly
    one of:

    - ``"rpy"``: ``[roll, pitch, yaw]`` in radians, or
    - ``"wxyz"``: a unit quaternion ``[w, x, y, z]`` (scalar first, as in
      ``pydrake.common.eigen_geometry.Quaternion``).

    A quaternion is converted to roll-pitch-yaw with Drake's ``RollPitchYaw``
    and checked against the ``rpy`` bounds, so validated poses are always
    ``{"xyz": [...], "rpy": [...]}``.

    **Orientation convention.** Every angle is expressed in ``[-pi, pi]``, so
    ``rpy_min`` and ``rpy_max`` must lie inside ``[-pi, pi]``. A quaternion's
    converted angles always do (Drake also keeps pitch in ``[-pi/2, pi/2]``),
    which is what lets the same bounds apply to both input forms. See
    :data:`POSE_CONVENTION_DOCS_URL`.
    """

    xyz_min: Tuple[float, float, float]
    xyz_max: Tuple[float, float, float]
    rpy_min: Tuple[float, float, float] = (-math.pi, -math.pi, -math.pi)
    rpy_max: Tuple[float, float, float] = (math.pi, math.pi, math.pi)

    kind: ClassVar[str] = "pose"
    units: ClassVar[str] = "m, rad"
    orientation_keys: ClassVar[Tuple[str, ...]] = ("rpy", "wxyz")

    def __post_init__(self):
        for name in ("xyz_min", "xyz_max", "rpy_min", "rpy_max"):
            values = _as_float_tuple(getattr(self, name), f"PoseParam {name}")
            if len(values) != 3:
                raise ValueError(
                    f"PoseParam {name} must have 3 entries, got {len(values)}"
                )
            object.__setattr__(self, name, values)
        _check_bounds(self.xyz_min, self.xyz_max, "PoseParam xyz")
        _check_bounds(self.rpy_min, self.rpy_max, "PoseParam rpy")

        # Orientation convention: rpy bounds must lie inside [-pi, pi].
        outside = [
            f"{bound_name}[{i}] ({ANGLE_NAMES[i]}) = {a!r}"
            for bound_name in ("rpy_min", "rpy_max")
            for i, a in enumerate(getattr(self, bound_name))
            if not _angle_in_convention(a)
        ]
        if outside:
            raise ValueError(
                "PoseParam rpy bounds must lie inside [-pi, pi], but "
                + ", ".join(outside)
                + (" are" if len(outside) > 1 else " is")
                + " outside it.\n"
                "Brom expresses every pose angle in [-pi, pi]. A quaternion "
                '("wxyz") input is converted with Drake\'s RollPitchYaw, which '
                "always returns roll and yaw in [-pi, pi] and pitch in "
                "[-pi/2, pi/2], so bounds outside [-pi, pi] would accept a "
                "rotation given as rpy but refuse the same rotation given as a "
                "quaternion.\n"
                + _suggest_rpy_bounds(self.rpy_min, self.rpy_max)
                + f"\nSee {POSE_CONVENTION_DOCS_URL}"
            )

    def to_python(self, value: Dict[str, List[float]]):
        """
        The constructor argument for a validated pose: a Drake
        ``RigidTransform(RollPitchYaw(r, p, y), [x, y, z])``.
        """
        from pydrake.math import RigidTransform, RollPitchYaw

        return RigidTransform(RollPitchYaw(value["rpy"]), np.array(value["xyz"]))

    def bounds(self) -> Dict[str, Any]:
        return {
            "xyz_min": list(self.xyz_min),
            "xyz_max": list(self.xyz_max),
            "rpy_min": list(self.rpy_min),
            "rpy_max": list(self.rpy_max),
            "orientation_keys": list(self.orientation_keys),
        }

    def check(
        self, value: Any, label: str, problems: List[str]
    ) -> Optional[Dict[str, List[float]]]:
        if not isinstance(value, dict):
            problems.append(
                f'{label}: expected a pose {{"xyz": [x, y, z], "rpy": [r, p, y]}} '
                f'or {{"xyz": [x, y, z], "wxyz": [w, x, y, z]}}, got '
                f"{type(value).__name__}. See {POSE_CONVENTION_DOCS_URL}"
            )
            return None

        n_problems = len(problems)
        allowed = {"xyz", *self.orientation_keys}
        for key in sorted(set(value) - allowed, key=str):
            problems.append(
                f'{label}: unknown pose key {key!r}. A pose is "xyz" plus '
                'exactly one of "rpy" ([roll, pitch, yaw] in radians) or "wxyz" '
                f"(a unit quaternion, scalar first). See {POSE_CONVENTION_DOCS_URL}"
            )
        if "xyz" not in value:
            problems.append(f"{label}: missing pose key 'xyz' ([x, y, z] in metres)")
        given_orientations = [k for k in self.orientation_keys if k in value]
        if not given_orientations:
            problems.append(
                f'{label}: missing orientation. Give exactly one of "rpy" '
                '([roll, pitch, yaw] in radians) or "wxyz" (a unit quaternion '
                f"[w, x, y, z]). See {POSE_CONVENTION_DOCS_URL}"
            )
        elif len(given_orientations) > 1:
            problems.append(
                f'{label}: both "rpy" and "wxyz" were given. Give exactly one '
                "orientation, so there is no doubt which one is used. "
                f"See {POSE_CONVENTION_DOCS_URL}"
            )

        xyz = rpy = None
        if "xyz" in value:
            xyz = _check_number_list(
                value["xyz"], 3, self.xyz_min, self.xyz_max, f"{label}.xyz", problems
            )
        if given_orientations == ["rpy"]:
            rpy = self._check_rpy(value["rpy"], f"{label}.rpy", problems)
        elif given_orientations == ["wxyz"]:
            rpy = self._check_quaternion(value["wxyz"], f"{label}.wxyz", problems)

        if len(problems) > n_problems:
            return None
        return {"xyz": xyz, "rpy": rpy}

    def _check_rpy(
        self, value: Any, label: str, problems: List[str]
    ) -> Optional[List[float]]:
        """Check ``[roll, pitch, yaw]`` against the convention and the bounds."""
        rpy = _check_number_list(
            value, 3, (-math.inf,) * 3, (math.inf,) * 3, label, problems
        )
        if rpy is None:
            return None

        ok = True
        for i, a in enumerate(rpy):
            if not _angle_in_convention(a):
                problems.append(
                    f"{label}[{i}]: {ANGLE_NAMES[i]} = {a!r} rad is outside "
                    "[-pi, pi]. Brom expresses every pose angle in [-pi, pi] so "
                    "that rpy and quaternion inputs are checked against the same "
                    f"bounds; the same angle in [-pi, pi] is {_wrap_angle(a):.6g} "
                    f"rad. See {POSE_CONVENTION_DOCS_URL}"
                )
                ok = False
            elif not (self.rpy_min[i] <= a <= self.rpy_max[i]):
                problems.append(
                    f"{label}[{i}]: {ANGLE_NAMES[i]} = {a!r} rad is outside the "
                    f"allowed range [{self.rpy_min[i]!r}, {self.rpy_max[i]!r}]"
                )
                ok = False
        return rpy if ok else None

    def _check_quaternion(
        self, value: Any, label: str, problems: List[str]
    ) -> Optional[List[float]]:
        """
        Check a ``[w, x, y, z]`` unit quaternion (Drake's order, scalar first)
        and convert it to roll-pitch-yaw, which is then checked against the
        ``rpy`` bounds.
        """
        wxyz = _check_number_list(
            value, 4, (-math.inf,) * 4, (math.inf,) * 4, label, problems
        )
        if wxyz is None:
            return None

        norm = math.sqrt(sum(c * c for c in wxyz))
        if abs(norm - 1.0) > QUATERNION_NORM_TOLERANCE:
            problems.append(
                f"{label}: {wxyz} is not a unit quaternion (norm {norm:.6g}; it "
                f"must be within {QUATERNION_NORM_TOLERANCE} of 1). The order is "
                "[w, x, y, z] with the scalar w first, as in Drake's Quaternion "
                "(not Eigen's internal x, y, z, w). See "
                f"{POSE_CONVENTION_DOCS_URL}"
            )
            return None

        # Imported here so the schema can be read without loading Drake.
        from pydrake.common.eigen_geometry import Quaternion
        from pydrake.math import RollPitchYaw

        q = Quaternion(np.array(wxyz) / norm)
        # "+ 0.0" turns -0.0 into 0.0 so the JSON reads cleanly.
        rpy = [float(a) + 0.0 for a in RollPitchYaw(q).vector()]
        outside = [
            f"{ANGLE_NAMES[i]} = {a:.6g} rad (allowed "
            f"[{self.rpy_min[i]:.6g}, {self.rpy_max[i]:.6g}])"
            for i, a in enumerate(rpy)
            if not (self.rpy_min[i] <= a <= self.rpy_max[i])
        ]
        if outside:
            problems.append(
                f"{label}: the quaternion {wxyz} is the rotation rpy = "
                f"{_fmt_angles(rpy)}, and " + "; ".join(outside) + " is outside "
                "the allowed range. Quaternions are converted with Drake's "
                "RollPitchYaw, which gives roll and yaw in [-pi, pi] and pitch in "
                "[-pi/2, pi/2], then checked against the same rpy bounds as an "
                f"rpy input. See {POSE_CONVENTION_DOCS_URL}"
            )
            return None
        return rpy


ParamSpec = Union[FloatParam, VectorParam, PoseParam]


def param(spec: ParamSpec, default: Any = None, description: str = "") -> Any:
    """
    *Description*

    Declare a ``Scenario`` field. Use it as the field's default value::

        @dataclass
        class Scenario(BaseScenario):
            table_height: float = param(FloatParam(0.05, 0.5, "m"), default=0.1)

    *Parameters*

    spec: FloatParam | VectorParam | PoseParam
        The parameter's type and allowed range.

    default: Any, optional
        The value used when a case does not override this field, in the wire
        format. ``None`` (the default) means "use the production constructor's
        own default".

    description: str, optional
        A short human-readable description, included in :meth:`Scenario.schema`.
    """
    if not isinstance(spec, (FloatParam, VectorParam, PoseParam)):
        raise TypeError(
            f"param() expects a FloatParam, VectorParam or PoseParam, got {spec!r}"
        )
    metadata = {PARAM_METADATA_KEY: spec, "description": description}
    if isinstance(default, (dict, list, np.ndarray)):
        # Mutable defaults (poses, vectors) need a factory in a dataclass.
        frozen_default = copy.deepcopy(
            default.tolist() if isinstance(default, np.ndarray) else default
        )
        return dataclasses.field(
            default_factory=lambda: copy.deepcopy(frozen_default), metadata=metadata
        )
    return dataclasses.field(default=default, metadata=metadata)


# ---------------------------------------------------------------------------
# Scenario base class
# ---------------------------------------------------------------------------


@dataclass
class Scenario:
    """
    *Description*

    Base class for a production's scenario: the initial conditions a case may
    change. Subclasses must be decorated with ``@dataclass`` and declare every
    field with :func:`param`.

    *Class attributes*

    mutually_exclusive: tuple[tuple[str, ...], ...]
        Groups of field names of which at most one may be set (not ``None``)
        in a single call to :meth:`validate`, for example
        ``(("start_configuration", "start_pose"),)``.
    """

    mutually_exclusive: ClassVar[Tuple[Tuple[str, ...], ...]] = ()

    @classmethod
    def param_fields(cls) -> List[Tuple[dataclasses.Field, ParamSpec]]:
        """
        *Description*

        Return each field with its parameter spec, after checking that the
        class is declared correctly (raises ``TypeError`` otherwise).
        """
        if "__dataclass_fields__" not in cls.__dict__:
            raise TypeError(
                f"{cls.__qualname__} must be decorated with @dataclass "
                "to declare scenario fields"
            )

        out = []
        problems = []
        for f in dataclasses.fields(cls):
            spec = f.metadata.get(PARAM_METADATA_KEY)
            if spec is None:
                problems.append(f"field {f.name!r} must be declared with param(...)")
                continue
            default = _field_default(f)
            if default is not None:
                spec.check(default, f"default of {f.name!r}", problems)
            out.append((f, spec))

        names = {f.name for f, _ in out}
        for group in cls.mutually_exclusive:
            for name in group:
                if name not in names:
                    problems.append(f"mutually_exclusive names unknown field {name!r}")

        if problems:
            raise TypeError(
                f"Invalid Scenario declaration {cls.__qualname__}: "
                + "; ".join(problems)
            )
        return out

    @classmethod
    def schema(cls) -> Dict[str, Any]:
        """
        *Description*

        Describe this scenario as JSON-serializable data.

        *Returns*

        schema: dict
            ``{"fields": [...], "mutually_exclusive": [[...], ...]}``, with one
            entry per field: ``name``, ``kind`` (``"float"``, ``"vector"`` or
            ``"pose"``), ``default``, ``units``, ``description`` and the
            kind's bounds (``min``/``max``, plus ``length`` for a vector;
            ``xyz_min``/``xyz_max``/``rpy_min``/``rpy_max`` for a pose).
        """
        entries = []
        for f, spec in cls.param_fields():
            entry = {
                "name": f.name,
                "kind": spec.kind,
                "default": _field_default(f),
                "units": spec.units,
                "description": f.metadata.get("description", ""),
            }
            entry.update(spec.bounds())
            entries.append(entry)
        return {
            "fields": entries,
            "mutually_exclusive": [list(g) for g in cls.mutually_exclusive],
        }

    @classmethod
    def validate(cls, values: Dict[str, Any]) -> Dict[str, Any]:
        """
        *Description*

        Check override values against this scenario's declaration.

        *Parameters*

        values: dict
            Field name to value, in the wire format. ``None`` is accepted only
            for a field whose default is ``None``.

        *Returns*

        overrides: dict
            Only the keys that were given, normalized to plain JSON (floats as
            ``float``, vectors as lists of floats, poses with both ``xyz`` and
            ``rpy``).

        *Raises*

        ScenarioError
            With every problem found in ``problems``.
        """
        if not isinstance(values, dict):
            raise ScenarioError(
                [f"expected a dict of scenario values, got {type(values).__name__}"]
            )

        specs = {f.name: (f, spec) for f, spec in cls.param_fields()}
        problems: List[str] = []
        overrides: Dict[str, Any] = {}

        for name in values:
            if name not in specs:
                problems.append(
                    f"unknown parameter {name!r} for {cls.__qualname__}; "
                    f"known parameters: {sorted(specs)}"
                )

        for name, (f, spec) in specs.items():
            if name not in values:
                continue
            value = values[name]
            if value is None:
                if _field_default(f) is None:
                    overrides[name] = None
                else:
                    problems.append(f"{name}: None is not allowed for this parameter")
                continue
            normalized = spec.check(value, name, problems)
            if normalized is not None:
                overrides[name] = normalized

        for group in cls.mutually_exclusive:
            given = [n for n in group if values.get(n) is not None]
            if len(given) > 1:
                problems.append(f"at most one of {list(group)} may be set, got {given}")

        if problems:
            raise ScenarioError(problems)
        return overrides

    @classmethod
    def constructor_kwargs(cls, values: Dict[str, Any]) -> Dict[str, Any]:
        """
        *Description*

        Validate ``values`` and convert them to the production constructor's
        keyword arguments: floats stay floats, vectors become numpy arrays and
        poses become ``RigidTransform`` objects. ``None`` stays ``None``.

        Use it to build a production from scenario values in Python::

            kwargs = ChemLab1.Scenario.constructor_kwargs({"table_height": 0.2})
            production = ChemLab1(**kwargs)

        *Raises*

        ScenarioError
            As :meth:`validate` does.
        """
        overrides = cls.validate(values)
        specs = {f.name: spec for f, spec in cls.param_fields()}
        return {
            name: None if value is None else specs[name].to_python(value)
            for name, value in overrides.items()
        }


def _field_default(f: dataclasses.Field) -> Any:
    if f.default is not dataclasses.MISSING:
        return f.default
    if f.default_factory is not dataclasses.MISSING:
        return f.default_factory()
    return None
