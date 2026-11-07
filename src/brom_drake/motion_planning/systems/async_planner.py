import concurrent.futures
import itertools
import logging
from dataclasses import dataclass
from typing import Callable, Dict, Optional, Tuple

import numpy as np
from pydrake.common.value import AbstractValue
from pydrake.geometry import SceneGraph
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import ModelInstanceIndex
from pydrake.systems.framework import Context, EventStatus, State

# Internal Imports
from brom_drake.utils.constants import MotionPlan
from .prototypical_planner import PrototypicalPlannerSystem

# Period used for events that should fire exactly once (at their offset).
_ONE_SHOT_PERIOD_S = 1e12


@dataclass
class AsyncPlanningConfig:
    """
    **Description**

    Settings for :py:class:`AsyncPlannerSystem`.

    **Parameters**

    plan_start_time: float, optional
        The simulation time T (seconds) at which planning starts. The planner
        plans from the robot's configuration at this time. Default is 0.0.

    planning_latency: float, optional
        How much simulation time (seconds) planning takes. The plan is released
        at exactly T + planning_latency, regardless of how long the planner
        actually ran on the computer, so simulations are repeatable. If the
        planner is still running at that time, the simulation waits for it.
        Default is 0.5.

    drift_tolerance: float, optional
        While the plan is being computed, the robot is held at its configuration
        from time T. If, when the plan is released, the robot has moved further
        than this from that configuration (2-norm, in the robot's position units),
        a warning is logged, because the plan starts from where the robot was,
        not where it is. Default is 0.05.

    max_wall_wait_s: float, optional
        The longest the simulation will wait (in real seconds) for the planner
        at T + planning_latency before raising an error. Default is None (wait
        as long as it takes).
    """

    plan_start_time: float = 0.0
    planning_latency: float = 0.5
    drift_tolerance: float = 0.05
    max_wall_wait_s: Optional[float] = None

    def __post_init__(self):
        if self.plan_start_time < 0.0:
            raise ValueError(
                f"plan_start_time must be non-negative; received {self.plan_start_time}."
            )
        if self.planning_latency <= 0.0:
            raise ValueError(
                f"planning_latency must be positive; received {self.planning_latency}."
            )
        if self.drift_tolerance < 0.0:
            raise ValueError(
                f"drift_tolerance must be non-negative; received {self.drift_tolerance}."
            )

    @property
    def plan_release_time(self) -> float:
        """The simulation time at which the plan becomes available."""
        return self.plan_start_time + self.planning_latency


class _Status:
    kIdle = "idle"  # Before plan_start_time
    kPlanning = "planning"  # Between plan_start_time and the release time
    kReady = "ready"  # The plan has been released


class AsyncPlannerSystem(PrototypicalPlannerSystem):
    """
    **Description**

    A planner that plans *while the simulation keeps running*, modelling a
    real robot whose planner takes time to respond.

    - At simulation time ``T = config.plan_start_time``, the planner records the
      robot's configuration and the goal, takes a private copy of the whole
      simulation context (a snapshot of the world at time T), and starts the
      planning algorithm on a background thread using that copy for collision
      checks. The live simulation is never modified by the planner.
    - Until the plan is released, the ``motion_plan`` output is a one-row plan
      containing the configuration to hold, and ``plan_is_ready`` is False. The
      hold command does not change at T: the robot keeps being commanded to the
      start configuration, as a real robot keeps its last command while waiting.
    - At exactly ``T + config.planning_latency`` the plan is released. If the
      background planner has not finished, the simulation waits for it. The
      released plan starts from the robot's *measured* configuration at time T.

    Plan progress is stored in the system's state (not on the Python object),
    so the same diagram can be simulated more than once.

    It has the same ports as :py:class:`PrototypicalPlannerSystem`, so it can
    fill the same roles in productions.

    **Parameters**

    plant: MultibodyPlant
        The plant that contains the robot.

    scene_graph: SceneGraph
        The scene graph used for collision checking.

    planning_algorithm: Callable
        The planning algorithm: ``(q_start, q_goal, collision_check_fcn) -> plan``.
        It runs on a background thread and must only use the collision check
        function it is given to query the world.

    robot_model_idx: ModelInstanceIndex
        The robot's model instance.

    config: AsyncPlanningConfig, optional
        When to plan and how long planning takes. Default is AsyncPlanningConfig().

    .. note::

        Like PrototypicalPlannerSystem, this system needs the simulation's root
        context; call ``set_internal_root_context(diagram_context)`` after
        building (the motion planning productions do this for you).
    """

    def __init__(
        self,
        plant: MultibodyPlant,
        scene_graph: SceneGraph,
        planning_algorithm: Callable[
            [np.ndarray, np.ndarray, Callable[[np.ndarray], bool]],
            Tuple[MotionPlan, int],
        ],
        robot_model_idx: ModelInstanceIndex = None,
        config: Optional[AsyncPlanningConfig] = None,
        **kwargs,
    ):
        # Must exist before the base constructor declares the output ports
        self.config = config if config is not None else AsyncPlanningConfig()
        self._executor: Optional[concurrent.futures.ThreadPoolExecutor] = None
        self._jobs: Dict[int, concurrent.futures.Future] = {}
        self._job_ids = itertools.count()
        self._logger = logging.getLogger(__name__)

        super().__init__(
            plant,
            scene_graph,
            planning_algorithm,
            robot_model_idx=robot_model_idx,
            **kwargs,
        )

        # Planning progress lives in the context, not on this object
        self._state_index = self.DeclareAbstractState(
            AbstractValue.Make(self._initial_state())
        )

        # Start planning at T, release the plan at T + latency
        self.DeclarePeriodicUnrestrictedUpdateEvent(
            period_sec=_ONE_SHOT_PERIOD_S,
            offset_sec=self.config.plan_start_time,
            update=self._start_planning,
        )
        self.DeclarePeriodicUnrestrictedUpdateEvent(
            period_sec=_ONE_SHOT_PERIOD_S,
            offset_sec=self.config.plan_release_time,
            update=self._release_plan,
        )

    @staticmethod
    def _initial_state() -> dict:
        return {
            "status": _Status.kIdle,
            "job_id": -1,
            "q_at_T": None,
            "q_hold": None,
            "plan": None,
        }

    # ------------------------------------------------------------------ ports

    def create_output_ports(self):
        """
        Declares ``motion_plan`` and ``plan_is_ready``, both computed from the
        system's state (evaluating them never triggers planning).
        """
        sample_plan = np.zeros((0, self.n_actuated_dof))
        self.DeclareAbstractOutputPort(
            "motion_plan",
            lambda: AbstractValue.Make(sample_plan),
            self._calc_motion_plan,
        )
        self.DeclareAbstractOutputPort(
            "plan_is_ready",
            lambda: AbstractValue.Make(False),
            self._calc_plan_is_ready,
        )

    def _planning_state(self, context: Context) -> dict:
        return context.get_abstract_state(self._state_index).get_value()

    def _calc_motion_plan(self, context: Context, output: AbstractValue):
        state = self._planning_state(context)
        if state["status"] == _Status.kReady:
            output.set_value(state["plan"])
        elif state["status"] == _Status.kPlanning:
            output.set_value(np.atleast_2d(state["q_hold"]))
        else:
            q_start = self.GetInputPort("start_configuration").Eval(context)
            output.set_value(np.atleast_2d(q_start))

    def _calc_plan_is_ready(self, context: Context, output: AbstractValue):
        output.set_value(self._planning_state(context)["status"] == _Status.kReady)

    # ----------------------------------------------------------------- events

    def _robot_model(self, context: Context) -> ModelInstanceIndex:
        if self.robot_model_idx is None:
            self.robot_model_idx = (
                self.GetInputPort("robot_model_index").EvalAbstract(context).get_value()
            )
        return self.robot_model_idx

    def _start_planning(self, context: Context, state: State) -> EventStatus:
        current = self._planning_state(context)
        if current["status"] != _Status.kIdle:
            return EventStatus.DidNothing()

        if self.root_context is None:
            raise RuntimeError(
                "AsyncPlannerSystem cannot start planning because its root context is "
                "not set. Call set_internal_root_context(diagram_context) after building."
            )

        robot = self._robot_model(context)

        # Snapshot of the world at time T; the planner only ever touches this copy
        planning_context = self.root_context.Clone()
        q_at_T = np.array(
            self.plant.GetPositions(
                self.plant.GetMyContextFromRoot(planning_context), robot
            )
        )
        q_goal = np.array(self.GetInputPort("goal_configuration").Eval(context))

        # Keep commanding what was commanded at T while planning. Holding the
        # *measured* configuration instead would make a controller with a
        # steady-state error (e.g., PD under gravity) sag further.
        q_hold = np.array(self.GetInputPort("start_configuration").Eval(context))

        def collision_check(q: np.ndarray) -> bool:
            return self.check_collision_in_config_with_context(q, planning_context)

        def run_planner() -> np.ndarray:
            result = self.planning_algorithm(q_at_T, q_goal, collision_check)
            return self.planning_result_to_array(result)

        if self._executor is None:
            self._executor = concurrent.futures.ThreadPoolExecutor(
                max_workers=1, thread_name_prefix="brom-async-planner"
            )
        job_id = next(self._job_ids)
        self._jobs[job_id] = self._executor.submit(run_planner)

        state.get_mutable_abstract_state(self._state_index).set_value(
            {
                "status": _Status.kPlanning,
                "job_id": job_id,
                "q_at_T": q_at_T,
                "q_hold": q_hold,
                "plan": None,
            }
        )
        return EventStatus.Succeeded()

    def _release_plan(self, context: Context, state: State) -> EventStatus:
        current = self._planning_state(context)
        if current["status"] != _Status.kPlanning:
            return EventStatus.DidNothing()

        future = self._jobs.pop(current["job_id"])
        try:
            plan = future.result(timeout=self.config.max_wall_wait_s)
        except concurrent.futures.TimeoutError as e:
            raise RuntimeError(
                f"The planner did not finish within max_wall_wait_s="
                f"{self.config.max_wall_wait_s} s of real time. Increase "
                "max_wall_wait_s or set it to None to wait as long as needed."
            ) from e
        except Exception as e:
            raise RuntimeError(
                f"The planner started at t={self.config.plan_start_time} s failed: {e}"
            ) from e

        self._warn_if_robot_drifted(context, current["q_at_T"])

        state.get_mutable_abstract_state(self._state_index).set_value(
            {
                "status": _Status.kReady,
                "job_id": current["job_id"],
                "q_at_T": current["q_at_T"],
                "q_hold": current["q_hold"],
                "plan": plan,
            }
        )
        return EventStatus.Succeeded()

    def _warn_if_robot_drifted(self, context: Context, q_at_T: np.ndarray):
        if self.root_context is None:
            return
        q_now = self.plant.GetPositions(
            self.plant.GetMyContextFromRoot(self.root_context),
            self._robot_model(context),
        )
        drift = float(np.linalg.norm(np.asarray(q_now) - q_at_T))
        if drift > self.config.drift_tolerance:
            self._logger.warning(
                f"The robot moved {drift:.3f} while the plan was computed "
                f"(t={self.config.plan_start_time} s to t={self.config.plan_release_time} s), "
                f"more than drift_tolerance={self.config.drift_tolerance}. The plan "
                "starts from the robot's configuration at the start of planning."
            )

    def get_planning_status(self, root_context: Context) -> str:
        """
        **Description**

        Returns "idle", "planning" or "ready" for the given root (diagram) context,
        e.g. ``simulator.get_context()``.
        """
        return self._planning_state(self.GetMyContextFromRoot(root_context))["status"]

    def __del__(self):
        try:
            if self._executor is not None:
                self._executor.shutdown(wait=False, cancel_futures=True)
        except Exception:
            pass
