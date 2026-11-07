import threading
import unittest

import numpy as np
from pydrake.systems.analysis import Simulator

# Internal imports
from brom_drake.all import AsyncPlannerSystem, AsyncPlanningConfig
from brom_drake.motion_planning.algorithms.rrt.bidirectional_connect import (
    BidirectionalRRTConnectPlanner,
    BidirectionalRRTConnectPlannerConfig,
)
from brom_drake.productions.motion_planning.offline import ShelfPlanning1
from brom_drake.productions.motion_planning.offline.dynamic.chem_lab2 import ChemLab2

# Long enough for the test's own steps, short enough not to hang CI on a bug
_THREAD_TIMEOUT_S = 30.0


def _rrt(production):
    return BidirectionalRRTConnectPlanner(
        production.arm,
        production.plant,
        production.scene_graph,
        config=BidirectionalRRTConnectPlannerConfig(steering_step_size=0.2),
    )


def _build(production, planning_algorithm, config: AsyncPlanningConfig):
    diagram, diagram_context = production.easy_cast_and_build(
        planning_algorithm,
        with_watcher=False,
        async_planning=config,
    )
    planner = next(s for s in diagram.GetSystems() if isinstance(s, AsyncPlannerSystem))
    simulator = Simulator(diagram, diagram_context)
    simulator.Initialize()
    return simulator, planner


def _arm_positions(production, simulator) -> np.ndarray:
    plant = production.plant
    return np.array(
        plant.GetPositions(
            plant.GetMyContextFromRoot(simulator.get_context()), production.arm
        )
    )


class AsyncPlanningConfigTest(unittest.TestCase):
    def test_release_time1(self):
        config = AsyncPlanningConfig(plan_start_time=0.5, planning_latency=1.25)
        self.assertAlmostEqual(config.plan_release_time, 1.75)

    def test_validation1(self):
        with self.assertRaises(ValueError):
            AsyncPlanningConfig(plan_start_time=-1.0)
        with self.assertRaises(ValueError):
            AsyncPlanningConfig(planning_latency=0.0)
        with self.assertRaises(ValueError):
            AsyncPlanningConfig(drift_tolerance=-0.1)


class AsyncPlannerKinematicTest(unittest.TestCase):
    def test_plans_while_simulation_runs1(self):
        """
        Description
        -----------
        The planner blocks until the test releases it. The simulation must keep
        advancing (with the arm held still) while the planner is blocked, then
        release the plan at exactly plan_start_time + planning_latency. The plan
        must start from the arm's configuration at plan_start_time, and executing
        it must reach the goal.
        """
        production = ShelfPlanning1(meshcat_port_number=None)
        rrt = _rrt(production)
        planner_may_finish = threading.Event()
        seen = {}

        def blocking_plan(q_start, q_goal, collision_check):
            seen["q_start"] = np.array(q_start)
            seen["thread"] = threading.current_thread().name
            result = rrt.plan(q_start, q_goal, collision_check)
            planner_may_finish.wait(timeout=_THREAD_TIMEOUT_S)
            return result

        config = AsyncPlanningConfig(plan_start_time=0.5, planning_latency=1.0)
        simulator, planner = _build(production, blocking_plan, config)

        # Before T: idle
        simulator.AdvanceTo(0.4)
        self.assertEqual(planner.get_planning_status(simulator.get_context()), "idle")

        # During planning: the simulation advances even though the planner is blocked
        simulator.AdvanceTo(1.45)
        self.assertEqual(
            planner.get_planning_status(simulator.get_context()), "planning"
        )
        self.assertFalse(planner_may_finish.is_set())
        self.assertTrue(seen["thread"].startswith("brom-async-planner"))
        np.testing.assert_allclose(
            _arm_positions(production, simulator), seen["q_start"], atol=1e-9
        )

        # Release
        planner_may_finish.set()
        simulator.AdvanceTo(1.6)
        self.assertEqual(planner.get_planning_status(simulator.get_context()), "ready")

        dispenser = production.plan_dispenser
        self.assertAlmostEqual(dispenser.t0, config.plan_release_time, delta=2e-3)
        np.testing.assert_allclose(
            dispenser.planned_trajectory.value(0.0).flatten(),
            seen["q_start"],
            atol=1e-12,
        )

        # Execute the plan
        simulator.AdvanceTo(
            dispenser.t0 + dispenser.planned_trajectory.end_time() + 0.5
        )
        np.testing.assert_allclose(
            _arm_positions(production, simulator),
            production.goal_configuration,
            atol=1e-3,
        )

    def test_planner_error1(self):
        """
        Description
        -----------
        A planner that fails should raise a clear error when the plan is due.
        """
        production = ShelfPlanning1(meshcat_port_number=None)

        def failing_plan(q_start, q_goal, collision_check):
            raise ValueError("no path found")

        config = AsyncPlanningConfig(plan_start_time=0.0, planning_latency=0.2)
        simulator, _ = _build(production, failing_plan, config)
        with self.assertRaises(RuntimeError) as ctx:
            simulator.AdvanceTo(0.5)
        self.assertIn("no path found", str(ctx.exception))

    def test_max_wall_wait1(self):
        """
        Description
        -----------
        If the planner is still running max_wall_wait_s after the plan is due,
        the simulation should stop with an error instead of hanging.
        """
        production = ShelfPlanning1(meshcat_port_number=None)
        never = threading.Event()

        def stuck_plan(q_start, q_goal, collision_check):
            never.wait(timeout=_THREAD_TIMEOUT_S)
            return np.array([q_start, q_goal])

        config = AsyncPlanningConfig(
            plan_start_time=0.0, planning_latency=0.2, max_wall_wait_s=0.2
        )
        simulator, _ = _build(production, stuck_plan, config)
        try:
            with self.assertRaises(RuntimeError) as ctx:
                simulator.AdvanceTo(0.5)
            self.assertIn("max_wall_wait_s", str(ctx.exception))
        finally:
            never.set()

    def test_live_context_untouched_by_planner1(self):
        """
        Description
        -----------
        The planner's collision checks must use a private copy of the world, so
        the live simulation's arm configuration never changes during planning
        even when the collision checker visits many configurations.
        """
        production = ShelfPlanning1(meshcat_port_number=None)
        rng = np.random.default_rng(0)
        checks = {"n": 0}

        def probing_plan(q_start, q_goal, collision_check):
            for _ in range(200):
                collision_check(rng.uniform(-np.pi, np.pi, size=q_start.shape))
                checks["n"] += 1
            return np.array([q_start, q_goal])

        config = AsyncPlanningConfig(plan_start_time=0.1, planning_latency=0.5)
        simulator, _ = _build(production, probing_plan, config)
        simulator.AdvanceTo(0.1)
        q_before = _arm_positions(production, simulator)
        simulator.AdvanceTo(0.59)
        self.assertEqual(checks["n"], 200)
        np.testing.assert_allclose(
            _arm_positions(production, simulator), q_before, atol=1e-9
        )


class AsyncPlannerDynamicTest(unittest.TestCase):
    def test_holds_still_while_planning1(self):
        """
        Description
        -----------
        On the dynamic (PD-controlled) station, the arm must stay where it was at
        plan_start_time while the plan is computed, and the plan must start from
        the arm's measured configuration at that time.
        """
        production = ChemLab2(meshcat_port_number=None)
        rrt = _rrt(production)
        seen = {}

        def plan(q_start, q_goal, collision_check):
            seen["q_start"] = np.array(q_start)
            return rrt.plan(q_start, q_goal, collision_check)

        config = AsyncPlanningConfig(plan_start_time=0.5, planning_latency=1.0)
        simulator, planner = _build(production, plan, config)

        simulator.AdvanceTo(1.49)
        self.assertEqual(
            planner.get_planning_status(simulator.get_context()), "planning"
        )
        drift = np.linalg.norm(_arm_positions(production, simulator) - seen["q_start"])
        self.assertLess(drift, config.drift_tolerance / 10.0)

        simulator.AdvanceTo(1.6)
        np.testing.assert_allclose(
            production.plan_dispenser.planned_trajectory.value(0.0).flatten(),
            seen["q_start"],
            atol=1e-12,
        )


if __name__ == "__main__":
    unittest.main()
