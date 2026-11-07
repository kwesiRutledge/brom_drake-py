from pathlib import Path
import unittest
from typing import Tuple

import numpy as np
from pydrake.common.value import AbstractValue
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import ConstantValueSource

# Internal Imports
from brom_drake.all import add_watcher_and_build
from brom_drake.motion_planning.systems.open_loop_dispensers.open_loop_plan_dispenser import (
    OpenLoopPlanDispenser,
)


class OpenLoopPlanDispenserTest(unittest.TestCase):
    def test_init1(self):
        """
        *Description*

        This test checks if the open loop plan dispenser can be initialized without any errors.
        """
        # Setup
        dispenser = OpenLoopPlanDispenser(6, 1.0)

        # Successfully Ran
        self.assertTrue(True)

    @staticmethod
    def setup_basic_sim1() -> Tuple[int, ConstantValueSource, ConstantValueSource]:
        """
        *Description*

        This function will create a basic simulation which can be used to test the open loop plan dispenser.
        """
        # Setup
        n_dof = 6
        # simple_plan = np.random.normal(0, 6, (2, n_dof)) # Create a plan with 2 rows and 6 columns
        simple_plan = np.array(
            [[0, 0, 0, 0, 0, 0], [1, 1, 1, 1, 1, 1]]
        )  # Create a simple 2 point plan.

        # Create a source which will output a simple 2 point plan.
        plan_source = ConstantValueSource(AbstractValue.Make(simple_plan))

        plan_ready_source = ConstantValueSource(AbstractValue.Make(True))

        return n_dof, plan_source, plan_ready_source, simple_plan

    def test_in_simulation1(self):
        """
        *Description*

        This test checks if the open loop plan dispenser can be used in a simulation without any errors.
        """
        # Setup sim
        builder = DiagramBuilder()
        n_dof, plan_source, plan_ready_source, _ = self.setup_basic_sim1()
        builder.AddSystem(plan_source)
        builder.AddSystem(plan_ready_source)

        # Create an OpenLoopPlanDispenser
        dispenser = OpenLoopPlanDispenser(n_dof, 1.0)
        builder.AddSystem(dispenser)

        # Connect the source to the dispenser
        builder.Connect(
            plan_source.get_output_port(),
            dispenser.get_input_port(0),
        )

        builder.Connect(
            plan_ready_source.get_output_port(),
            dispenser.get_input_port(1),
        )

        # Build the diagram with watcher and simulate
        watcher, diagram, diagram_context = add_watcher_and_build(builder)

        # Create a context for the diagram
        simulator = Simulator(diagram, diagram_context)
        simulator.set_target_realtime_rate(1.0)

        # Run simulation
        simulator.Initialize()
        simulator.AdvanceTo(5.0)

        self.assertTrue(True)

    def test_in_simulation2(self):
        """
        *Description*

        This test checks if the open loop plan dispenser can be used in a simulation and returns the correct results.
        If doing proper interpolation, then this test will pass. If it is not interpolating correctly, then it will fail.
        """
        # Setup sim
        builder = DiagramBuilder()
        n_dof, plan_source, plan_ready_source, simple_plan = self.setup_basic_sim1()
        builder.AddSystem(plan_source)
        builder.AddSystem(plan_ready_source)

        # Create an OpenLoopPlanDispenser
        dispenser = OpenLoopPlanDispenser(n_dof, 1.0)
        builder.AddSystem(dispenser)

        # Connect the source to the dispenser
        builder.Connect(
            plan_source.get_output_port(),
            dispenser.get_input_port(0),
        )

        builder.Connect(
            plan_ready_source.get_output_port(),
            dispenser.get_input_port(1),
        )

        # Build the diagram with watcher and simulate
        watcher, diagram, diagram_context = add_watcher_and_build(builder)

        # Create a context for the diagram
        simulator = Simulator(diagram, diagram_context)
        simulator.set_target_realtime_rate(1.0)

        # Run simulation
        simulator.Initialize()
        simulator.AdvanceTo(5.0)

        # Check if the output of the dispenser is as expected (the last point should be
        # the last point of the simple_plan)
        first_key = dispenser.get_name()
        point_in_plan_watcher = watcher._port_watchers[first_key][
            dispenser.get_output_port(0).get_name()
        ]
        temp_log = point_in_plan_watcher.get_vector_log_sink().FindLog(diagram_context)
        point_in_plan_data = temp_log.data()
        last_point = point_in_plan_data[:, -1]

        self.assertTrue(
            np.allclose(
                last_point,
                simple_plan[-1, :],
                atol=1e-2,
            )
        )

        self.assertTrue(True)

    @staticmethod
    def setup_basic_sim2() -> Tuple[int, ConstantValueSource, ConstantValueSource]:
        """
        Description
        -----------
        This function will create a basic simulation which can be used to test the open loop plan dispenser.
        :return:
        """
        # Setup
        n_dof = 6
        # simple_plan = np.random.normal(0, 6, (2, n_dof)) # Create a plan with 2 rows and 6 columns
        simple_plan = np.array(
            [[0, 0, 0, 0, 0, 0], [1, 1, 1, 1, 1, 1], [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]]
        )  # Create a simple 2 point plan.

        # Create a source which will output a simple 2 point plan.
        plan_source = ConstantValueSource(AbstractValue.Make(simple_plan))

        plan_ready_source = ConstantValueSource(AbstractValue.Make(True))

        return n_dof, plan_source, plan_ready_source, simple_plan

    def test_in_simulation3(self):
        """
        Description
        -----------
        This test checks if the open loop plan dispenser can be used in a simulation and returns the correct results.
        If doing proper interpolation, then this test will pass. If it is not interpolating correctly, then it will fail.
        Trying a more complicated plan (with more points).
        :return:
        """
        # Setup sim
        builder = DiagramBuilder()
        n_dof, plan_source, plan_ready_source, simple_plan = self.setup_basic_sim2()
        builder.AddSystem(plan_source)
        builder.AddSystem(plan_ready_source)

        # Create an OpenLoopPlanDispenser
        dispenser = OpenLoopPlanDispenser(n_dof, 1.0)
        builder.AddSystem(dispenser)

        # Connect the source to the dispenser
        builder.Connect(
            plan_source.get_output_port(),
            dispenser.get_input_port(0),
        )

        builder.Connect(
            plan_ready_source.get_output_port(),
            dispenser.get_input_port(1),
        )

        # Build the diagram with watcher and simulate
        watcher, diagram, diagram_context = add_watcher_and_build(
            builder, watcher_dir=Path("brom/watcher_plots3")
        )

        # Create a context for the diagram
        simulator = Simulator(diagram, diagram_context)
        simulator.set_target_realtime_rate(1.0)

        # Run simulation
        simulator.Initialize()
        dt = 0.1
        N_steps = 50
        for ii in range(N_steps):
            simulator.AdvanceTo(ii * dt)
        simulator.AdvanceTo(8.0)

        # Check if the output of the dispenser is as expected (the last point should be
        # the last point of the simple_plan)
        first_key = dispenser.get_name()
        point_in_plan_watcher = watcher._port_watchers[first_key][
            dispenser.get_output_port(0).get_name()
        ]
        temp_log = point_in_plan_watcher.get_vector_log_sink().FindLog(diagram_context)
        point_in_plan_data = temp_log.data()
        last_point = point_in_plan_data[:, -1]

        print("Last Point: ", last_point)
        print("Expected Point: ", simple_plan[-1, :])

        self.assertTrue(
            np.allclose(
                last_point,
                simple_plan[-1, :],
                atol=1e-2,
            )
        )

        self.assertTrue(True)

    def test_unconnected_inputs1(self):
        """
        *Description*

        With neither input connected (e.g., a production simulated before a
        motion planner is cast), the dispenser treats the plan as "not ready"
        and outputs zeros instead of raising.
        """
        dispenser = OpenLoopPlanDispenser(6, 1.0)
        context = dispenser.CreateDefaultContext()

        point = dispenser.GetOutputPort("point_in_plan").Eval(context)
        np.testing.assert_array_equal(point, np.zeros(6))
        self.assertIsNone(dispenser.planned_trajectory)

    def test_ready_without_plan1(self):
        """
        *Description*

        A ready flag with no plan connected has nothing to dispense, so it
        must also be treated as "not ready".
        """
        dispenser = OpenLoopPlanDispenser(6, 1.0)
        context = dispenser.CreateDefaultContext()
        dispenser.GetInputPort("plan_ready").FixValue(context, AbstractValue.Make(True))

        point = dispenser.GetOutputPort("point_in_plan").Eval(context)
        np.testing.assert_array_equal(point, np.zeros(6))
        self.assertIsNone(dispenser.planned_trajectory)

    def test_plan_without_ready_flag1(self):
        """
        *Description*

        A plan with no ready flag connected is not dispensed yet; the
        dispenser holds the plan's first configuration.
        """
        plan = np.array(
            [[0.1, 0.2, 0.3, 0.4, 0.5, 0.6], [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]]
        )
        dispenser = OpenLoopPlanDispenser(6, 1.0)
        context = dispenser.CreateDefaultContext()
        dispenser.GetInputPort("plan").FixValue(context, AbstractValue.Make(plan))

        point = dispenser.GetOutputPort("point_in_plan").Eval(context)
        np.testing.assert_allclose(point, plan[0])
        self.assertIsNone(dispenser.planned_trajectory)

    def test_single_point_plan1(self):
        """
        *Description*

        A plan with a single configuration (the robot already starts at the
        goal) is held, instead of failing to build a trajectory.
        """
        q = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
        dispenser = OpenLoopPlanDispenser(6, 1.0)
        context = dispenser.CreateDefaultContext()
        dispenser.GetInputPort("plan").FixValue(
            context, AbstractValue.Make(q.reshape(1, -1))
        )
        dispenser.GetInputPort("plan_ready").FixValue(context, AbstractValue.Make(True))

        np.testing.assert_allclose(
            dispenser.GetOutputPort("point_in_plan").Eval(context), q
        )
        self.assertEqual(dispenser.t_final, 0.0)

    def test_repeated_points1(self):
        """
        *Description*

        Consecutive duplicate configurations in a plan are skipped, so the
        trajectory's times stay strictly increasing.
        """
        plan = np.array([[0.0] * 6, [0.0] * 6, [1.0] + [0.0] * 5, [1.0] + [0.0] * 5])
        dispenser = OpenLoopPlanDispenser(6, 1.0)
        context = dispenser.CreateDefaultContext()
        dispenser.GetInputPort("plan").FixValue(context, AbstractValue.Make(plan))
        dispenser.GetInputPort("plan_ready").FixValue(context, AbstractValue.Make(True))

        dispenser.GetOutputPort("point_in_plan").Eval(context)
        self.assertAlmostEqual(dispenser.t_final, 1.0)
        np.testing.assert_allclose(
            dispenser.planned_trajectory.value(1.0).flatten(), plan[-1]
        )


if __name__ == "__main__":
    unittest.main()
