"""
Description
-----------
This module tests the Manipulation Station that we're making
for the UR10e robot.
"""
import numpy as np
from pydrake.all import (
    AbstractValue,
    ConstantValueSource,
    DiagramBuilder,
    Simulator,
)
import unittest

# Internal imports
from brom_drake.all import (
    add_watcher_and_build,
)
from brom_drake.robots.gripper_type import GripperType
from brom_drake.robots.stations.classical.ur10e import UR10eStation
from brom_drake.motion_planning.systems import (
    OpenLoopPlanDispenser
)

class TestUR10e(unittest.TestCase):
    def test_init1(self):
        """
        Description
        -----------
        This test verifies that we can initialize the UR10e station without any errors.
        """
        # Setup
        station = UR10eStation(
            meshcat_port_number=None, # Use None for CI
        )

        self.assertTrue(True)

    def test_diagram1(self):
        """
        Description
        -----------
        This test verifies that we can build the UR10e station without any errors.
        """
        # Setup
        builder = DiagramBuilder()
        station = builder.AddSystem(
            UR10eStation(
                meshcat_port_number=7004,
                gripper_type=GripperType.NoGripper,
            )
        )

        # Finalize Station
        station.Finalize()

        # Create a simple plan and a source for it
        plan = np.array([
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, np.pi],
        ])
        plan_source = builder.AddSystem(
            ConstantValueSource(
                AbstractValue.Make(plan)
            )
        )

        ready_signal_source = builder.AddSystem(
            ConstantValueSource(
                AbstractValue.Make(True),
            )
        )

        # Create a simple dispenser to give to the robot
        dispenser = builder.AddSystem(
            OpenLoopPlanDispenser(
                n_dof=6,
                speed=0.1,
            )
        )

        # Connect the plan source to the dispenser and ready signal
        builder.Connect(
            plan_source.get_output_port(),
            dispenser.GetInputPort("plan"),
        )
        builder.Connect(
            ready_signal_source.get_output_port(),
            dispenser.GetInputPort("plan_ready"),
        )

        # Connect the dispenser to the robot
        builder.Connect(
            dispenser.GetOutputPort("point_in_plan"),
            station.GetInputPort("desired_joint_positions"),
        )

        # Build the diagram with watcher
        watcher, diagram, diagram_context = add_watcher_and_build(
            builder,
            watcher_dir="brom/watcher_testur10e_test_diagram1",
        )

        # Simulate the diagram
        simulator = Simulator(diagram, diagram_context)
        simulator.Initialize()
        simulator.set_target_realtime_rate(1.0)

        # Set time limit + advance to that time
        simulator.AdvanceTo(0.05)
        planned_trajectory = dispenser.planned_trajectory
        simulator.AdvanceTo(planned_trajectory.end_time() + 1.0)

        


if __name__ == "__main__":
    unittest.main()