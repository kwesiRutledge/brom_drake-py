from brom_drake.watchers.port_watcher.port_watcher import PortWatcher
from brom_drake.systems.abstract_list_selection_system import AbstractListSelectionSystem
from brom_drake.watchers.add_watcher import add_watcher_and_build
import numpy as np
from pydrake.all import (
    AbstractValue,
    ConstantValueSource,
    DiagramBuilder,
    Quaternion,
    RigidTransform,
    Simulator,
)
import unittest

class TestAbstractListSelectionSystem(unittest.TestCase):
    def test_correctness1(self):
        """
        *Description*

        This test verifies that the `AbstractListSelectionSystem` correctly selects
        an element from a list of RigidTransform objects based on the specified index.
        """
        # Create a list of RigidTransform objects
        transform_list = [
            RigidTransform(),  # Identity
            RigidTransform(p=[1.0, 0.0, 0.0]),  # Translation along x
            RigidTransform(p=[0.0, 1.0, 0.0]),  # Translation along y
        ]

        # Create builder
        builder = DiagramBuilder()

        # Create the List Selection System to select the second element (index 1)
        selection_system = builder.AddNamedSystem(
            system=AbstractListSelectionSystem(index=1, output_type=RigidTransform),
            name="list_selector"
        )

        # Create a source for the list input
        list_source = builder.AddSystem(
            ConstantValueSource(value=AbstractValue.Make(transform_list))
        )

        # Connect the list source to the selection system
        builder.Connect(list_source.get_output_port(), selection_system.get_input_port())

        # Build the diagram with a watcher to observe the output
        watcher, diagram, diagram_context = add_watcher_and_build(builder)

        # Create a simulator
        simulator = Simulator(diagram, diagram_context)
        simulator.Initialize()
        simulator.AdvanceTo(1.0)

        # Retrieve the output from the watcher
        selector_dict_of_port_watchers = watcher.port_watchers["list_selector"]
        output_pose_port_watcher: PortWatcher = selector_dict_of_port_watchers["element_out"]

        log_for_output_pose = output_pose_port_watcher.get_vector_log_sink().FindLog(diagram_context)
        final_output_pose_as_vec: np.ndarray = log_for_output_pose.data()[:,-1]
        final_output_pose = RigidTransform(
            p=final_output_pose_as_vec[:3],
            quaternion=Quaternion(
                wxyz=final_output_pose_as_vec[3:7]
            )
        )

        # Verify that the output is correct
        expected_output_pose: RigidTransform = transform_list[1]
        self.assertTrue(
            expected_output_pose.IsNearlyEqualTo(final_output_pose, tolerance=1e-10),
        )

if __name__ == "__main__":
    unittest.main()