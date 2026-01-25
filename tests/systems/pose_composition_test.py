from brom_drake.systems.pose_composition import PoseCompositionSystem
from brom_drake.watchers.port_watcher.port_watcher import PortWatcher
from brom_drake.watchers.add_watcher import add_watcher_and_build
import numpy as np
from pydrake.all import (
    AbstractValue,
    ConstantValueSource,
    DiagramBuilder,
    Quaternion,
    RigidTransform,
    RollPitchYaw,
    Simulator,
)
import unittest

class TestPoseComposition(unittest.TestCase):
    def test_correctness1(self):
        """
        *Description*

        This test verifies that the composition of two rigidtransforms
        given as input to the ``PoseCompositionSystem`` system is computed correctly.
        In this case, we will give two unique RigidTransform and verifiy
        that the output is not identity.
        
        """
        # Create builder
        builder = DiagramBuilder()

        # Create the two transforms that we want to compose
        # and sources for them both
        pose1 = RigidTransform(
            p=np.array([0.0, 1.0, 0.0]),
            rpy=RollPitchYaw(0.0, (-3.0/8.0)*np.pi, 0.0)
        )
        source1 = builder.AddSystem(
            ConstantValueSource(value=AbstractValue.Make(pose1))
        )

        pose2 = RigidTransform(
            p=np.array([-2.0, 0.3, 0.1]),
            rpy=RollPitchYaw(np.pi, 0.0, 0.0)
        )
        source2 = builder.AddSystem(
            ConstantValueSource(value=AbstractValue.Make(pose2))
        )

        # Create the composition system and connect it
        composition_system = builder.AddNamedSystem(
            system=PoseCompositionSystem(),
            name="composer"
        )

        builder.Connect(source1.get_output_port(), composition_system.GetInputPort("pose_AB"))
        builder.Connect(source2.get_output_port(), composition_system.GetInputPort("pose_BC"))

        # Add a diagram watcher and log the signals
        watcher, diagram, diagram_context = add_watcher_and_build(
            builder,
            watcher_dir="brom/pose_composition_test/test_correctness1"
        )

        # Simulate
        simulator = Simulator(diagram, diagram_context)
        simulator.Initialize()
        simulator.AdvanceTo(1.0)

        # Extract the values in the data field
        composition_system_port_watchers = watcher.port_watchers[composition_system.get_name()]

        output_pose_port_watcher: PortWatcher = composition_system_port_watchers["pose_AC"]
        
        log_for_output_pose = output_pose_port_watcher.get_vector_log_sink().FindLog(diagram_context)
        final_output_pose_as_vec: np.ndarray = log_for_output_pose.data()[:,-1]
        final_output_pose = RigidTransform(
            p=final_output_pose_as_vec[:3],
            quaternion=Quaternion(
                wxyz=final_output_pose_as_vec[3:7]
            )
        )

        # Verify that the output is correct
        expected_output_pose: RigidTransform = pose1.multiply(pose2)
        self.assertTrue(
            expected_output_pose.IsNearlyEqualTo(final_output_pose, tolerance=1e-10),
        )

    def test_correctness2(self):
        """
        *Description*

        This test verifies that the composition of two rigidtransforms
        given as input to the ``PoseCompositionSystem`` system is computed correctly.
        In this case, we will give two RigidTransform objects that, when composed,
        should produce the identity.
        
        """
        # Create builder
        builder = DiagramBuilder()

        # Create the two transforms that we want to compose
        # and sources for them both
        pose1 = RigidTransform(
            p=np.array([0.0, 1.0, 0.0]),
            rpy=RollPitchYaw(0.0, (-3.0/8.0)*np.pi, 0.0)
        )
        source1 = builder.AddSystem(
            ConstantValueSource(value=AbstractValue.Make(pose1))
        )

        source2 = builder.AddSystem(
            ConstantValueSource(value=AbstractValue.Make(pose1.inverse()))
        )

        # Create the composition system and connect it
        composition_system = builder.AddNamedSystem(
            system=PoseCompositionSystem(),
            name="composer"
        )

        builder.Connect(source1.get_output_port(), composition_system.GetInputPort("pose_AB"))
        builder.Connect(source2.get_output_port(), composition_system.GetInputPort("pose_BC"))

        # Add a diagram watcher and log the signals
        watcher, diagram, diagram_context = add_watcher_and_build(
            builder,
            watcher_dir="brom/pose_composition_test/test_correctness2"
        )

        # Simulate
        simulator = Simulator(diagram, diagram_context)
        simulator.Initialize()
        simulator.AdvanceTo(1.0)

        # Extract the values in the data field
        composition_system_port_watchers = watcher.port_watchers[composition_system.get_name()]

        output_pose_port_watcher: PortWatcher = composition_system_port_watchers["pose_AC"]
        
        log_for_output_pose = output_pose_port_watcher.get_vector_log_sink().FindLog(diagram_context)
        final_output_pose_as_vec: np.ndarray = log_for_output_pose.data()[:,-1]
        final_output_pose = RigidTransform(
            p=final_output_pose_as_vec[:3],
            quaternion=Quaternion(
                wxyz=final_output_pose_as_vec[3:7]
            )
        )

        # Verify that the output is correct
        expected_output_pose: RigidTransform = RigidTransform.Identity()
        self.assertTrue(
            expected_output_pose.IsNearlyEqualTo(final_output_pose, tolerance=1e-10),
        )

if __name__ == '__main__':
    unittest.main()