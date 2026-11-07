from brom_drake.systems.pose_memory_system import PoseMemorySystem
import numpy as np
from pydrake.all import (
    AbstractValue,
    RigidTransform,
    RollPitchYaw,
    Simulator,
)
import unittest


class TestPoseMemorySystem(unittest.TestCase):
    def setUp(self):
        # Create the system and a simulator for it
        self.memory_system = PoseMemorySystem()
        self.simulator = Simulator(self.memory_system)
        self.context = self.simulator.get_mutable_context()

        # Create some unique poses
        self.pose1 = RigidTransform(
            p=np.array([0.0, 1.0, 0.0]),
            rpy=RollPitchYaw(0.0, (-3.0 / 8.0) * np.pi, 0.0),
        )
        self.pose2 = RigidTransform(
            p=np.array([-2.0, 0.3, 0.1]),
            rpy=RollPitchYaw(np.pi, 0.0, 0.0),
        )

    def set_inputs(self, pose_in: RigidTransform, hold: bool):
        """
        *Description*

        Fixes the values of the input ports of the memory system.
        """
        self.memory_system.get_pose_in_port().FixValue(
            self.context, AbstractValue.Make(pose_in)
        )
        self.memory_system.get_hold_port().FixValue(
            self.context, AbstractValue.Make(hold)
        )

    def get_output(self) -> RigidTransform:
        return self.memory_system.get_pose_out_port().Eval(self.context)

    def test_pass_through1(self):
        """
        *Description*

        This test verifies that, while hold is False, the system
        outputs the input pose (even when it changes).
        """
        # Setup
        self.set_inputs(self.pose1, hold=False)
        self.simulator.Initialize()

        # Verify
        self.assertTrue(self.get_output().IsExactlyEqualTo(self.pose1))

        # Change the input pose
        self.set_inputs(self.pose2, hold=False)
        self.simulator.AdvanceTo(0.1)

        # Verify
        self.assertTrue(self.get_output().IsExactlyEqualTo(self.pose2))

    def test_hold1(self):
        """
        *Description*

        This test verifies that, once hold becomes True, the system
        keeps outputting the pose it received before holding
        (even when the input pose changes).
        """
        # Setup
        self.set_inputs(self.pose1, hold=False)
        self.simulator.Initialize()
        self.simulator.AdvanceTo(0.1)

        # Start holding and change the input pose
        self.set_inputs(self.pose2, hold=True)

        # Verify (immediately and after time passes)
        self.assertTrue(self.get_output().IsExactlyEqualTo(self.pose1))

        self.simulator.AdvanceTo(0.2)
        self.assertTrue(self.get_output().IsExactlyEqualTo(self.pose1))

    def test_hold2(self):
        """
        *Description*

        This test verifies that, once hold becomes False again,
        the system goes back to passing the input pose through.
        """
        # Setup
        self.set_inputs(self.pose1, hold=False)
        self.simulator.Initialize()
        self.simulator.AdvanceTo(0.1)

        self.set_inputs(self.pose2, hold=True)
        self.simulator.AdvanceTo(0.2)

        # Stop holding
        self.set_inputs(self.pose2, hold=False)

        # Verify
        self.assertTrue(self.get_output().IsExactlyEqualTo(self.pose2))

    def test_initialization1(self):
        """
        *Description*

        This test verifies that the system remembers the input pose
        when the simulator is initialized (i.e., holding from the start
        outputs the initial input pose instead of the identity).
        """
        # Setup
        self.set_inputs(self.pose1, hold=False)
        self.simulator.Initialize()

        # Start holding and change the input pose
        self.set_inputs(self.pose2, hold=True)

        # Verify
        self.assertTrue(self.get_output().IsExactlyEqualTo(self.pose1))

    def test_new_context1(self):
        """
        *Description*

        This test verifies that the remembered pose is stored in the context
        (i.e., a new context does not remember anything from another context).
        """
        # Setup
        self.set_inputs(self.pose1, hold=False)
        self.simulator.Initialize()

        # Create a new context and hold in it
        new_context = self.memory_system.CreateDefaultContext()
        self.memory_system.get_pose_in_port().FixValue(
            new_context, AbstractValue.Make(self.pose2)
        )
        self.memory_system.get_hold_port().FixValue(
            new_context, AbstractValue.Make(True)
        )

        # Verify
        output = self.memory_system.get_pose_out_port().Eval(new_context)
        self.assertTrue(output.IsExactlyEqualTo(RigidTransform()))


if __name__ == "__main__":
    unittest.main()
