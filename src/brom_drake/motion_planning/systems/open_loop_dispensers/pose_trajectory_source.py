from pydrake.all import AbstractValue, Context, LeafSystem, PiecewisePose, RigidTransform

class PoseTrajectorySource(LeafSystem):
    def __init__(self, trajectory: PiecewisePose):
        LeafSystem.__init__(self)

        # Setup
        self._trajectory = trajectory

        # Create output port
        initial_pose = self._trajectory.GetPose(0.0)
        self.DeclareAbstractOutputPort(
            "current_pose",
            lambda: AbstractValue.Make(initial_pose),
            self.get_current_pose,
        )

    def get_current_pose(self, context: Context, output: AbstractValue):
        # Get the current time
        t = context.get_time()

        # Evaluate the trajectory at the current time
        current_pose = self._trajectory.GetPose(t)

        # Set the output
        output.set_value(current_pose)
