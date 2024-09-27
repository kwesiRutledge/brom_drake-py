from manipulation.scenarios import AddMultibodyTriad
import numpy as np
from pydrake.multibody.plant import MultibodyPlant
from pydrake.systems.framework import LeafSystem, BasicVector


class ShowMeSystem(LeafSystem):
    """
    Description
    -----------
    This function will create a system that will show the user's model.
    :param LeafSystem:
    :return:
    """
    def __init__(
        self,
        plant: MultibodyPlant,
        model_name: str,
        desired_joint_positions: np.ndarray,
        **kwargs,
    ):
        LeafSystem.__init__(self)

        # Constants
        self.plant = plant
        self.model_name = model_name

        # TODO(kwesi): Do we need any ports?
        # Define Input Ports
        n = len(desired_joint_positions)
        self.desired_joint_positions_port = self.DeclareVectorInputPort(
            "desired_joint_positions",
            BasicVector(n),
        )

        # Define Output Ports
        self.DeclareVectorOutputPort(
            "measured_joint_positions",
            BasicVector(n),
            self.SetModelJointPositions,
            {self.time_ticket()}    # indicate that this doesn't depend on any inputs,
        )                           # but should still be updated each timestep

        # Create plant context
        self.mutable_plant_context = None

    def SetModelJointPositions(self, context, output):
        """
        Description
        -----------
        This method will set the joint positions of the model to the desired joint positions.
        :param context:
        :param output:
        :return:
        """
        # Setup

        # Get desired joint positions
        pose_as_vec = self.desired_joint_positions_port.Eval(context)

        # Set the joint positions
        self.plant.SetPositions(
            self.mutable_plant_context,
            pose_as_vec,
        )

        self.plant.SetVelocities(
            self.mutable_plant_context,
            np.zeros(self.plant.num_velocities()),
        )

        output.SetFromVector(pose_as_vec)

