from manipulation.scenarios import AddMultibodyTriad
import numpy as np
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import ModelInstanceIndex
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
        model_index: ModelInstanceIndex,
        desired_joint_positions: np.ndarray,
        **kwargs,
    ):
        LeafSystem.__init__(self)

        # Constants
        self.plant = plant
        self.model_index = model_index

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
        try:
            self.plant.SetPositions(
                self.mutable_plant_context,
                self.model_index,
                pose_as_vec,
            )
        except Exception as e:
            print(e)
            raise ValueError(
                f"Could not set the joint positions; this is most likely a mismatch between:\n" + \
                f"- Number of joint positions in command: {len(pose_as_vec)}\n" + \
                f"- Number of joint positions in model: {self.plant.num_positions(self.model_index)}."
            )

        self.plant.SetVelocities(
            self.mutable_plant_context,
            self.model_index,
            np.zeros(self.plant.num_velocities(self.model_index)),
        )

        output.SetFromVector(pose_as_vec)

