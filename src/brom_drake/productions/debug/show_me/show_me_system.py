import numpy as np
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import ModelInstanceIndex
from pydrake.systems.framework import LeafSystem, BasicVector, Context

from brom_drake.utils import AddMultibodyTriad

class ShowMeSystem(LeafSystem):
    """
    **Description**
    
    This class defines a LeafSystem that will force a specific model
    (model_index) in a MultibodyPlant to hold the joint positions
    provided to it as input. 
    
    The object may or may not be welded to the world frame.

    **Block Diagram**

    The ShowMeSystem block diagram is as follows: ::

                                    |---------------|
                                    |               |
        desired_joint_positions --->| ShowMeSystem  |---> measured_joint_positions
                                    |               |
                                    |---------------|

    **Parameters**

    plant : MultibodyPlant
        The MultibodyPlant containing the model to be shown.

    model_index : ModelInstanceIndex
        The ModelInstanceIndex of the model to be shown.

    desired_joint_positions : np.ndarray
        The desired joint positions for the model, if any.

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

        # Define the name of the system
        self.set_name(f"ShowMeSystem_for_{plant.GetModelInstanceName(model_index)}")

    def SetModelJointPositions(self, context: Context, output: BasicVector):
        """
        **Description**
        
        This method will set the joint positions of the model to the desired joint positions.
        
        **Parameters**
        
        context: Context
            The context of the LeafSystem at the time that this callback is triggered.

        output: BasicVector
            The output vector to be populated with the current joint positions.
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

