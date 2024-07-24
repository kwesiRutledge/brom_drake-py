
from pydrake.geometry import SceneGraph
from pydrake.multibody.plant import MultibodyPlant
from pydrake.systems.framework import Diagram, DiagramBuilder
from pydrake.systems.primitives import Demultiplexer

# Local imports
from brom_drake.robots.gripper_type import GripperType

class UR10eStation(Diagram):
    """
    A template system diagram for controlling a UR10e robot in a simulated environment.
    """

    def __init__(self, time_step: float = 0.002):
        """
        Description:

            This class defines the
        """
        Diagram.__init__(self)
        self.set_name("UR10e_Station")

        # Create a diagram builder
        self.builder = DiagramBuilder()

        # Create scene_graph and plant
        self.scene_graph = self.builder.AddSystem(SceneGraph())
        self.scene_graph.set_name("UR10e_Station_SceneGraph")

        self.plant = self.builder.AddSystem(MultibodyPlant(time_step=time_step))
        self.plant.RegisterAsSourceForSceneGraph(self.scene_graph)
        self.plant.set_name("UR10e_Station_Plant")

        # Add a shadow plant that only has access to the robot arm + gripper mass,
        # and is used for the controller. (Doesn't know about anything else in the scene)
        self.controller_plant = MultibodyPlant(time_step=time_step)

        # Body ID's and Poses for Anything else in the scene
        self.object_ids = []
        self.object_poses = []

        # Which sort of gripper we're using (if any)
        self.gripper_type = GripperType.NoGripper

        # Whether or not we have a camera in the simulation
        # self.has_camera = False

    def Finalize(self):
        """
        Description
        ----------
        This finalizes the diagram by:
        - Finalizing all plants in the diagram
        :return: Nothing
        """
        # Setup

        # Finalize all plants
        self.plant.Finalize()
        self.controller_plant.Finalize()

        # Set up the scene graph
        self.builder.Connect(
            self.scene_graph.get_query_output_port(),
            self.plant.get_geometry_query_input_port()
        )

        self.builder.Connect(
            self.plant.get_geometry_query_output_port(),
            self.scene_graph.get_source_pose_port(self.plant.get_source_id())
        )

        # Create controller that uses the shadow plant to control the robot
        cartesian_controller = self.builder.AddSystem(
            CartesianController(self.controller_plant, self.controller_arm),
        )
        cartesian_controller.set_name("UR10e_Station_CartesianController")

        # End effector target and target type go to the controller
        self.builder.ExportInput(
            cartesian_controller.ee_target_port,
            "ee_target",
        )
        self.builder.ExportInput(
            cartesian_controller.target_type_port,
            "ee_target_type",
        )

        # Output measured arm position and velocity
        demux = self.builder.AddSystem(
            Demultiplexer(self.plant.num_multibody_states(self.arm), self.plant.num_positions(self.arm)),
        )
        demux.set_name("UR10e_Station_Demux")

        self.builder.Connect(
            self.plant.get_state_output_port(self.arm),
            demux.get_input_port(0),
        )
        self.builder.ExportOutput(
            demux.get_output_port(0),
            "measured_arm_position",
        )
        self.builder.ExportOutput(
            demux.get_output_port(1),
            "measured_arm_velocity",
        )

        # Measured Arm Position and Velocity are sent to the controller
        self.builder.Connect(
            demux.get_output_port(0),
            cartesian_controller.model_position_port,
        )
        self.builder.Connect(
            demux.get_output_port(1),
            cartesian_controller.model_velocity_port,
        )

        # Torques from controller go to the "true" plant
        self.builder.Connect(
            cartesian_controller.control_output_port,
            self.plant.get_actuation_input_port(self.arm),
        )

        # Controller outputs measured arm torques, end-effector pose, end-effector twist
        controller_output_port_names = ["applied_arm_torque", "measured_ee_pose", "measured_ee_twist"]
        for output_port_name in controller_output_port_names:
            self.builder.ExportOutput(
                cartesian_controller.GetOutputPort(output_port_name),
                output_port_name,
            )

        # Create gripper controller