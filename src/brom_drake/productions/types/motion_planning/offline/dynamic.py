import os
from pathlib import Path
from typing import Tuple, Callable

import networkx as nx
import numpy as np
from pydrake.all import (
    RotationMatrix, Quaternion, RollPitchYaw,
    Solve, SolutionResult, SpatialVelocity,
    InverseKinematics, ModelInstanceIndex,
)
from pydrake.common.value import AbstractValue
from pydrake.math import RigidTransform
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant
from pydrake.systems.framework import Diagram, Context
from pydrake.systems.primitives import ConstantVectorSource, ConstantValueSource

# Internal Imports
from brom_drake.motion_planning.systems.prototypical_planner import PrototypicalPlannerSystem
from brom_drake.productions.types import BaseProduction
from brom_drake.productions.roles import kKinematicMotionPlanner, Role
from brom_drake.file_manipulation.urdf.shapes.sphere import SphereDefinition
from brom_drake.file_manipulation.urdf.simple_writer.urdf_definition import SimpleShapeURDFDefinition
from brom_drake.utils import Performer, MotionPlan


class OfflineDynamicMotionPlanningProduction(BaseProduction):
    def __init__(
        self,
        start_configuration: np.ndarray = None,
        start_pose: RigidTransform = None,
        goal_configuration: np.ndarray = None,
        goal_pose: RigidTransform = None,
        **kwargs,
    ):
        super().__init__(**kwargs)

        # Start and Goal Configurations
        self._start_config = start_configuration
        self._goal_config = goal_configuration

        # Start and Goal Poses
        self._start_pose = start_pose
        self._goal_pose = goal_pose

        # Create placeholder for the some of the systems that we'll use, including:
        # - plant
        # - scene_graph
        self.plant = None
        self.scene_graph = None

        # If the performer does not have plan_is_ready port, then
        # let's create a dummy value and connect it to the right place.
        plan_ready_source = ConstantValueSource(
            AbstractValue.Make(True),
        )
        self.plan_ready_source = self.builder.AddSystem(plan_ready_source)

        # Create placeholder for the robot model index
        self.robot_model_idx_ = None

        # Create a list of all OBJECTS in the supporting cast
        # and their desired poses. This will be a list of Tuple[ModelInstanceIndex, RigidTransform]
        self.models_in_supporting_cast = []

    def add_supporting_cast(self):
        """
        Description
        -----------
        This method will add the start and goal poses to the builder.
        
        Returns
        -------
        None
        """
        # Add visual elements for the start and goal poses
        self.add_start_and_goal_to_plant(self.plant)

    def add_robot_source_system(self):
        """
        Description
        -----------
        This method adds a source for providing the motion planner
        with the model index for the robot that we are trying to control.
        :return:
        """
        # Setup

        # Create AbstractValueSource
        robot_source_system = ConstantValueSource(
            AbstractValue.Make(self.robot_model_idx_)
        )
        robot_source_system.set_name("robot_model_index_source")

        self.builder.AddSystem(robot_source_system)

    def add_start_and_goal_to_plant(self, plant: MultibodyPlant):
        """
        Description
        -----------
        Add the start and goal to this plant.
        Parameters
        ----------
        :param plant: The plant to add the start and goal to.

        Notes
        -----
        Feel free to overwrite this if you have specific sizes of the start and goal in mind,
        or if you want to add more than just spheres, etc.
        :return:

        """

        # Create a sphere for the start
        start_sphere_defn = SimpleShapeURDFDefinition(
            name="start_sphere",
            shape=SphereDefinition(radius=0.1),
            create_collision=False,
            color=np.array([0, 0.3, 0.8, 0.3]),
        )
        start_sphere_urdf_location = Path("brom/models/spheres/start.urdf")
        os.makedirs(start_sphere_urdf_location.parent, exist_ok=True)

        start_sphere_defn.write_to_file(start_sphere_urdf_location)

        # Load the start sphere into the plant and rigidly attach it at the start_pose
        start_sphere_model_idx = Parser(plant).AddModels(str(start_sphere_urdf_location))[0]
        plant.WeldFrames(
            plant.world_frame(),
            plant.GetFrameByName("start_sphere_base_link", start_sphere_model_idx),
            self.start_pose,
        )

        # Create a sphere for the goal
        goal_sphere_defn = SimpleShapeURDFDefinition(
            name="end_sphere",
            shape=SphereDefinition(radius=0.1),
            create_collision=False,
            color=np.array([0, 0.8, 0.3, 0.3]),
        )
        goal_sphere_urdf_location = Path("brom/models/spheres/end.urdf")
        os.makedirs(goal_sphere_urdf_location.parent, exist_ok=True)

        goal_sphere_defn.write_to_file(goal_sphere_urdf_location)

        # Load the start sphere into the plant and rigidly attach it at the start_pose
        goal_sphere_model_idx = Parser(plant).AddModels(str(goal_sphere_urdf_location))[0]
        plant.WeldFrames(
            plant.world_frame(),
            plant.GetFrameByName(f"{goal_sphere_defn.name}_base_link", goal_sphere_model_idx),
            self.goal_pose,
        )

    def add_start_and_goal_sources_to_builder(self):
        """
        Description
        -----------
        This method will add the start and goal sources to the builder.
        """
        # Input Processing
        assert self.plant.is_finalized(), "Plant must be finalized before adding start and goal sources."

        # Add the start and goal poses to the builder
        self.add_start_source_system()
        self.add_goal_source_system()

    def add_start_source_system(self):
        """
        Add a source system for the start pose.
        :return:
        """
        # Setup

        # Adds a source system for the start pose to the builder
        start_source_system = ConstantVectorSource(self.start_configuration)
        start_source_system.set_name("start_configuration")

        self.builder.AddSystem(start_source_system)

    def add_goal_source_system(self):
        """
        Add a source system for the goal pose.
        :return:
        """
        # Setup

        # Adds a source system for the goal pose to the builder
        goal_source_system = ConstantVectorSource(self.goal_configuration)
        goal_source_system.set_name("goal_configuration")

        self.builder.AddSystem(goal_source_system)

    def add_main_cast(
        self,
        cast: Tuple[Role, Performer] = [],
    ):
        """
        Description
        -----------
        This small modification to the normal add_main_cast()
        function includes a call to create the optional inputs if needed.
        :param cast:
        :return:
        """
        # Setup

        # Default
        super().add_main_cast(cast=cast)

        # Create optional outputs, if needed
        for role_ii, performer_ii in cast:
            self.create_optional_outputs_if_necessary(
                role_ii, performer_ii
            )

    def build_production(
        self,
        with_watcher: bool = True,
    ) -> Tuple[Diagram, Context]:
        """
        Description
        -----------
        This method builds the production.
        It assumes that all components have been added to the builder.

        Arguments
        ---------
        with_watcher: bool
            A Boolean that determines whether to add a watcher to the diagram.
        """
        # Setup

        # Call the parent method
        diagram, diagram_context = super().build_production(with_watcher=with_watcher)

        # Set the initial poses of the members of the cast that are objects
        for model_ii, pose_ii in self.models_in_supporting_cast:
            body_list_ii = self.plant.GetBodyIndices(model_ii)
            # print(f"body_list_ii contains {len(body_list_ii)} elements")
            first_body = self.plant.get_body(body_list_ii[0])
            # first_body.body_frame().SetPoseInParentFrame(
            #     pose,
            # )
            self.plant.SetFreeBodyPose(
                self.plant.GetMyContextFromRoot(diagram_context),
                first_body,
                pose_ii,
            )

        return diagram, diagram_context

    def create_optional_outputs_if_necessary(
        self,
        role: Role, # This role will always be the Motion Planner, won't it?
        performer: Performer,
    ):
        """
        Description
        -----------
        This method checks to see if all optional outputs were connected
        for the given role. If all of them were, then we do nothing.
        If any of the optional outputs were NOT connected,
        then we add a dummy value to replace it.

        Requirements
        ------------
        This method should only be called after the full production has
        been built. Otherwise, this may have unexpected behavior.

        :return:
        """
        # Setup

        # Iterate through all OPTIONAL OUTPUTS
        # for assignment_ii in role.port_assignments:
        #     # Should we investigate this port?
        #     investigate_this_assignment = assignment_ii.pairing_type == PairingType.kOutput
        #     investigate_this_assignment = investigate_this_assignment and \
        #                                   (not assignment_ii.is_required)
        #
        #     if not investigate_this_assignment:
        #         continue # Skip this assignment value if it doesn't satisfy this
        #
        #     # Otherwise, check to see if we have the desired output port
        #     if performer.HasOutputPort(assignment_ii.performer_port_name):
        #         continue # If performer has the output port, then we should have already connected it; skip.
        #
        #     # If performer DOES NOT HAVE

        # TODO(kwesi): Maybe move above, general code to somewhere else?
        if role.name != "OfflineMotionPlanner":
            raise ValueError(
                f"Expected role for OfflineMotionPlanning Production to be \"OfflineMotionPlanner\";" +
                f"received {role.name}"
            )

        # Check to see if performer has the LAST assignment in the motion planning role
        last_assignment = role.port_assignments[-1]
        if last_assignment.performer_port_name != "plan_is_ready":
            raise ValueError(
                f"Expected last port assignment to be \"plan_is_ready\" assignment, but" +
                f"received an assignment with performer_port_name \"{last_assignment.performer_port_name}\""
            )

        if performer.HasOutputPort(last_assignment.performer_port_name):
            return # Do nothing; performer should already be connected

        # Find system we want to connect it to
        systems_list = last_assignment.find_any_matching_input_targets(self.builder)
        assert len(systems_list) == 1, \
            f"Expected 1 system to have port \"{last_assignment.external_target_name}\"," + \
            f" but found {len(systems_list)} systems with that output port."
        # TODO(kwesi): Perhaps move more of these error assertions to a separate file?

        self.builder.Connect(
            self.plan_ready_source.get_output_port(),
            systems_list[0].GetInputPort(last_assignment.external_target_name)
        )

    def define_pose_ik_problem(
        self,
        pose_WorldTarget: RigidTransform,
        target_frame_name: str,
        eps0: float = 2.5e-2,
        orientation_cost_scaling: float = 0.25,
    ) -> InverseKinematics:
        """
        Description
        -----------
        Sets up the inverse kinematics problem for the start pose
        input to theis function.
        :return:
        """
        # Setup

        # Create IK Problem
        ik_problem = InverseKinematics(self.plant)

        # Add Pose Target
        ik_problem.AddPositionConstraint(
            self.plant.world_frame(),
            pose_WorldTarget.translation(),
            self.plant.GetFrameByName(target_frame_name),
            (- np.ones((3,)) * eps0).reshape((-1, 1)),
            (+ np.ones((3,)) * eps0).reshape((-1, 1)),
        )

        # TODO(kwesi): Add OrientationCosntraint
        # ik_problem.AddOrientationConstraint(
        #     self.plant.world_frame(),
        #     RotationMatrix(Quaternion(input_pose_vec[3:]).rotation()),
        #     self.plant.GetFrameByName("ft_frame"),
        #     RotationMatrix.Identity(),
        #     0.25,
        # )

        ik_problem.AddOrientationCost(
            self.plant.world_frame(),
            pose_WorldTarget.rotation(),
            self.plant.GetFrameByName(target_frame_name),
            RotationMatrix.Identity(),
            orientation_cost_scaling,
        )

        return ik_problem
    
    def easy_cast_and_build(
        self,
        planning_algorithm: Callable[
            [np.ndarray, np.ndarray, Callable[[np.ndarray], bool]],
            Tuple[MotionPlan, bool],
        ],
        with_watcher: bool = True,
    ) -> Tuple[Diagram, Context]:
        """
        Description
        -----------
        This function is used to easily cast and build the production.
        :param planning_algorithm: The algorithm that we will use to
        plan the motion.
        :param with_watcher: A Boolean that determines whether to add a watcher to the diagram.
        :return:
        """
        # Setup

        # Add all elements to the builder
        self.add_supporting_cast()
        # print("added supporting cast")

        # Create a planner from the algorithm
        prototypical_planner = PrototypicalPlannerSystem(
            self.plant, self.scene_graph,
            planning_algorithm,
            robot_model_idx=self.robot_model_index,
            controller_plant=self.station.controller_plant,
        )

        # Cast the production using the prototypical planner
        planner_role = self.suggested_roles()[0]

        # Fulfill each role-performer pair in the casting_call list
        self.fill_role(planner_role, prototypical_planner)
        print("filled role")

        self.create_optional_outputs_if_necessary(
            planner_role, prototypical_planner
        )

        # Build
        diagram, diagram_context = self.build_production(with_watcher=with_watcher)

        # Assign prototypical planner's context
        prototypical_planner.set_internal_root_context(diagram_context)

        return diagram, diagram_context

    def fill_role(
        self,
        role: Role,
        system: Performer,
    ):
        """
        Description
        -----------
        This method should be implemented by the subclass. It should add the
        system to the role.
        :param role:
        :param system:
        :return:
        """
        # Setup
        builder = self.builder

        # Call the member method of the role object
        role.connect_performer_ports_to(builder, system)

        # # Create a system for setting the arms into the proper initial configuration
        # initialize_robots_system = TriggerModelInitializationSystem(
        #     plant_and_model_idcs=[
        #         (self.plant, self.station.arm),
        #         (self.station.controller_plant, self.station.controller_arm),
        #     ]
        # )
        # initialize_robots_system = builder.AddSystem(initialize_robots_system)

        # # Connect the initialize_robots_system to the plant and other systems
        # builder.Connect(
        #     system.GetOutputPort("motion_plan"),
        #     initialize_robots_system.GetInputPort("plan"),
        # )        

        # self.builder.Connect(
        #     self.plan_ready_source.get_output_port(),
        #     initialize_robots_system.GetInputPort("trigger_initialize"),
        # )

        # Save the performer
        self.performers.append(system)

    @property
    def goal_configuration(self) -> np.ndarray:
        """
        Get the goal pose. This should be defined by the subclass.
        :return:
        """
        if self._goal_config is not None:
            return self._goal_config
        elif self._goal_pose is not None:
            # Use the goal pose to get the goal configuration
            # Using the IK solver (potentially buggy because default ik problem ignores obstacles)
            return self.solve_pose_ik_problem(self._goal_pose)
        else:
            raise NotImplementedError(
                "This function should be implemented by the subclass."
            )

    @property
    def goal_pose(self) -> RigidTransform:
        """
        Get the goal pose. This should be defined by the subclass.
        :return:
        """
        if self._goal_pose is not None:
            return self._goal_pose
        elif self._goal_config is not None:
            # The user would need to define a forward kinematics function,
            # to convert the goal configuration to a goal pose.
            raise NotImplementedError(
                "It looks like you are trying to get the goal pose from the goal configuration.\n" +
                "This is not implemented (yet!) because it would require solving the forward kinematics" +
                "problem for your specific scene.\n" +
                "Please implement your own goal_pose() function with a custom forward kinamtics function" +
                " (see ChemLab2's forward kinematics function) to convert the goal configuration to a goal pose."
            )
            #TODO(kwesi): Is there a way to do this without knowing about the user's robot?
        else:
            raise NotImplementedError(
                "This function should be implemented by the subclass."
            )

    @property
    def robot_model_index(self) -> ModelInstanceIndex:
        """
        Get the robot model index. This should be defined by the subclass.
        :return:
        """
        if self.robot_model_idx_ is not None:
            return self.robot_model_idx_
        else:        
            raise NotImplementedError(
                "This function should be implemented by the subclass."
            )

    @property
    def start_configuration(self) -> np.ndarray:
        """
        Description:
            This function returns the start and goal poses as a single array.
        """
        if self._start_config is not None:
            return self._start_config
        elif self._start_pose is not None:
            # Use the start pose to get the start configuration
            # Using the IK solver (potentially buggy because default ik problem ignores obstacles)
            return self.solve_pose_ik_problem(self._start_pose)
        else:
            raise NotImplementedError(
                "This function should be implemented by the subclass."
            )
    
    @property
    def start_pose(self) -> RigidTransform:
        """
        Get the start pose. This should be defined by the subclass.
        :return:
        """
        if self._start_pose is not None:
            return self._start_pose
        elif self._start_config is not None:
            # The user would need to define a forward kinematics function,
            # to convert the start configuration to a start pose.
            raise NotImplementedError(
                "It looks like you are trying to get the start pose from the start configuration.\n" +
                "This is not implemented (yet!) because it would require solving the forward kinematics" +
                "problem for your specific scene.\n" +
                "Please implement your own start_pose() function with a custom forward kinamtics function" +
                " (see ChemLab2's forward kinematics function) to convert the start configuration to a start pose."
            )
            #TODO(kwesi): Is there a way to do this without knowing about the user's robot?
        else:
            raise NotImplementedError(
                "This function should be implemented by the subclass."
            )

    def suggested_roles(self):
        return [kKinematicMotionPlanner]

    def solve_pose_ik_problem(
        self,
        pose_WorldTarget: RigidTransform,
        frame_name: str = "ft_frame",
        robot_joint_names: list = None,
    ) -> np.ndarray:
        """
        Description:
        :param input_pose_vec: An n-dimensional vector
        :return:
        """
        # Input Processing
        assert self.plant is not None, "Plant must be defined before solving IK problem!"

        # Setup

        # Define Problem
        ik_problem = self.define_pose_ik_problem(
            pose_WorldTarget,
            frame_name,
        )

        # Solve problem
        ik_program = ik_problem.prog()
        ik_result = Solve(ik_program)

        assert ik_result.get_solution_result() == SolutionResult.kSolutionFound, \
            f"Solution result was {ik_result.get_solution_result()}; need SolutionResult.kSolutionFound to make RRT Plan!"

        q_solution = ik_result.get_x_val()
        # print(f"solved ik problem: {q_solution}")

        # Extract only the positions that correspond to our robot's joints
        if robot_joint_names is None:
            robot_joint_names = self.plant.GetPositionNames(self.robot_model_index, add_model_instance_prefix=True)
        
        all_joint_names = self.plant.GetPositionNames()
        q_out_list = []
        for ii, joint_name in enumerate(all_joint_names):
            if joint_name in robot_joint_names:
                q_out_list.append(q_solution[ii])

        # print(f"q_out_list: {q_out_list}")

        return np.array(q_out_list)