from importlib import resources as impresources
from typing import Callable, List, Optional, Tuple

import numpy as np
from pydrake.all import (
    AbstractValue,
    ConstantValueSource,
    ConstantVectorSource,
    Context,
    Diagram,
    InverseKinematics,
    Parser,
    RigidTransform,
    RollPitchYaw,
    RotationMatrix,
)

# Internal Imports
from brom_drake import robots
from brom_drake.control import GripperTarget
from brom_drake.file_manipulation.urdf import drakeify_my_urdf
from brom_drake.motion_planning.systems.async_planner import AsyncPlanningConfig
from brom_drake.motion_planning.systems.open_loop_dispensers.open_loop_plan_dispenser import (
    OpenLoopPlanDispenser,
)
from brom_drake.productions.ids import ProductionID
from brom_drake.productions.roles.role import Role
from brom_drake.productions.types import OfflineDynamicMotionPlanningProduction
from brom_drake.robots.gripper_type import GripperType
from brom_drake.stations.classical import KinovaGen3Station, KinovaGen3Variant
from brom_drake.systems.video import CameraSpec
from brom_drake.utils import AddGround, MotionPlan, Performer


class CornellTech1(OfflineDynamicMotionPlanningProduction):
    """
    *Description*

    A dynamic, offline motion planning production for a Kinova Gen3 arm (7-DoF or 6-DoF)
    mounted on a mobile robot cart, modeled after the robot workcell at Cornell Tech.

    The scene contains:

    - The Kinova Gen3 arm (with a Robotiq 2F-85 gripper, by default), whose base
      is at the world origin, mounted on the back strip of the cart
    - The robot cart (aluminum extrusion frame), whose top surface is at z = 0,
      along with an emergency stop and a cardboard box sitting on it
    - An office desk with two monitors behind the cart
    - A beaker (free body) sitting on the cart
    - The floor, 0.42 m below the top of the cart

    The robot must move its end effector from a start pose above the front-right
    corner of the cart to a goal pose above the beaker, using the full dynamics of
    the arm (just like :py:class:`ChemLab2`).

    *Coordinate Frames*

    The world frame's origin is at the base of the Kinova arm. +x points along the
    cart's length (from the cardboard box toward the arm), +y points toward the
    desk behind the cart, and +z points up.

    *Parameters*

    time_step: float, optional
        The time step for the production, by default 1e-3.

    meshcat_port_number: int, optional
        The port number for meshcat visualization, by default 7001.
        Use None to disable meshcat.

    plan_execution_speed: float, optional
        The speed at which to execute the motion plan, by default 0.3.

    pose_WorldBeaker: RigidTransform, optional
        The pose of the beaker in the world frame, by default None
        (the beaker sits on the cart, to the left of the arm).

    pose_WorldCart: RigidTransform, optional
        The pose of the cart's frame (center of its top surface) in the world frame,
        by default None (the cart is placed so that the arm sits on its back-right corner).

    pose_WorldDesk: RigidTransform, optional
        The pose of the desk's frame (on the floor, below the center of the desktop)
        in the world frame, by default None (just behind the cart).

    gripper_type: GripperType, optional
        The type of gripper to use, by default GripperType.Robotiq_2f_85.

    arm_variant: KinovaGen3Variant, optional
        Which Kinova Gen3 arm to use, by default KinovaGen3Variant.kSevenDof.
        Start and goal configurations (if given) must have ``arm_variant.num_dofs`` entries.

    *Usage*

    To use this production, create an instance of the CornellTech1 class,
    provide it with a motion planning function, and then build it. ::

        from pydrake.all import Simulator
        from brom_drake.motion_planning.algorithms.rrt.bidirectional_connect import (
            BidirectionalRRTConnectPlanner,
            BidirectionalRRTConnectPlannerConfig,
        )
        from brom_drake.productions import CornellTech1

        production = CornellTech1()

        planner = BidirectionalRRTConnectPlanner(
            production.arm,
            production.plant,
            production.scene_graph,
            config=BidirectionalRRTConnectPlannerConfig(steering_step_size=0.2),
        )

        diagram, diagram_context = production.easy_cast_and_build(
            planner.plan,
            with_watcher=True,
        )

        simulator = Simulator(diagram, diagram_context)
        simulator.Initialize()
        simulator.AdvanceTo(0.05)
        planned_trajectory = production.plan_dispenser.planned_trajectory
        simulator.AdvanceTo(planned_trajectory.end_time() + 1.0)
    """

    #: Height of the top of the cart above the floor (m)
    cart_height: float = 0.42

    def __init__(
        self,
        time_step: float = 1e-3,
        meshcat_port_number: int = 7001,
        plan_execution_speed: float = 0.3,
        pose_WorldBeaker: RigidTransform = None,
        pose_WorldCart: RigidTransform = None,
        pose_WorldDesk: RigidTransform = None,
        gripper_type: GripperType = GripperType.Robotiq_2f_85,
        arm_variant: KinovaGen3Variant = KinovaGen3Variant.kSevenDof,
        **kwargs,
    ):
        """
        *Description*

        Constructor for the CornellTech1 Production.
        See the class docstring for a description of the parameters.
        """
        # Superclass constructor
        super().__init__(**kwargs)

        # Input Processing
        self.time_step = time_step
        self.meshcat_port_number = meshcat_port_number
        self.plan_execution_speed = plan_execution_speed
        self.gripper_type = gripper_type
        self.arm_variant = arm_variant

        # Initialize pose data
        self.pose_WorldBeaker = pose_WorldBeaker
        self.pose_WorldCart = pose_WorldCart
        self.pose_WorldDesk = pose_WorldDesk
        self.initialize_pose_data()

        # Set station
        self.station = KinovaGen3Station(
            time_step=self.time_step,
            meshcat_port_number=self.meshcat_port_number,
            gripper_type=self.gripper_type,
            variant=self.arm_variant,
        )
        self.arm = self.station.arm
        self.robot_model_idx_ = self.arm

        # Record metadata about the robot
        self._metadata["robot_model"] = {
            "model_index": int(self.robot_model_idx_),
            "model_name": self.station.plant.GetModelInstanceName(
                self.robot_model_idx_
            ),
        }

        # Set Names of Plant and scene graph
        self.plant = self.station.plant
        self.scene_graph = self.station.scene_graph

        self.plant.set_name("CornellTech1_Production_Plant")
        self.scene_graph.set_name("CornellTech1_Production_SceneGraph")

        # Placeholders for the models in the scene
        self.cart_model_index = None
        self.desk_model_index = None
        self.beaker_model_index = None

    def initialize_pose_data(self):
        """
        *Description*

        Initialize the following poses if they are not already set:

        - pose_WorldCart
        - pose_WorldDesk
        - pose_WorldBeaker
        """
        # The arm sits on the cart's back strip, 0.375 m from its right end
        # (the cart frame is at the center of the cart's top surface)
        if self.pose_WorldCart is None:
            self.pose_WorldCart = RigidTransform(np.array([-0.375, -0.31, 0.0]))

        # The desk stands on the floor just behind the cart
        if self.pose_WorldDesk is None:
            self.pose_WorldDesk = RigidTransform(
                np.array([-0.55, 0.475, -self.cart_height])
            )

        # The beaker sits on the cart, to the left of the arm
        # (the beaker mesh's "up" direction is +y, hence the roll)
        if self.pose_WorldBeaker is None:
            self.pose_WorldBeaker = RigidTransform(
                RollPitchYaw(np.pi / 2.0, 0.0, 0.0),
                np.array([-0.55, -0.20, 0.012]),
            )

    def add_supporting_cast(self):
        """
        *Description*

        This method adds all secondary cast members to the builder.
        The secondary cast members in the production are the:

        - Robot cart (with the items on it), where the robot is mounted
        - Office desk behind the cart
        - Beaker
        - Ground
        - Motion Planning components (e.g., dispensers, etc.)
        - Start and Goal sources
        """
        # Call the superclass method (adds start and goal visualizations)
        super().add_supporting_cast()

        # Create the environment
        self.add_cart()
        self.add_desk()
        self.add_beaker()
        AddGround(self.plant, z_ground=-self.cart_height)

        # Add the station
        self.builder.AddSystem(self.station)
        self.station.Finalize()

        # Add the motion planning components
        self.add_robot_source_system()
        self.add_motion_planning_components()
        self.add_start_and_goal_sources_to_builder()

        # Add initial conditions for the arm
        self.initial_condition_manager.add_initial_configuration(
            model_instance_index=self.arm,
            configuration=self.start_configuration,
        )

        print("Completed adding supporting cast members.")

    def add_beaker(self):
        """
        *Description*

        Adds the beaker (a free body) to the production's plant.
        """
        urdf_file_path = str(impresources.files(robots) / "models/beaker/beaker.urdf")
        new_beaker_urdf = drakeify_my_urdf(urdf_file_path)

        self.beaker_model_index = Parser(plant=self.plant).AddModels(
            str(new_beaker_urdf)
        )[0]

        # The beaker's initial pose is set when the production is built
        self.models_in_supporting_cast.append(
            (self.beaker_model_index, self.pose_WorldBeaker),
        )

    def add_cart(self):
        """
        *Description*

        Adds the robot cart (and the e-stop and cardboard box on top of it)
        to the production's plant and welds it to the world frame.
        """
        sdf_path = str(impresources.files(robots) / "models/cornell_tech/robot_cart.sdf")
        self.cart_model_index = Parser(plant=self.plant).AddModels(sdf_path)[0]
        self.plant.WeldFrames(
            self.plant.world_frame(),
            self.plant.GetFrameByName("cart_body", self.cart_model_index),
            self.pose_WorldCart,
        )

    def add_desk(self):
        """
        *Description*

        Adds the office desk (with its monitors) to the production's plant
        and welds it to the world frame.
        """
        sdf_path = str(
            impresources.files(robots) / "models/cornell_tech/office_desk.sdf"
        )
        self.desk_model_index = Parser(plant=self.plant).AddModels(sdf_path)[0]
        self.plant.WeldFrames(
            self.plant.world_frame(),
            self.plant.GetFrameByName("desk_body", self.desk_model_index),
            self.pose_WorldDesk,
        )

    def add_motion_planning_components(self):
        """
        *Description*

        Add the following components to the production's
        ``builder`` in order to help with motion planning:

        - `plan_dispenser`: An OpenLoopPlanDispenser which will describe the reference robot trajectory.
        - Connections between reference trajectory and the Kinova Gen3 station.
        - Gripper control components (type of control and an "open" command)
        """
        n_actuated_dof = self.plant.num_actuated_dofs(self.arm)

        # Add the Plan Dispenser and connect it to the station
        self.plan_dispenser = self.builder.AddSystem(
            OpenLoopPlanDispenser(n_actuated_dof, self.plan_execution_speed)
        )
        self.builder.Connect(
            self.plan_dispenser.GetOutputPort("point_in_plan"),
            self.station.GetInputPort("desired_joint_positions"),
        )

        if self.gripper_type != GripperType.NoGripper:
            # Keep the gripper open (position control with a constant target)
            gripper_target_type_source = self.builder.AddSystem(
                ConstantValueSource(AbstractValue.Make(GripperTarget.kPosition)),
            )
            self.builder.Connect(
                gripper_target_type_source.get_output_port(),
                self.station.GetInputPort("gripper_target_type"),
            )

            gripper_target_source = self.builder.AddSystem(
                ConstantVectorSource(np.array([0.0])),
            )
            self.builder.Connect(
                gripper_target_source.get_output_port(),
                self.station.GetInputPort("gripper_target"),
            )

    def add_cast_and_build(
        self,
        cast: List[Tuple[Role, Performer]] = [],
        with_watcher: bool = False,
    ) -> Tuple[Diagram, Context]:
        """
        *Description*

        Modifies the normal add_cast_and_build, so that
        we share the context of the diagram with the motion planner.

        *Parameters*

        cast: List[Tuple[Role, Performer]], optional
            The cast to add to the production, by default [].

        with_watcher: bool, optional
            A Boolean that determines whether to add a watcher to the diagram, by default False.

        *Returns*

        diagram: Diagram
            The built diagram.

        diagram_context: Context
            The context for the built diagram.
        """
        diagram, diagram_context = super().add_cast_and_build(
            main_cast_members=cast,
            with_watcher=with_watcher,
        )

        for role_ii, performer_ii in cast:
            if role_ii.name == "OfflineMotionPlanner":
                performer_ii.set_internal_root_context(diagram_context)

        return diagram, diagram_context

    def define_pose_ik_problem(
        self,
        pose_WorldTarget: RigidTransform,
        target_frame_name: str,
        eps0: float = 2.5e-2,
        orientation_cost_scaling: float = 0.25,
    ) -> InverseKinematics:
        """
        *Description*

        Sets up the inverse kinematics problem for reaching pose_WorldTarget
        with the frame named target_frame_name.
        The orientation must be within pi/8 radians of the target's.
        """
        ik_problem = InverseKinematics(self.plant)

        ik_problem.AddPositionConstraint(
            self.plant.world_frame(),
            pose_WorldTarget.translation(),
            self.plant.GetFrameByName(target_frame_name, self.arm),
            (-np.ones((3,)) * eps0).reshape((-1, 1)),
            (+np.ones((3,)) * eps0).reshape((-1, 1)),
        )
        ik_problem.AddOrientationConstraint(
            self.plant.world_frame(),
            pose_WorldTarget.rotation(),
            self.plant.GetFrameByName(target_frame_name, self.arm),
            RotationMatrix.Identity(),
            np.pi / 8.0,
        )

        return ik_problem

    def solve_pose_ik_problem(
        self,
        pose_WorldTarget: RigidTransform,
        frame_name: str = "end_effector",
        robot_joint_names: list = None,
    ) -> np.ndarray:
        """
        *Description*

        Solves the inverse kinematics problem for the Kinova arm.
        By default, the target frame is the station's ``end_effector`` frame
        (located between the gripper's fingertips).
        """
        return super().solve_pose_ik_problem(
            pose_WorldTarget,
            frame_name=frame_name,
            robot_joint_names=robot_joint_names,
        )

    def easy_cast_and_build(
        self,
        planning_algorithm: Callable[
            [np.ndarray, np.ndarray, Callable[[np.ndarray], bool]],
            Tuple[MotionPlan, bool],
        ],
        with_watcher: bool = False,
        async_planning: Optional[AsyncPlanningConfig] = None,
    ) -> Tuple[Diagram, Context]:
        """
        *Description*

        This function is used to easily cast and build the production.

        *Parameters*

        planning_algorithm: Callable[[np.ndarray, np.ndarray, Callable[[np.ndarray], bool]],Tuple[MotionPlan, bool]]
            The algorithm that we will use to plan the motion.

        with_watcher: bool, optional
            A Boolean that determines whether to add a watcher to the diagram,
            by default False.

        async_planning: AsyncPlanningConfig, optional
            If given, plan in the background while the simulation runs.

        *Returns*

        diagram: Diagram
            The built diagram.

        diagram_context: Context
            The context for the built diagram.
        """
        diagram, diagram_context = super().easy_cast_and_build(
            planning_algorithm,
            with_watcher=with_watcher,
            async_planning=async_planning,
        )

        self.performers[0].set_internal_root_context(diagram_context)

        return diagram, diagram_context

    def default_cameras(self) -> List[CameraSpec]:
        """
        *Description*

        The cameras used by ``add_cameras()`` when no cameras are given.
        A view from the front of the cart (similar to the reference photo)
        and an angled overview.
        """
        return [
            CameraSpec.looking_at(
                "front",
                position=[-0.4, -2.4, 0.6],
                target=[-0.4, 0.0, 0.1],
                fov_y=np.deg2rad(50),
            ),
            CameraSpec.looking_at(
                "overview",
                position=[1.4, -1.6, 1.2],
                target=[-0.3, 0.0, 0.1],
                fov_y=np.deg2rad(45),
            ),
        ]

    @property
    def goal_configuration(self) -> np.ndarray:
        """
        *Description*

        Get the goal configuration of the arm. If it was not given, it is computed
        from the goal pose with inverse kinematics.

        *Returns*

        q_goal: np.ndarray
            The goal configuration of the robot.
        """
        if self._goal_config is None:
            self._goal_config = self.solve_pose_ik_problem(self.goal_pose)
        return self._goal_config

    @property
    def goal_pose(self) -> RigidTransform:
        """
        *Description*

        Get the goal pose of the end effector frame.
        By default, the end effector points straight down, 0.2 m above the
        bottom of the beaker.

        *Returns*

        pose_WorldGoal: RigidTransform
            The goal pose of the end effector frame.
        """
        if self._goal_pose is None:
            if self._goal_config is not None:
                self._goal_pose = self.solve_forward_kinematics_problem_for_arm(
                    self._goal_config
                )
            else:
                self._goal_pose = RigidTransform(
                    RollPitchYaw(np.pi, 0.0, 0.0),
                    self.pose_WorldBeaker.translation() + np.array([0.0, 0.0, 0.2]),
                )
        return self._goal_pose

    @property
    def id(self) -> ProductionID:
        """
        *Description*

        Always returns the ProductionID for CornellTech1.
        """
        return ProductionID.kCornellTech1

    def solve_forward_kinematics_problem_for_arm(
        self,
        robot_joint_positions: np.ndarray,
    ) -> RigidTransform:
        """
        *Description*

        Computes the pose of the arm's ``end_effector`` frame for the given
        joint positions, using a standalone Kinova Gen3 station.
        """
        shadow_station = KinovaGen3Station(
            time_step=self.time_step,
            meshcat_port_number=None,
            gripper_type=self.gripper_type,
            variant=self.arm_variant,
        )
        shadow_station.Finalize()
        shadow_plant = shadow_station.plant

        temp_context = shadow_plant.CreateDefaultContext()
        shadow_plant.SetPositions(
            temp_context,
            shadow_station.arm,
            robot_joint_positions,
        )

        return shadow_plant.GetFrameByName(
            "end_effector", shadow_station.arm
        ).CalcPoseInWorld(temp_context)

    @property
    def start_configuration(self) -> np.ndarray:
        """
        *Description*

        Get the start configuration of the arm. If it was not given, it is computed
        from the start pose with inverse kinematics.

        *Returns*

        q_start: np.ndarray
            The start configuration of the robot.
        """
        if self._start_config is None:
            self._start_config = self.solve_pose_ik_problem(self.start_pose)
        return self._start_config

    @property
    def start_pose(self) -> RigidTransform:
        """
        *Description*

        Get the start pose of the end effector frame.
        By default, the end effector points straight down, above the
        front-right corner of the cart.

        *Returns*

        pose_WorldStart: RigidTransform
            The start pose of the end effector frame.
        """
        if self._start_pose is None:
            if self._start_config is not None:
                self._start_pose = self.solve_forward_kinematics_problem_for_arm(
                    self._start_config
                )
            else:
                self._start_pose = RigidTransform(
                    RollPitchYaw(np.pi, 0.0, 0.0),
                    np.array([0.15, -0.45, 0.20]),
                )
        return self._start_pose
