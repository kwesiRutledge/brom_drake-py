from importlib import resources as impresources
from typing import Tuple, List
import unittest

import numpy as np
from pydrake.all import (
    RigidTransform, Quaternion,
    MultibodyPlant, Parser, LeafSystem,
    Simulator, DiagramBuilder,
    ModelInstanceIndex,
    ConstantVectorSource
)
from pydrake.common.value import AbstractValue
from pydrake.geometry import SceneGraph
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.primitives import ConstantValueSource

from brom_drake.watchers.port_watcher.port_watcher_options import FigureNamingConvention
from brom_drake.all import add_watcher_and_build
from brom_drake.motion_planning.algorithms.rrt.base import BaseRRTPlannerConfig
# Internal Imports
from brom_drake.motion_planning.systems.open_loop_dispensers.open_loop_plan_dispenser import OpenLoopPlanDispenser
from brom_drake.motion_planning.systems.rrt_plan_generator import RRTPlanGenerator
import brom_drake.robots as robots
from brom_drake.file_manipulation.urdf import drakeify_my_urdf


class RRTPlanGeneratorTest(unittest.TestCase):
    @staticmethod
    def create_example_scene1() -> Tuple[
        DiagramBuilder,
        MultibodyPlant,
        SceneGraph,
        List[ModelInstanceIndex],
        LeafSystem,
        LeafSystem,
        LeafSystem,
        LeafSystem
    ]:
        """
        Description:
            This function creates an example scene for testing the RRT plan generator.
        :param self:
        :return:
        """
        # Setup
        builder = DiagramBuilder()

        # Create start and goal configurations
        start_configuration = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        goal_configuration = np.array([0.0, 0.0, -np.pi/4.0, 0.0, 0.0, 0.0])
        
        # Create a MultibodyPlant
        plant, scene_graph = AddMultibodyPlantSceneGraph(
            builder,
            time_step=1e-3,
        )

        # Add the UR10e
        urdf_file_path = str(
            impresources.files(robots) / "models/ur/ur10e.urdf"
        )

        new_urdf_path = drakeify_my_urdf(
            urdf_file_path,
            overwrite_old_logs=True,
            log_file_name="test_plan1.log",
        )

        ur_model_idcs = Parser(plant).AddModels(str(new_urdf_path))
        plant.WeldFrames(
            plant.world_frame(),
            plant.GetFrameByName("base_link", ur_model_idcs[0]),
        )
        plant.Finalize()

        # Create start and goal sources
        start_source = ConstantVectorSource(start_configuration)
        builder.AddSystem(start_source)

        goal_source = ConstantVectorSource(goal_configuration)
        builder.AddSystem(goal_source)

        # Also create a source which tells the planner what the model index
        # is of the of the robot is
        robot_model_source = ConstantValueSource(
            AbstractValue.Make(ur_model_idcs[0])
        )
        builder.AddSystem(robot_model_source)

        # Also create a plan dispenser, so that we can easily plot the outputs
        dispenser = OpenLoopPlanDispenser(
            plant.num_actuated_dofs(),
            speed=0.1,
        )
        builder.AddSystem(dispenser)

        return builder, plant, scene_graph, ur_model_idcs, start_source, goal_source, dispenser, robot_model_source

    def test_plan1(self):
        """
        Description:
            In this test, we verify that our RRT planner generates a valid RRT with nodes
            that are getting CLOSER to the goal configuration over time.
        :return:
        """
        # Setup
        (builder, plant, scene_graph,
         model_idcs, start_source, goal_source,
         dispenser, robot_model_idx_source) = self.create_example_scene1()

        # Create an RRTPlanGenerator system
        rrt_config = BaseRRTPlannerConfig(
            max_iterations=1000
        )
        plan_generator = RRTPlanGenerator(
            plant,
            scene_graph,
            model_idcs[0],
            rrt_config=rrt_config,
            dim_config=6,
        )
        builder.AddSystem(plan_generator)

        # Connect the start and goal sources to the RRTPlanGenerator
        builder.Connect(
            start_source.get_output_port(0),
            plan_generator.GetInputPort("start_configuration"),
        )
        builder.Connect(
            goal_source.get_output_port(0),
            plan_generator.GetInputPort("goal_configuration"),
        )

        # Connect the robot model index to the plan_generator
        builder.Connect(
            robot_model_idx_source.get_output_port(0),
            plan_generator.GetInputPort("robot_model_index"),
        )

        # Connect the planner's outputs to the dispenser
        builder.Connect(
            plan_generator.GetOutputPort("plan_is_ready"),
            dispenser.GetInputPort("plan_ready")
        )

        builder.Connect(
            plan_generator.GetOutputPort("motion_plan"),
            dispenser.GetInputPort("plan")
        )

        # Can we successfully build this?
        watcher, diagram, diagram_context = add_watcher_and_build(
            builder,
            figure_naming_convention=FigureNamingConvention.kHierarchical,
        )

        # Simulate for a short time
        simulator = Simulator(diagram, diagram_context)
        plan_generator.set_internal_root_context(diagram_context)

        simulator.Initialize()
        T_sim = 10.0
        N_steps = 100
        for ii in np.linspace(0, T_sim, N_steps):
            simulator.AdvanceTo(ii)


        self.assertTrue(True)

if __name__ == "__main__":
    unittest.main()