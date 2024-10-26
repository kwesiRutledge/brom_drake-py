from distutils.command.build import build
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

from brom_drake.PortWatcher.port_watcher_options import FigureNamingConvention
from brom_drake.all import add_watcher_and_build
# Internal Imports
from brom_drake.motion_planning.algorithms.rrt.base import BaseRRTPlanner
from brom_drake.motion_planning.systems.open_loop_plan_dispenser import OpenLoopPlanDispenser
from brom_drake.motion_planning.systems.rrt_plan_generator import RRTPlanGenerator
import brom_drake.robots as robots
from brom_drake.urdf import drakeify_my_urdf


class RRTPlanGeneratorTest(unittest.TestCase):
    @staticmethod
    def create_example_scene1() -> Tuple[
        DiagramBuilder,
        MultibodyPlant,
        List[ModelInstanceIndex],
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

        # Create start pose
        start_position = np.array([-0.2, 0.9, 0.3])
        start_orientation = Quaternion(1, 0, 0, 0)
        p_WStart = RigidTransform(start_orientation, start_position)

        # Create goal pose
        goal_position = np.array([+0.2, 1.0, 0.65])
        goal_orientation = Quaternion(1, 0, 0, 0)
        p_WGoal = RigidTransform(goal_orientation, goal_position)

        # Create a MultibodyPlant
        plant = MultibodyPlant(time_step=1e-3)

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

        builder.AddSystem(plant)

        # Create start and goal sources
        start_source = ConstantVectorSource(
            np.hstack((start_position, start_orientation.wxyz()))
        )
        builder.AddSystem(start_source)

        goal_source = ConstantVectorSource(
            np.hstack((goal_position, goal_orientation.wxyz()))
        )
        builder.AddSystem(goal_source)

        # Also create a plan dispenser, so that we can easily plot the outputs
        dispenser = OpenLoopPlanDispenser(
            plant.num_actuated_dofs(),
            speed=0.1,
        )
        builder.AddSystem(dispenser)

        return builder, plant, ur_model_idcs, start_source, goal_source, dispenser

    def test_plan1(self):
        """
        Description:
            In this test, we verify that our RRT planner generates a valid RRT with nodes
            that are getting CLOSER to the goal configuration over time.
        :return:
        """
        # Setup
        builder, plant, model_idcs, start_source, goal_source, dispenser = self.create_example_scene1()

        # Create an RRTPlanGenerator system
        plan_generator = RRTPlanGenerator(
            plant,
            model_idcs[0],
            max_rrt_iterations=1000,
        )
        builder.AddSystem(plan_generator)

        # Connect the start and goal sources to the RRTPlanGenerator
        builder.Connect(
            start_source.get_output_port(0),
            plan_generator.GetInputPort("start_pose"),
        )
        builder.Connect(
            goal_source.get_output_port(0),
            plan_generator.GetInputPort("goal_pose"),
        )

        # Connect the planner's outputs to the dispenser
        builder.Connect(
            plan_generator.GetOutputPort("plan_is_ready"),
            dispenser.GetInputPort("plan_ready")
        )

        builder.Connect(
            plan_generator.GetOutputPort("plan"),
            dispenser.GetInputPort("plan")
        )

        # Can we successfully build this?
        watcher, diagram, diagram_context = add_watcher_and_build(
            builder,
            figure_naming_convention=FigureNamingConvention.kHierarchical,
        )

        # Simulate for a short time
        simulator = Simulator(diagram, diagram_context)
        simulator_context = simulator.get_mutable_context()
        plant_context = plant.GetMyContextFromRoot(diagram_context)

        plan_generator.plant_context = plant_context

        simulator.Initialize()
        T_sim = 10.0
        N_steps = 100
        for ii in np.linspace(0, T_sim, N_steps):
            simulator.AdvanceTo(ii)


        self.assertTrue(True)

if __name__ == "__main__":
    unittest.main()