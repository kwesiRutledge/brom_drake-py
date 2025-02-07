from importlib import resources as impresources
import networkx as nx
import numpy as np
from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    MultibodyPlant,
    Parser,
    DiagramBuilder,
)
import unittest

# Internal Imports
from brom_drake.all import drakeify_my_urdf
from brom_drake import robots
from brom_drake.motion_planning.algorithms.rrt.bidirectional import (
    BidirectionalRRTPlanner,
    BidirectionalRRTPlannerConfig,
    BiRRTSamplingProbabilities,
)
from brom_drake.motion_planning import PlanningNode

class BidirectionalRRTPlannerTest(unittest.TestCase):
    def setUp(self):
        """
        Description
        -----------
        This function sets up the test case.
        """
        # Setup

        # Create plant and scene graph
        builder = DiagramBuilder()
        self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(
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
            log_file_name="drakeify-my-urdf1.log",
        )

        model_idcs = Parser(self.plant).AddModels(str(new_urdf_path))
        self.arm_model_idx = model_idcs[0]
        self.plant.WeldFrames(
            self.plant.world_frame(),
            self.plant.GetFrameByName("base_link", self.arm_model_idx),
        )
        self.plant.Finalize()

        # Build the diagram
        self.diagram = builder.Build()
        self.root_context = self.diagram.CreateDefaultContext()

    def test_sample_from_tree1(self):
        """
        Description
        -----------
        This function tests the sample_from_tree function of the BidirectionalRRTPlanner.
        We want to verify that the sampling function
        returns a node that is in the tree.
        """
        # Setup
        rrt0 = nx.DiGraph()
        rrt0.add_node(
            0,
            q=np.array([0, 0, 0, 0, 0, 0]),
        )
        rrt0.add_node(
            1,
            q=np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1]),
        )


        # Create Planner
        planner = BidirectionalRRTPlanner(
            self.arm_model_idx,
            self.plant,
            self.scene_graph,
        )

        # Sample from the tree
        sampled_node = planner.sample_from_tree(rrt0)

        # Check that the sampled node is in the tree
        found_node = False
        for node in rrt0.nodes:
            if np.allclose(rrt0.nodes[sampled_node]['q'], rrt0.nodes[node]['q']):
                found_node = True
                break

        self.assertTrue(found_node)

    def test_sample_nearest_in_tree1(self):
        """
        Description
        -----------
        This function tests the sample_nearest_in_tree function of the BidirectionalRRTPlanner.
        We want to verify that the sampling function truly gets the closest
        configuration to the one we provide it by giving a rrt with 2
        nodes in it (the second node should be closest to the query point).
        Then, we check that the sampled node is the one that we expect.
        """
        # Setup
        rrt0 = nx.DiGraph()
        rrt0.add_node(
            0,
            q=np.array([0, 0, 0, 0, 0, 0])
        )
        rrt0.add_node(
            1,
            q=np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
        )

        # Create Planner
        planner = BidirectionalRRTPlanner(
            self.arm_model_idx,
            self.plant,
            self.scene_graph,
        )

        # Sample from the tree
        sampled_node, min_dist = planner.sample_nearest_in_tree(
            rrt0,
            np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1]),
        )

        # Check that the sampled node is in the tree
        node_idcs = [ idx for idx in rrt0.nodes ]
        self.assertTrue(
            np.allclose(
                rrt0.nodes[sampled_node]['q'],
                rrt0.nodes[list(rrt0.nodes)[1]]['q']
            )
        )

        # Check that minimum distance is close to zero
        self.assertTrue(np.isclose(min_dist, 0.0))

    def test_sample_nearest_in_tree2(self):
        """
        Description
        -----------
        This function tests the sample_nearest_in_tree function of the BidirectionalRRTPlanner.
        We want to verify that the sampling function truly gets the closest
        configuration to the one we provide it by giving a rrt with 1
        node in it. Then, we check that the sampled node is the one that we expect.
        """
        # Setup
        rrt0 = nx.DiGraph()
        rrt0.add_node(
            0,
            q=np.array([0, 0, 0, 0, 0, 0]),
        )

        # Create Planner
        planner = BidirectionalRRTPlanner(
            self.arm_model_idx,
            self.plant,
            self.scene_graph,
        )

        # Sample from the tree
        sampled_node, min_dist = planner.sample_nearest_in_tree(
            rrt0,
            np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1]),
        )

        # Check that the sampled node is in the tree
        self.assertTrue(
            np.allclose(
                rrt0.nodes[sampled_node]['q'],
                rrt0.nodes[list(rrt0.nodes)[0]]['q']
            )
        )

        # Check that minimum distance is close to zero
        self.assertFalse(np.isclose(min_dist, 0.0))
    

    def test_steer_towards_tree1(self):
        """
        Description
        -----------
        This function tests the steer_towards_tree function of the BidirectionalRRTPlanner.
        We want to verify that the steering function truly steers towards
        the opposite tree when the opposite tree is the GOAL tree.
        We will make the trees far enough apart that the steering function
        will not make it to the next tree.
        """
        # Setup

        # Create rrt_start with three nodes
        rrt_start = nx.DiGraph()
        rrt_start.add_node(
            0,
            q=np.array([0, 0, 0, 0, 0, 0]),
        )
        rrt_start.add_node(
            1,
            q=np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1]),
        )
        rrt_start.add_node(
            2,
            q=np.array([0.2, 0.2, 0.2, 0.2, 0.2, 0.2]),
        )

        # Create rrt_goal with one node
        rrt_goal = nx.DiGraph()
        rrt_goal.add_node(
            0,
            q=np.array([1, 1, 1, 1, 1, 1]),
        )

        # Create Planner
        planner = BidirectionalRRTPlanner(
            self.arm_model_idx,
            self.plant,
            self.scene_graph,
        )

        # Steer towards the goal tree
        planner.root_context = self.diagram.CreateDefaultContext()
        combined_rrt, rrts_touch = planner.steer_towards_tree(
            rrt_start,
            rrt_goal,
            current_tree_is_goal=False,
        )

        # Check that the rrt_start has 4 nodes and rrt_goal has 1 node
        self.assertEqual(len(rrt_start.nodes), 4)
        self.assertEqual(len(rrt_goal.nodes), 1)

        # Verify that the rrts DO NOT CONNECT in this step
        self.assertFalse(rrts_touch)
        self.assertIsNone(combined_rrt)

        # Check that the connection that is made is from the second to last node of
        # rrt_start to the last node of rrt_start
        node2, node3 = None, None
        for ii, node_ii in enumerate(rrt_start.nodes):
            if ii == 2:
                node2 = node_ii
            elif ii == 3:
                node3 = node_ii
        
        self.assertTrue(
            node3 in [elt for elt in rrt_start.neighbors(node2)]
        )

    def test_steer_towards_tree2(self):
        """
        Description
        -----------
        This function tests the steer_towards_tree function of the BidirectionalRRTPlanner.
        We want to verify that the steering function truly steers towards
        the opposite tree when the opposite tree is the START tree.
        We will make the trees far enough apart that the steering function
        will not make it to the next tree.
        """
        # Setup

        # Create rrt_start with three nodes
        rrt_start = nx.DiGraph()
        rrt_start.add_node(
            0,
            q=np.array([0, 0, 0, 0, 0, 0]),
        )
        rrt_start.add_node(
            1,
            q=np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1]),
        )
        rrt_start.add_node(
            2,
            q=np.array([0.2, 0.2, 0.2, 0.2, 0.2, 0.2]),
        )

        # Create rrt_goal with one node
        rrt_goal = nx.DiGraph()
        rrt_goal.add_node(
            0,
            q=np.array([0.5, 0.5, 0.5, 0.5, 0.5, 0.5]),
        )

        # Create Planner
        planner = BidirectionalRRTPlanner(
            self.arm_model_idx,
            self.plant,
            self.scene_graph,
        )

        # Steer towards the start tree
        planner.root_context = self.diagram.CreateDefaultContext()
        combined_rrt, rrts_touch = planner.steer_towards_tree(
            rrt_start,
            rrt_goal,
            current_tree_is_goal=True,
        )

        # Check that the rrt_start has 3 nodes and rrt_goal has 2 nodes
        self.assertEqual(rrt_start.number_of_nodes(), 3)
        self.assertEqual(rrt_goal.number_of_nodes(), 2)

        # Verify that the rrts DO NOT CONNECT in this step
        self.assertFalse(rrts_touch)
        self.assertIsNone(combined_rrt)

        # Check that the connection that is made is from the second to last node of
        # rrt_start to the last node of rrt_start
        node1, node0 = None, None
        for ii, node_ii in enumerate(rrt_goal.nodes):
            if ii == 1:
                node1 = node_ii
            elif ii == 0:
                node0 = node_ii
        
        self.assertTrue(
            node0 in [elt for elt in rrt_goal.neighbors(node1)]
        )

    def test_steer_towards_tree3(self):
        """
        Description
        -----------
        This function tests the steer_towards_tree function of the BidirectionalRRTPlanner.
        We want to verify that the steering function truly steers towards
        the opposite tree when the opposite tree is the GOAL tree.
        We will make the trees far enough apart that the steering function
        will need to connect the trees.
        """
        # Setup

        # Create rrt_start with three nodes
        rrt_start = nx.DiGraph()
        rrt_start.add_node(
            0,
            q=np.array([0, 0, 0, 0, 0, 0]),
        )
        rrt_start.add_node(
            1,
            q=np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1]),
        )
        rrt_start.add_edge(0, 1)
        rrt_start.add_node(
            2,
            q=np.array([0.2, 0.2, 0.2, 0.2, 0.2, 0.2]),
        )
        rrt_start.add_edge(1, 2)

        # Create rrt_goal with one node
        rrt_goal = nx.DiGraph()
        rrt_goal.add_node(
            0,
            q=np.array([0.21, 0.21, 0.21, 0.21, 0.21, 0.21]),
        )

        # Create Planner
        planner = BidirectionalRRTPlanner(
            self.arm_model_idx,
            self.plant,
            self.scene_graph,
        )

        # Steer towards the start tree
        planner.root_context = self.diagram.CreateDefaultContext()
        combined_rrt, rrts_touch = planner.steer_towards_tree(
            rrt_start,
            rrt_goal,
            current_tree_is_goal=False,
        )

        # Check that the rrt_start has 3 nodes and rrt_goal has 1 nodes
        self.assertEqual(rrt_start.number_of_nodes(), 3)
        self.assertEqual(rrt_goal.number_of_nodes(), 1)

        # Verify that the rrts DO NOT CONNECT in this step
        self.assertTrue(rrts_touch)
        self.assertIsNotNone(combined_rrt)

        # Check that there exists a path from the last node of the combined
        # rrt to the first node of the combined rrt
        self.assertTrue(
            nx.has_path(combined_rrt, 0, 3)
        )

    def test_plan1(self):
        """
        Description
        -----------
        In this test, we will include one of the small problems from above
        and verify that it can be solved by the planner.
        This should be possible in a bounded number of iterations if we force the 
        planner to always sample the opposite tree.
        """
        # Setup
        q0 = np.array([0, 0, 0, 0, 0, 0])
        q_goal = np.array([0.21, 0.21, 0.21, 0.21, 0.21, 0.21])

        # Create Planner
        planner = BidirectionalRRTPlanner(
            self.arm_model_idx,
            self.plant,
            self.scene_graph,
            config=BidirectionalRRTPlannerConfig(
                steering_step_size=0.2,
                probabilities=BiRRTSamplingProbabilities(
                    sample_opposite_tree=1.0,
                )
            ),
        )

        # Plan
        plan, success = planner.plan(
            q0,
            q_goal,
            lambda q: False,
        )

        # Verify that plan was successfully created
        self.assertTrue(success)
        self.assertIsNotNone(plan)

if __name__ == "__main__":
    unittest.main()
