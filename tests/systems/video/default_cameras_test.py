from pathlib import Path
import shutil
import unittest

from pydrake.systems.analysis import Simulator

# Internal imports
from brom_drake.all import BaseProduction, CameraSpec
from brom_drake.motion_planning.algorithms.rrt.bidirectional_connect import (
    BidirectionalRRTConnectPlanner,
    BidirectionalRRTConnectPlannerConfig,
)
from brom_drake.productions.motion_planning.offline import ShelfPlanning1

OUTPUT_DIR = Path("./brom/test_videos/default_cameras")


class DefaultCamerasTest(unittest.TestCase):
    def setUp(self):
        shutil.rmtree(OUTPUT_DIR, ignore_errors=True)

    def test_base_production_default1(self):
        """
        Description
        -----------
        Every production has at least a generic overview camera.
        """
        cameras = BaseProduction().default_cameras()
        self.assertEqual([c.name for c in cameras], ["overview"])

    def test_add_cameras_without_arguments1(self):
        """
        Description
        -----------
        add_cameras() with no cameras uses the production's default_cameras(),
        including a camera attached to the robot (ShelfPlanning1's wrist view).
        """
        production = ShelfPlanning1(meshcat_port_number=None)
        production.add_cameras(output_dir=OUTPUT_DIR)

        planner = BidirectionalRRTConnectPlanner(
            production.arm,
            production.plant,
            production.scene_graph,
            config=BidirectionalRRTConnectPlannerConfig(steering_step_size=0.2),
        )
        diagram, diagram_context = production.easy_cast_and_build(
            planner.plan, with_watcher=False
        )
        simulator = Simulator(diagram, diagram_context)
        simulator.AdvanceTo(0.2)

        video_paths = production.save_videos()
        self.assertEqual(set(video_paths), {"overview", "top", "wrist"})
        for path in video_paths.values():
            self.assertTrue(Path(path).exists())

    def test_explicit_cameras_replace_defaults1(self):
        """
        Description
        -----------
        Passing cameras explicitly uses only those cameras.
        """
        production = ShelfPlanning1(meshcat_port_number=None)
        production.add_cameras(
            [CameraSpec("only", width=64, height=48)], output_dir=OUTPUT_DIR
        )
        self.assertEqual([c.name for c in production._camera_specs], ["only"])


if __name__ == "__main__":
    unittest.main()
