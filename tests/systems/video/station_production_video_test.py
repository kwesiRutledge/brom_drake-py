from pathlib import Path
import shutil
import unittest

from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder

# Internal imports
from brom_drake.all import BaseProduction, CameraSpec
from brom_drake.motion_planning.algorithms.rrt.bidirectional_connect import (
    BidirectionalRRTConnectPlanner,
    BidirectionalRRTConnectPlannerConfig,
)
from brom_drake.productions.motion_planning.offline import ShelfPlanning1
from brom_drake.productions.motion_planning.offline.dynamic.chem_lab2 import ChemLab2

OUTPUT_DIR = Path("./brom/test_videos/stations")


def _count_frames(path: Path) -> int:
    import imageio.v2 as imageio

    reader = imageio.get_reader(str(path))
    n_frames = reader.count_frames()
    reader.close()
    return n_frames


def _overview_camera() -> CameraSpec:
    return CameraSpec.looking_at(
        "overview",
        position=[2.0, -2.0, 1.8],
        target=[0.3, 0.0, 0.4],
        width=96,
        height=64,
        fps=10,
    )


class StationProductionVideoTest(unittest.TestCase):
    def setUp(self):
        shutil.rmtree(OUTPUT_DIR, ignore_errors=True)

    def _record(self, production, sub_dir: str, duration: float) -> Path:
        planner = BidirectionalRRTConnectPlanner(
            production.arm,
            production.plant,
            production.scene_graph,
            config=BidirectionalRRTConnectPlannerConfig(steering_step_size=0.2),
        )
        production.add_cameras([_overview_camera()], output_dir=OUTPUT_DIR / sub_dir)
        diagram, diagram_context = production.easy_cast_and_build(
            planner.plan,
            with_watcher=False,
        )

        simulator = Simulator(diagram, diagram_context)
        simulator.Initialize()
        simulator.AdvanceTo(duration)

        video_paths = production.save_videos()
        self.assertIn("overview", video_paths)
        return video_paths["overview"]

    def test_kinematic_station1(self):
        """
        Description
        -----------
        A production built around the KINEMATIC UR10e station (ShelfPlanning1)
        can record a video, even though its SceneGraph lives inside the station.
        """
        path = self._record(ShelfPlanning1(meshcat_port_number=None), "shelf", 1.0)
        self.assertEqual(_count_frames(path), 11)

    def test_classical_station1(self):
        """
        Description
        -----------
        A production built around the CLASSICAL UR10e station (ChemLab2) can
        record a video. This also checks that the Robotiq gripper's visual
        meshes can be loaded by Drake's VTK renderer.
        """
        path = self._record(ChemLab2(meshcat_port_number=None), "chem_lab2", 1.0)
        self.assertEqual(_count_frames(path), 11)


class _NestedSceneGraphProduction(BaseProduction):
    """
    A minimal production whose SceneGraph is inside a nested diagram that does
    NOT export a "query_object" port.
    """

    def __init__(self):
        super().__init__()
        inner_builder = DiagramBuilder()
        self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(
            inner_builder, time_step=1e-3
        )
        self.plant.Finalize()
        self.inner = self.builder.AddSystem(inner_builder.Build())


class NestedSceneGraphErrorTest(unittest.TestCase):
    def test_missing_query_object_port1(self):
        """
        Description
        -----------
        If the SceneGraph is in a nested diagram without an exported
        "query_object" port, building with cameras should explain what to do.
        """
        production = _NestedSceneGraphProduction()
        production.add_cameras([CameraSpec("cam")])
        with self.assertRaises(NotImplementedError) as ctx:
            production.build_production(with_watcher=False)
        self.assertIn("query_object", str(ctx.exception))


if __name__ == "__main__":
    unittest.main()
