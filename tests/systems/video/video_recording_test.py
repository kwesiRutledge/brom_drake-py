from importlib import resources as impresources
from pathlib import Path
import shutil
import unittest

import numpy as np
from pydrake.geometry import Box
from pydrake.math import RigidTransform
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder

# Internal imports
from brom_drake import robots
from brom_drake.all import (
    CameraSpec,
    VideoRecorder,
    add_cameras_to_builder,
    drakeify_my_urdf,
)
from brom_drake.productions.debug import ShowMeThisModel

OUTPUT_DIR = Path("./brom/test_videos")


def _read_video(path: Path):
    import imageio.v2 as imageio

    reader = imageio.get_reader(str(path))
    frames = [frame for frame in reader]
    reader.close()
    return frames


class CameraSpecTest(unittest.TestCase):
    def test_looking_at1(self):
        """
        Description
        -----------
        A camera at +X looking at the origin (with +Z up) should have its
        optical axis (+Z_C) along -X_W, its image-right (+X_C) along -Y_W and
        its image-down (+Y_C) along -Z_W.
        """
        spec = CameraSpec.looking_at("cam", position=[2.0, 0.0, 0.0], target=[0, 0, 0])
        R = spec.X_PC.rotation().matrix()

        np.testing.assert_allclose(R[:, 2], [-1, 0, 0], atol=1e-12)
        np.testing.assert_allclose(R[:, 0], [0, 1, 0], atol=1e-12)
        np.testing.assert_allclose(R[:, 1], [0, 0, -1], atol=1e-12)
        np.testing.assert_allclose(spec.X_PC.translation(), [2, 0, 0])

    def test_looking_at2(self):
        """
        Description
        -----------
        The rotation produced by looking_at must be a valid rotation matrix,
        even for an arbitrary viewpoint.
        """
        spec = CameraSpec.looking_at(
            "cam", position=[1.3, -0.7, 2.1], target=[0.1, 0.2, 0.3]
        )
        R = spec.X_PC.rotation().matrix()
        np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-12)
        self.assertAlmostEqual(np.linalg.det(R), 1.0)

    def test_looking_at3(self):
        """
        Description
        -----------
        Looking straight along the 'up' direction is ill-defined and should raise.
        """
        with self.assertRaises(ValueError):
            CameraSpec.looking_at("cam", position=[0, 0, 3], target=[0, 0, 0])

    def test_validation1(self):
        """
        Description
        -----------
        Invalid specs should be rejected at construction time.
        """
        with self.assertRaises(ValueError):
            CameraSpec("odd", width=641)
        with self.assertRaises(ValueError):
            CameraSpec("fps", fps=0.0)
        with self.assertRaises(ValueError):
            CameraSpec("fov", fov_y=np.pi)
        with self.assertRaises(ValueError):
            CameraSpec("clip", clipping=(1.0, 0.5))
        with self.assertRaises(ValueError):
            CameraSpec("")


class AddCamerasToBuilderTest(unittest.TestCase):
    def setUp(self):
        shutil.rmtree(OUTPUT_DIR / "builder", ignore_errors=True)

    def _make_box_scene(self):
        builder = DiagramBuilder()
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-3)
        plant.RegisterVisualGeometry(
            plant.world_body(),
            RigidTransform(),
            Box(0.3, 0.3, 0.3),
            "box",
            [1, 0, 0, 1],
        )
        plant.Finalize()
        return builder, plant, scene_graph

    def test_records_video1(self):
        """
        Description
        -----------
        A single camera looking at a red box should write an MP4 with
        (duration * fps + 1) frames that contain red pixels.
        """
        builder, plant, scene_graph = self._make_box_scene()
        specs = [
            CameraSpec.looking_at(
                "box_cam",
                position=[1.0, 0, 0.3],
                target=[0, 0, 0],
                width=64,
                height=48,
                fps=10,
            )
        ]
        recorders = add_cameras_to_builder(
            builder, scene_graph, specs, plant=plant, output_dir=OUTPUT_DIR / "builder"
        )
        diagram = builder.Build()

        simulator = Simulator(diagram)
        simulator.AdvanceTo(1.0)
        path = recorders[0].save()

        self.assertEqual(recorders[0].num_frames, 11)
        frames = _read_video(path)
        self.assertEqual(len(frames), 11)
        self.assertEqual(frames[0].shape, (48, 64, 3))

        # The center pixel should be (mostly) red
        r, g, b = frames[0][24, 32].astype(int)
        self.assertGreater(r, 100)
        self.assertGreater(r, g + 50)
        self.assertGreater(r, b + 50)

    def test_save_twice1(self):
        """
        Description
        -----------
        save() should be idempotent, and a recorder that never captured a frame
        should not write a file.
        """
        recorder = VideoRecorder(OUTPUT_DIR / "builder" / "never.mp4", fps=10)
        self.assertIsNone(recorder.save())
        self.assertIsNone(recorder.save())
        self.assertFalse((OUTPUT_DIR / "builder" / "never.mp4").exists())

    def test_duplicate_names1(self):
        """
        Description
        -----------
        Two cameras with the same name would write to the same file, so this
        should raise.
        """
        builder, plant, scene_graph = self._make_box_scene()
        with self.assertRaises(ValueError):
            add_cameras_to_builder(
                builder, scene_graph, [CameraSpec("a"), CameraSpec("a")], plant=plant
            )


class ProductionVideoTest(unittest.TestCase):
    def setUp(self):
        shutil.rmtree(OUTPUT_DIR / "production", ignore_errors=True)

    def _make_production(self) -> ShowMeThisModel:
        urdf_file_path = drakeify_my_urdf(
            str(impresources.files(robots) / "models/ur/ur10e.urdf"),
            overwrite_old_logs=True,
            log_file_name="video-recording-test-drakeify.log",
        )
        return ShowMeThisModel(
            str(urdf_file_path),
            with_these_joint_positions=[0.0, -np.pi / 4.0, np.pi / 4.0, 0.0, 0.0, 0.0],
            meshcat_port_number=None,  # Turn off for CI
        )

    def test_multiple_cameras1(self):
        """
        Description
        -----------
        A production with two cameras (at different frame rates) should
        write one MP4 per camera.
        """
        production = self._make_production()
        production.add_cameras(
            [
                CameraSpec.looking_at(
                    "front",
                    position=[2.5, 0, 1.2],
                    target=[0, 0, 0.5],
                    width=96,
                    height=64,
                    fps=20,
                ),
                CameraSpec.looking_at(
                    "top",
                    position=[0, 0, 3.5],
                    target=[0, 0, 0],
                    up=[1, 0, 0],
                    width=64,
                    height=64,
                    fps=5,
                ),
            ],
            output_dir=OUTPUT_DIR / "production",
        )
        diagram, diagram_context = production.add_cast_and_build()

        simulator = Simulator(diagram, diagram_context)
        simulator.AdvanceTo(1.0)
        video_paths = production.save_videos()

        self.assertEqual(set(video_paths.keys()), {"front", "top"})
        self.assertEqual(len(_read_video(video_paths["front"])), 21)
        self.assertEqual(len(_read_video(video_paths["top"])), 6)

    def test_grid_video1(self):
        """
        Description
        -----------
        save_videos(grid=True) also writes one video showing every camera,
        at the highest camera fps, aligned by simulation time.
        """
        production = self._make_production()
        production.add_cameras(
            [
                CameraSpec.looking_at(
                    "front",
                    position=[2.5, 0, 1.2],
                    target=[0, 0, 0.5],
                    width=96,
                    height=64,
                    fps=20,
                ),
                CameraSpec.looking_at(
                    "top",
                    position=[0, 0, 3.5],
                    target=[0, 0, 0],
                    up=[1, 0, 0],
                    width=64,
                    height=64,
                    fps=5,
                ),
            ],
            output_dir=OUTPUT_DIR / "production",
        )
        diagram, diagram_context = production.add_cast_and_build()

        simulator = Simulator(diagram, diagram_context)
        simulator.AdvanceTo(1.0)
        video_paths = production.save_videos(grid=True)

        self.assertEqual(set(video_paths.keys()), {"front", "top", "all_cameras"})
        grid_frames = _read_video(video_paths["all_cameras"])
        self.assertEqual(len(grid_frames), 21)  # 20 fps over 1.0 s
        self.assertEqual(grid_frames[0].shape, (64, 192, 3))  # 2 tiles of 96x64

    def test_grid_name_collision1(self):
        """
        Description
        -----------
        The grid video cannot share its name with a camera.
        """
        production = self._make_production()
        production.add_cameras(
            [CameraSpec("all_cameras", width=64, height=48)],
            output_dir=OUTPUT_DIR / "production",
        )
        production.add_cast_and_build()
        with self.assertRaises(ValueError):
            production.save_videos(grid=True)

    def test_add_cameras_after_build1(self):
        """
        Description
        -----------
        Cameras cannot be added once the diagram is built.
        """
        production = self._make_production()
        production.add_cast_and_build()
        with self.assertRaises(RuntimeError):
            production.add_cameras([CameraSpec("late")])

    def test_no_cameras1(self):
        """
        Description
        -----------
        Cameras are strictly opt-in: a production that never calls
        add_cameras() gets no renderer, no camera systems and no recorders,
        so its simulation does no rendering work at all.
        """
        from pydrake.systems.sensors import RgbdSensor

        production = self._make_production()
        diagram, _ = production.add_cast_and_build()

        self.assertEqual(production.scene_graph.RendererCount(), 0)
        systems = diagram.GetSystems()
        self.assertFalse(any(isinstance(s, RgbdSensor) for s in systems))
        self.assertFalse(any(isinstance(s, VideoRecorder) for s in systems))
        self.assertEqual(production.video_recorders, {})
        self.assertEqual(production.save_videos(), {})


if __name__ == "__main__":
    unittest.main()
