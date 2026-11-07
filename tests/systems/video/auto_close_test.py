from pathlib import Path
import shutil
import subprocess
import sys
import textwrap
import unittest

OUTPUT_DIR = Path("./brom/test_videos/auto_close").resolve()

SCRIPT = textwrap.dedent("""
    from pydrake.all import (
        AddMultibodyPlantSceneGraph, Box, DiagramBuilder, RigidTransform, Simulator,
    )
    from brom_drake.all import CameraSpec, add_cameras_to_builder

    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 1e-3)
    plant.RegisterVisualGeometry(
        plant.world_body(), RigidTransform(), Box(0.3, 0.3, 0.3), "box", [1, 0, 0, 1]
    )
    plant.Finalize()
    add_cameras_to_builder(
        builder,
        scene_graph,
        [CameraSpec.looking_at("cam", [1, 0, 0.3], [0, 0, 0], width=64, height=48, fps=10)],
        output_dir=r"{output_dir}",
    )
    Simulator(builder.Build()).AdvanceTo(1.0)
    # Exit WITHOUT calling recorder.save()
    """)


class AutoCloseAtExitTest(unittest.TestCase):
    def setUp(self):
        shutil.rmtree(OUTPUT_DIR, ignore_errors=True)

    def test_unsaved_video_is_closed_at_exit1(self):
        """
        Description
        -----------
        A script that never saves its videos still leaves a playable MP4
        (closed at interpreter exit), and warns about the missing save.
        """
        result = subprocess.run(
            [sys.executable, "-c", SCRIPT.format(output_dir=OUTPUT_DIR)],
            capture_output=True,
            text=True,
            timeout=300,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("was never saved", result.stderr)

        import imageio.v2 as imageio

        reader = imageio.get_reader(str(OUTPUT_DIR / "cam.mp4"))
        self.assertEqual(reader.count_frames(), 11)
        reader.close()


if __name__ == "__main__":
    unittest.main()
