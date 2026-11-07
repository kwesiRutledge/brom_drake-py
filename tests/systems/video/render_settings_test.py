from pathlib import Path
import shutil
import unittest

import numpy as np
from pydrake.geometry import Box, LightParameter
from pydrake.math import RigidTransform
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder

# Internal imports
from brom_drake.all import CameraSpec, RenderSettings, add_cameras_to_builder

OUTPUT_DIR = Path("./brom/test_videos/render_settings")


def _box_scene():
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-3)
    plant.RegisterVisualGeometry(
        plant.world_body(), RigidTransform(), Box(0.2, 0.2, 0.2), "box", [1, 0, 0, 1]
    )
    plant.Finalize()
    return builder, plant, scene_graph


def _camera(name: str) -> CameraSpec:
    return CameraSpec.looking_at(
        name, position=[1.0, 0, 0.3], target=[0, 0, 0], width=64, height=48, fps=10
    )


def _first_frame(path: Path) -> np.ndarray:
    import imageio.v2 as imageio

    reader = imageio.get_reader(str(path))
    frame = np.asarray(reader.get_data(0))
    reader.close()
    return frame


class RenderSettingsTest(unittest.TestCase):
    def setUp(self):
        shutil.rmtree(OUTPUT_DIR, ignore_errors=True)

    def _record(self, render_settings, name: str) -> np.ndarray:
        builder, plant, scene_graph = _box_scene()
        recorders = add_cameras_to_builder(
            builder,
            scene_graph,
            [_camera(name)],
            plant=plant,
            output_dir=OUTPUT_DIR,
            render_settings=render_settings,
        )
        Simulator(builder.Build()).AdvanceTo(0.05)
        return _first_frame(recorders[0].save())

    def test_background_color1(self):
        """
        Description
        -----------
        The background color is applied to every pixel that shows no geometry.
        """
        frame = self._record(RenderSettings(background_rgb=(0.1, 0.6, 0.2)), "bg")
        corner = frame[2, 2].astype(int)
        np.testing.assert_allclose(corner, [26, 153, 51], atol=12)

    def test_studio_differs_from_default1(self):
        """
        Description
        -----------
        The studio preset changes how the scene looks (background and lighting).
        """
        default = self._record(None, "default")
        studio = self._record(RenderSettings.studio(), "studio")
        self.assertGreater(np.abs(default.astype(int) - studio.astype(int)).mean(), 5)

    def test_settings_on_existing_renderer1(self):
        """
        Description
        -----------
        Render settings cannot change a renderer that already exists, so passing
        them a second time raises instead of being silently ignored.
        """
        builder, plant, scene_graph = _box_scene()
        add_cameras_to_builder(
            builder, scene_graph, [_camera("a")], plant=plant, output_dir=OUTPUT_DIR
        )
        with self.assertRaises(ValueError):
            add_cameras_to_builder(
                builder,
                scene_graph,
                [_camera("b")],
                plant=plant,
                output_dir=OUTPUT_DIR,
                render_settings=RenderSettings.studio(),
            )

    def test_validation1(self):
        with self.assertRaises(ValueError):
            RenderSettings(background_rgb=(255, 255, 255))
        with self.assertRaises(ValueError):
            RenderSettings(cast_shadows=True)  # Shadows need explicit lights

    def test_shadow_map_size1(self):
        """
        Description
        -----------
        With shadows on, the shadow map resolution is raised from Drake's
        default (256) to avoid blotchy shadows; it can also be set explicitly.
        """
        lights = [LightParameter(type="directional", frame="world")]
        self.assertEqual(
            RenderSettings(lights=lights, cast_shadows=True)
            .to_vtk_params()
            .shadow_map_size,
            2048,
        )
        self.assertEqual(
            RenderSettings(lights=lights, cast_shadows=True, shadow_map_size=512)
            .to_vtk_params()
            .shadow_map_size,
            512,
        )
        with self.assertRaises(ValueError):
            RenderSettings(shadow_map_size=0)

    def test_to_vtk_params1(self):
        lights = [LightParameter(type="directional", frame="world")]
        params = RenderSettings(
            background_rgb=(0.2, 0.3, 0.4), lights=lights, exposure=0.5
        ).to_vtk_params(backend="EGL")
        np.testing.assert_allclose(params.default_clear_color, [0.2, 0.3, 0.4])
        self.assertEqual(len(params.lights), 1)
        self.assertEqual(params.exposure, 0.5)
        self.assertEqual(params.backend, "EGL")


if __name__ == "__main__":
    unittest.main()
