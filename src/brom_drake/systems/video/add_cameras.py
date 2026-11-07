import math
from pathlib import Path
from typing import List, Optional, Sequence, Union

from pydrake.geometry import (
    MakeRenderEngineVtk,
    RenderEngineVtkParams,
    SceneGraph,
)
from pydrake.multibody.plant import MultibodyPlant
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.sensors import CameraConfig, RgbdSensor

# Internal Imports
from brom_drake.directories import DEFAULT_VIDEO_DIR
from .camera_spec import CameraSpec
from .video_recorder import VideoRecorder

DEFAULT_RENDERER_NAME = "brom_video_renderer"


def _camera_config_from_spec(spec: CameraSpec, renderer_name: str) -> CameraConfig:
    config = CameraConfig()
    config.name = spec.name
    config.width = spec.width
    config.height = spec.height
    config.focal = CameraConfig.FovDegrees(y=math.degrees(spec.fov_y))
    config.clipping_near, config.clipping_far = spec.clipping
    config.z_near, config.z_far = spec.clipping
    config.renderer_name = renderer_name
    config.rgb, config.depth, config.label = True, False, False
    return config


def add_cameras_to_builder(
    builder: DiagramBuilder,
    scene_graph: SceneGraph,
    camera_specs: Sequence[CameraSpec],
    plant: Optional[MultibodyPlant] = None,
    output_dir: Union[str, Path] = DEFAULT_VIDEO_DIR,
    renderer_name: str = DEFAULT_RENDERER_NAME,
    renderer_backend: str = "",
) -> List[VideoRecorder]:
    """
    **Description**

    Adds one camera (an ``RgbdSensor``) and one :py:class:`VideoRecorder` to
    ``builder`` for every CameraSpec. Each recorder writes ``<output_dir>/<name>.mp4``.

    A VTK render engine is added to ``scene_graph`` (if it does not already have
    one named ``renderer_name``). Only geometries with the *perception* role are
    visible to the cameras; MultibodyPlant gives this role to visual geometries
    from URDF/SDF files by default.

    **Parameters**

    builder: DiagramBuilder
        The builder to add the systems to. It must contain ``scene_graph``.

    scene_graph: SceneGraph
        The SceneGraph that holds the geometry to render.

    camera_specs: Sequence[CameraSpec]
        The cameras to add. Names must be unique.

    plant: MultibodyPlant, optional
        Required only if a spec uses ``parent_body_name``.

    output_dir: Union[str, Path], optional
        The directory to write videos into. Default is ``./brom/videos``.

    renderer_name: str, optional
        The name of the render engine to use/create.

    renderer_backend: str, optional
        The VTK backend ("" for Drake's default, "EGL" for headless Linux, "GLX", or
        "Cocoa" on macOS). Default is "".

    **Returns**

    recorders: List[VideoRecorder]
        The recorders, in the same order as ``camera_specs``.
    """
    # Check the inputs
    names = [spec.name for spec in camera_specs]
    duplicates = sorted({n for n in names if names.count(n) > 1})
    if duplicates:
        raise ValueError(f"Camera names must be unique; duplicated names: {duplicates}")

    if scene_graph not in builder.GetSystems():
        raise ValueError(
            "The given SceneGraph is not part of this builder. If the SceneGraph is "
            "inside a nested diagram (e.g., a station), cameras cannot be attached "
            "from the outer builder yet."
        )

    # Add a render engine, if needed
    if not scene_graph.HasRenderer(renderer_name):
        scene_graph.AddRenderer(
            renderer_name,
            MakeRenderEngineVtk(RenderEngineVtkParams(backend=renderer_backend)),
        )

    output_dir = Path(output_dir)
    recorders = []
    for spec in camera_specs:
        # Find the frame the camera is attached to
        if spec.parent_body_name is None:
            parent_id = scene_graph.world_frame_id()
        else:
            if plant is None:
                raise ValueError(
                    f"CameraSpec '{spec.name}' is attached to body "
                    f"'{spec.parent_body_name}', but no plant was given."
                )
            body = plant.GetBodyByName(spec.parent_body_name)
            parent_id = plant.GetBodyFrameIdOrThrow(body.index())

        # Create the camera
        color_camera, depth_camera = _camera_config_from_spec(
            spec, renderer_name
        ).MakeCameras()
        sensor = builder.AddSystem(
            RgbdSensor(
                parent_id=parent_id,
                X_PB=spec.X_PC,
                color_camera=color_camera,
                depth_camera=depth_camera,
            )
        )
        sensor.set_name(f"camera_{spec.name}")
        builder.Connect(
            scene_graph.get_query_output_port(),
            sensor.query_object_input_port(),
        )

        # Create the recorder
        recorder = builder.AddSystem(
            VideoRecorder(
                output_path=output_dir / f"{spec.name}.mp4",
                fps=spec.fps,
                name=f"video_recorder_{spec.name}",
            )
        )
        builder.Connect(
            sensor.color_image_output_port(),
            recorder.get_input_port(0),
        )
        recorders.append(recorder)

    return recorders
