from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

from pydrake.geometry import (
    LightParameter,
    MakeRenderEngineVtk,
    RenderEngineVtkParams,
    Rgba,
)


@dataclass
class RenderSettings:
    """
    **Description**

    How the cameras' renderer draws the scene: background color, lights,
    shadows and exposure. These settings apply to the whole renderer, which
    is shared by every camera of a production.

    Any field left as None keeps Drake's default (a light-blue background and
    a single light attached to each camera).

    **Parameters**

    background_rgb: Tuple[float, float, float], optional
        The background color, with components in [0, 1].

    lights: Sequence[LightParameter], optional
        The lights in the scene. Use ``frame="world"`` for lights fixed in the
        scene, or ``frame="camera"`` for lights that move with each camera.

    cast_shadows: bool, optional
        Whether lights cast shadows. Default is False.

    exposure: float, optional
        Enables tone mapping with this exposure (useful with bright lights).

    shadow_map_size: int, optional
        The resolution of the shadow maps, when shadows are on. Higher values
        give sharper shadows with fewer dark artifacts, at some rendering cost.
        Default is 2048 (Drake's own default, 256, leaves visible blotches on
        large surfaces).

    **Usage**

    ::

        production.add_cameras(cameras, render_settings=RenderSettings.studio())
    """

    background_rgb: Optional[Tuple[float, float, float]] = None
    lights: Optional[Sequence[LightParameter]] = None
    cast_shadows: bool = False
    exposure: Optional[float] = None
    shadow_map_size: int = 2048

    def __post_init__(self):
        if self.background_rgb is not None:
            if len(self.background_rgb) != 3 or not all(
                0.0 <= c <= 1.0 for c in self.background_rgb
            ):
                raise ValueError(
                    "RenderSettings.background_rgb must be three values in [0, 1]; "
                    f"received {self.background_rgb}."
                )
        if self.shadow_map_size <= 0:
            raise ValueError(
                "RenderSettings.shadow_map_size must be positive; "
                f"received {self.shadow_map_size}."
            )
        if self.cast_shadows and not self.lights:
            raise ValueError(
                "RenderSettings.cast_shadows needs at least one light in "
                "RenderSettings.lights (Drake's default camera light casts no shadows)."
            )

    @classmethod
    def studio(cls, background_rgb=(0.86, 0.87, 0.89)) -> "RenderSettings":
        """
        **Description**

        A neutral look for sharing: a light grey background, a key light from
        above, a soft fill light from the opposite side, and shadows.
        """
        return cls(
            background_rgb=background_rgb,
            lights=[
                LightParameter(
                    type="directional",
                    frame="world",
                    direction=[-0.35, 0.45, -1.0],
                    intensity=0.9,
                    color=Rgba(1.0, 0.98, 0.95),
                ),
                LightParameter(
                    type="directional",
                    frame="world",
                    direction=[0.6, -0.5, -0.35],
                    intensity=0.35,
                    color=Rgba(0.92, 0.95, 1.0),
                ),
                LightParameter(
                    type="directional",
                    frame="camera",
                    direction=[0.0, 0.0, 1.0],
                    intensity=0.2,
                ),
            ],
            cast_shadows=True,
        )

    def to_vtk_params(self, backend: str = "") -> RenderEngineVtkParams:
        """
        **Description**

        Converts these settings into Drake's VTK render engine parameters.
        """
        params = RenderEngineVtkParams(backend=backend)
        if self.background_rgb is not None:
            params.default_clear_color = list(self.background_rgb)
        if self.lights is not None:
            params.lights = list(self.lights)
        params.cast_shadows = self.cast_shadows
        if self.cast_shadows:
            params.shadow_map_size = self.shadow_map_size
        if self.exposure is not None:
            params.exposure = self.exposure
        return params

    def make_render_engine(self, backend: str = ""):
        """Creates a VTK render engine with these settings."""
        return MakeRenderEngineVtk(self.to_vtk_params(backend))
