import math
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Sequence, Tuple, Union

import numpy as np

from .video_recorder import _import_imageio


@dataclass
class GridSource:
    """
    **Description**

    One camera's recorded video, described so that it can be placed in a grid.

    **Parameters**

    name: str
        The label drawn on the camera's tile.

    path: Path
        The camera's MP4 file.

    fps: float
        The rate (in simulation time) at which the camera's frames were captured.

    first_frame_time: float
        The simulation time of the camera's first frame. Default is 0.0.
    """

    name: str
    path: Path
    fps: float
    first_frame_time: float = 0.0


def _grid_shape(n_tiles: int, columns: Optional[int]) -> Tuple[int, int]:
    if columns is None:
        columns = math.ceil(math.sqrt(n_tiles))
    columns = max(1, min(columns, n_tiles))
    rows = math.ceil(n_tiles / columns)
    return rows, columns


def _fit_into_tile(
    frame: np.ndarray,
    tile_size: Tuple[int, int],
    background: Tuple[int, int, int],
) -> np.ndarray:
    """
    Scales ``frame`` (preserving its aspect ratio) to fit inside a tile of
    ``tile_size`` = (width, height), centered on a solid background.
    """
    from PIL import Image

    tile_w, tile_h = tile_size
    h, w = frame.shape[:2]
    scale = min(tile_w / w, tile_h / h)
    new_w, new_h = max(1, round(w * scale)), max(1, round(h * scale))

    tile = np.empty((tile_h, tile_w, 3), dtype=np.uint8)
    tile[:, :] = background
    if (new_w, new_h) != (w, h):
        frame = np.asarray(
            Image.fromarray(frame).resize((new_w, new_h), Image.Resampling.BILINEAR)
        )
    y0, x0 = (tile_h - new_h) // 2, (tile_w - new_w) // 2
    tile[y0 : y0 + new_h, x0 : x0 + new_w] = frame
    return tile


def _label_tile(tile: np.ndarray, label: str) -> np.ndarray:
    """Draws ``label`` in the top-left corner of ``tile`` on a dark backing."""
    from PIL import Image, ImageDraw, ImageFont

    image = Image.fromarray(tile)
    draw = ImageDraw.Draw(image)
    font_size = max(10, tile.shape[0] // 24)
    try:
        font = ImageFont.load_default(size=font_size)
    except TypeError:  # Pillow < 10.1 has no size argument
        font = ImageFont.load_default()

    pad = max(3, font_size // 3)
    left, top, right, bottom = draw.textbbox((0, 0), label, font=font)
    draw.rectangle(
        (0, 0, (right - left) + 2 * pad, (bottom - top) + 2 * pad),
        fill=(20, 24, 30),
    )
    draw.text((pad - left, pad - top), label, fill=(240, 242, 245), font=font)
    return np.asarray(image)


class _FrameCursor:
    """
    Walks through one camera's video, returning the latest frame captured at or
    before a requested simulation time (holding the last frame once the video
    ends, and the first frame before it starts).
    """

    def __init__(self, source: GridSource):
        imageio = _import_imageio()
        self.source = source
        self._reader = imageio.get_reader(str(source.path))
        self._iter = iter(self._reader)
        self._index = -1
        self._frame: Optional[np.ndarray] = None
        self._exhausted = False
        self._advance()  # Load the first frame

    def _advance(self) -> bool:
        try:
            self._frame = np.asarray(next(self._iter))[:, :, :3]
            self._index += 1
            return True
        except StopIteration:
            self._exhausted = True
            return False

    @property
    def num_frames_read(self) -> int:
        return self._index + 1

    def frame_at(self, t: float) -> np.ndarray:
        # The frame to show is the last one whose capture time is <= t
        target_index = math.floor(
            (t - self.source.first_frame_time) * self.source.fps + 1e-6
        )
        while not self._exhausted and self._index < target_index:
            self._advance()
        return self._frame

    def end_time(self, num_frames: int) -> float:
        return self.source.first_frame_time + (num_frames - 1) / self.source.fps

    def close(self):
        self._reader.close()


def combine_videos_into_grid(
    sources: Sequence[GridSource],
    output_path: Union[str, Path],
    columns: Optional[int] = None,
    tile_size: Optional[Tuple[int, int]] = None,
    fps: Optional[float] = None,
    label_tiles: bool = True,
    background_rgb: Tuple[int, int, int] = (16, 18, 22),
) -> Path:
    """
    **Description**

    Combines several camera videos into one MP4 that shows all of them side by
    side in a grid. The videos are aligned by *simulation time*: each frame of
    the grid shows, for every camera, the latest frame captured at or before
    that time. Cameras may have different frame rates and resolutions.

    **Parameters**

    sources: Sequence[GridSource]
        The videos to combine, in the order they should fill the grid
        (left to right, top to bottom).

    output_path: Union[str, Path]
        Where to write the combined MP4.

    columns: int, optional
        The number of columns in the grid. Default is ceil(sqrt(n)).

    tile_size: Tuple[int, int], optional
        The (width, height) of each tile. Each video is scaled to fit inside it,
        preserving its aspect ratio. Default is the first source's resolution.

    fps: float, optional
        The frame rate of the combined video. Default is the highest source fps.

    label_tiles: bool, optional
        Whether to draw each camera's name on its tile. Default is True.

    background_rgb: Tuple[int, int, int], optional
        The color of empty grid cells and letterboxing.

    **Returns**

    output_path: Path
        The path of the combined video.
    """
    if len(sources) == 0:
        raise ValueError("combine_videos_into_grid needs at least one video.")

    imageio = _import_imageio()
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    # Count each source's frames (needed to know when the grid video ends)
    frame_counts: List[int] = []
    for source in sources:
        reader = imageio.get_reader(str(source.path))
        frame_counts.append(reader.count_frames())
        reader.close()

    cursors = [_FrameCursor(source) for source in sources]
    try:
        # Layout
        if tile_size is None:
            first = cursors[0].frame_at(sources[0].first_frame_time)
            tile_size = (first.shape[1], first.shape[0])
        tile_w, tile_h = int(tile_size[0]), int(tile_size[1])
        rows, cols = _grid_shape(len(sources), columns)

        # H.264 (yuv420p) needs an even frame size; pad the canvas if needed
        canvas_w, canvas_h = cols * tile_w, rows * tile_h
        canvas_w += canvas_w % 2
        canvas_h += canvas_h % 2

        # Timing
        if fps is None:
            fps = max(source.fps for source in sources)
        t_start = min(source.first_frame_time for source in sources)
        t_end = max(
            cursor.end_time(n_frames) for cursor, n_frames in zip(cursors, frame_counts)
        )
        n_output_frames = math.floor((t_end - t_start) * fps + 1e-6) + 1

        writer = imageio.get_writer(
            str(output_path),
            fps=fps,
            codec="libx264",
            quality=8,
            pixelformat="yuv420p",
            macro_block_size=1,
        )
        try:
            canvas = np.empty((canvas_h, canvas_w, 3), dtype=np.uint8)
            for k in range(n_output_frames):
                t = t_start + k / fps
                canvas[:, :] = background_rgb
                for i, (cursor, source) in enumerate(zip(cursors, sources)):
                    tile = _fit_into_tile(
                        cursor.frame_at(t), (tile_w, tile_h), background_rgb
                    )
                    if label_tiles:
                        tile = _label_tile(tile, source.name)
                    r, c = divmod(i, cols)
                    canvas[
                        r * tile_h : (r + 1) * tile_h, c * tile_w : (c + 1) * tile_w
                    ] = tile
                writer.append_data(canvas)
        finally:
            writer.close()
    finally:
        for cursor in cursors:
            cursor.close()

    return output_path
