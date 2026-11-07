import logging
from pathlib import Path
from typing import Optional, Union

import numpy as np
from pydrake.common.value import AbstractValue
from pydrake.systems.framework import Context, EventStatus, LeafSystem
from pydrake.systems.sensors import ImageRgba8U


def _import_imageio():
    """
    Imports imageio lazily so that brom_drake can be used without the optional
    video dependencies installed.
    """
    try:
        import imageio.v2 as imageio  # noqa: F401
        import imageio_ffmpeg  # noqa: F401
    except ImportError as e:
        raise ImportError(
            "Recording videos requires the optional 'video' dependencies. "
            "Install them with: pip install brom_drake[video]"
        ) from e
    return imageio


class VideoRecorder(LeafSystem):
    """
    **Description**

    A Drake LeafSystem that captures color images from its input port at a fixed
    rate (in simulation time) and streams them into an MP4 file.

    Frames are written to disk as they are captured (not held in memory), so long
    simulations do not exhaust RAM. Call :py:meth:`save` once the simulation has
    finished to close the file.

    Because the recorder only reads its input when a frame is due, an upstream
    ``RgbdSensor`` only renders at the recorder's frame rate.

    **Parameters**

    output_path: Union[str, Path]
        Where to write the MP4 file. Parent directories are created as needed.

    fps: float
        The number of frames to capture per second of simulation time.
        This is also the playback frame rate of the video.

    name: str, optional
        The name of this system. Default is "video_recorder".

    **Ports**

    color_image: AbstractValue[ImageRgba8U]
        The image to record (e.g., an RgbdSensor's ``color_image`` output).
    """

    def __init__(
        self,
        output_path: Union[str, Path],
        fps: float,
        name: str = "video_recorder",
    ):
        LeafSystem.__init__(self)
        if fps <= 0:
            raise ValueError(f"VideoRecorder fps must be positive; received {fps}.")

        self.set_name(name)
        self.output_path = Path(output_path)
        self.fps = float(fps)

        self._writer = None
        self._num_frames = 0
        self._last_capture_time: Optional[float] = None
        self._first_capture_time: Optional[float] = None
        self._closed = False

        # Fail fast if the optional dependencies are missing
        _import_imageio()

        self.DeclareAbstractInputPort(
            "color_image",
            AbstractValue.Make(ImageRgba8U()),
        )

        self.DeclarePeriodicPublishEvent(
            period_sec=1.0 / self.fps,
            offset_sec=0.0,
            publish=self._capture_frame,
        )

    @property
    def num_frames(self) -> int:
        """The number of frames written so far."""
        return self._num_frames

    @property
    def first_frame_time(self) -> Optional[float]:
        """The simulation time of the first captured frame (None if no frames)."""
        return self._first_capture_time

    @property
    def is_saved(self) -> bool:
        """Whether the video file has been finalized (closed)."""
        return self._closed

    def _capture_frame(self, context: Context) -> EventStatus:
        """
        **Description**

        Periodic publish callback: reads the input image and appends it to the video.
        """
        t = context.get_time()

        if self._closed:
            return EventStatus.DidNothing()

        # Guard against capturing the same instant twice (e.g., repeated publishes)
        if self._last_capture_time is not None and t <= self._last_capture_time:
            return EventStatus.DidNothing()

        image = self.get_input_port(0).Eval(context)
        frame = np.ascontiguousarray(image.data[:, :, :3])  # Drop the alpha channel

        if self._writer is None:
            self._open_writer()

        self._writer.append_data(frame)
        self._num_frames += 1
        if self._first_capture_time is None:
            self._first_capture_time = t
        self._last_capture_time = t
        return EventStatus.Succeeded()

    def _open_writer(self):
        imageio = _import_imageio()
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self._writer = imageio.get_writer(
            str(self.output_path),
            fps=self.fps,
            codec="libx264",
            quality=8,  # 0-10 scale used by imageio-ffmpeg
            pixelformat="yuv420p",  # Plays in browsers, Slack, QuickTime, etc.
            macro_block_size=1,  # CameraSpec already guarantees even dimensions
        )

    def save(self) -> Optional[Path]:
        """
        **Description**

        Closes the video file. Safe to call more than once.

        **Returns**

        output_path: Optional[Path]
            The path of the written video, or None if no frames were captured.
        """
        if self._closed:
            return self.output_path if self._num_frames > 0 else None

        self._closed = True
        if self._writer is None:
            logging.getLogger(__name__).warning(
                f"VideoRecorder '{self.get_name()}' captured no frames; "
                f"no video was written to {self.output_path}."
            )
            return None

        self._writer.close()
        self._writer = None
        return self.output_path

    def __del__(self):
        # Best-effort cleanup in case the user never called save()
        try:
            if not self._closed and self._writer is not None:
                self._writer.close()
                self._closed = True
        except Exception:
            pass
