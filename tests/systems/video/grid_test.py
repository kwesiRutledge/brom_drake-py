from pathlib import Path
import shutil
import unittest

import numpy as np

# Internal imports
from brom_drake.all import GridSource, combine_videos_into_grid

OUTPUT_DIR = Path("./brom/test_videos/grid")

RED, GREEN, BLUE = (220, 30, 30), (30, 200, 30), (30, 30, 220)


def _write_video(path: Path, frames, fps: float):
    import imageio.v2 as imageio

    path.parent.mkdir(parents=True, exist_ok=True)
    writer = imageio.get_writer(
        str(path), fps=fps, codec="libx264", pixelformat="yuv420p", macro_block_size=1
    )
    for frame in frames:
        writer.append_data(frame)
    writer.close()


def _solid(width: int, height: int, rgb) -> np.ndarray:
    frame = np.empty((height, width, 3), dtype=np.uint8)
    frame[:, :] = rgb
    return frame


def _read_video(path: Path):
    import imageio.v2 as imageio

    reader = imageio.get_reader(str(path))
    frames = [np.asarray(frame) for frame in reader]
    reader.close()
    return frames


def _assert_color(test: unittest.TestCase, pixel, rgb, tol: int = 40):
    test.assertTrue(
        np.all(np.abs(pixel.astype(int) - np.array(rgb)) < tol),
        f"pixel {pixel} is not close to {rgb}",
    )


class CombineVideosIntoGridTest(unittest.TestCase):
    def setUp(self):
        shutil.rmtree(OUTPUT_DIR, ignore_errors=True)

        # Camera "a": 10 fps for 1 s (11 frames), 64x48, red then blue from t = 0.5 s
        self.path_a = OUTPUT_DIR / "a.mp4"
        _write_video(
            self.path_a,
            [_solid(64, 48, RED if k < 5 else BLUE) for k in range(11)],
            fps=10,
        )

        # Camera "b": 5 fps for 0.6 s (4 frames), 32x24, green
        self.path_b = OUTPUT_DIR / "b.mp4"
        _write_video(self.path_b, [_solid(32, 24, GREEN) for _ in range(4)], fps=5)

    def test_layout_and_timing1(self):
        """
        Description
        -----------
        Two cameras with different frame rates and resolutions are combined
        side by side. The grid runs at the highest fps, lasts as long as the
        longest video, and shows each camera's latest frame at each time.
        """
        out = combine_videos_into_grid(
            [
                GridSource("a", self.path_a, fps=10),
                GridSource("b", self.path_b, fps=5),
            ],
            OUTPUT_DIR / "grid.mp4",
            label_tiles=False,
        )
        frames = _read_video(out)

        # 10 fps over 1.0 s of simulation time -> 11 frames, 2 tiles of 64x48
        self.assertEqual(len(frames), 11)
        self.assertEqual(frames[0].shape, (48, 128, 3))

        # Camera "a" switches from red to blue at t = 0.5 s
        _assert_color(self, frames[4][24, 32], RED)
        _assert_color(self, frames[5][24, 32], BLUE)

        # Camera "b" (scaled up to fill its tile) is green throughout, and holds
        # its last frame after its video ends at t = 0.6 s
        _assert_color(self, frames[0][24, 96], GREEN)
        _assert_color(self, frames[10][24, 96], GREEN)

    def test_first_frame_time1(self):
        """
        Description
        -----------
        A camera that started recording later is shown from its start time;
        before that, it shows its first frame.
        """
        out = combine_videos_into_grid(
            [
                GridSource("a", self.path_a, fps=10, first_frame_time=0.0),
                GridSource("late", self.path_a, fps=10, first_frame_time=0.5),
            ],
            OUTPUT_DIR / "grid_late.mp4",
            columns=1,
            label_tiles=False,
        )
        frames = _read_video(out)

        # Ends when the late camera ends: 0.5 + 1.0 s -> 16 frames, stacked
        self.assertEqual(len(frames), 16)
        self.assertEqual(frames[0].shape, (96, 64, 3))

        # At t = 0.7 s, camera "a" is blue but the late camera (0.2 s into its
        # video) is still red
        _assert_color(self, frames[7][24, 32], BLUE)
        _assert_color(self, frames[7][72, 32], RED)

    def test_odd_canvas_is_padded1(self):
        """
        Description
        -----------
        H.264 needs even frame sizes; an odd tile size is padded.
        """
        out = combine_videos_into_grid(
            [GridSource("a", self.path_a, fps=10)],
            OUTPUT_DIR / "grid_odd.mp4",
            tile_size=(33, 25),
        )
        self.assertEqual(_read_video(out)[0].shape, (26, 34, 3))

    def test_no_sources1(self):
        with self.assertRaises(ValueError):
            combine_videos_into_grid([], OUTPUT_DIR / "empty.mp4")


if __name__ == "__main__":
    unittest.main()
