# Default Cameras, Studio Lighting and a Grid Video

Records `ShelfPlanning1` while it executes a motion plan, using:

- **The production's default cameras.** `add_cameras()` with no camera list uses `production.default_cameras()`: here an overview, a top-down view and a `wrist` camera attached to the robot's `tool0` frame.
- **`RenderSettings.studio()`**: a grey background, world-fixed lights and shadows.
- **`save_videos(grid=True)`**, which also writes `all_cameras.mp4` with every camera side by side, aligned by simulation time.

Output: `brom/videos/default_cameras_and_grid1/{overview,top,wrist,all_cameras}.mp4`

## Requirements

See the [single camera example](../single_camera1/readme.md) for the install steps (`brom-drake[video]` and the EGL libraries on Linux).

## Resources

For more details, see the `Features > Recording Videos` section of the [docs](https://kwesirutledge.github.io/brom_drake-py/).
