# Custom Cameras and Lighting

Records `ChemLab2` (the dynamic UR10e station with a Robotiq gripper) with:

- **Cameras with different resolutions and frame rates** (854×480 at 30 fps, 640×360 at 10 fps, 480×480 at 15 fps).
- **A camera placed with an explicit pose** (`CameraSpec(X_PC=RigidTransform(...))`). Drake cameras look along their +Z axis, with +X to the right of the image and +Y down.
- **Custom `RenderSettings`**: a dark background, a warm key light, a cool rim light and shadows.
- **A grid layout**: `save_videos(grid=True, grid_columns=2, grid_tile_size=(640, 360))`.

The grid video stays in sync even though the cameras use different frame rates, because each grid frame shows every camera's latest image at that simulation time.

Output: `brom/videos/custom_cameras_and_lighting1/{overview,side,top,all_cameras}.mp4`

## Requirements

See the [single camera example](../single_camera1/readme.md) for the install steps (`brom-drake[video]` and the EGL libraries on Linux).

## Resources

For more details, see the `Features > Recording Videos` section of the [docs](https://kwesirutledge.github.io/brom_drake-py/).
