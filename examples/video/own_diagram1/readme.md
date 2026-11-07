# Recording Your Own Drake Diagram

You don't need a Brom production to record videos. `add_cameras_to_builder` adds cameras and MP4 recorders to any `DiagramBuilder` that contains a `SceneGraph`:

```python
recorders = add_cameras_to_builder(builder, scene_graph, [CameraSpec(...), ...], plant=plant)
diagram = builder.Build()
Simulator(diagram).AdvanceTo(2.0)
for recorder in recorders:
    recorder.save()
```

This example drops a red box onto the ground. It films the box with one camera fixed in the world and one camera attached to the box (`parent_body_name="box"`), which tumbles along with it.

Only geometry with Drake's *perception* role is visible to cameras. `MultibodyPlant.RegisterVisualGeometry` and URDF/SDF visuals have it by default.

Output: `brom/videos/own_diagram1/{world_view,box_view}.mp4`

## Requirements

See the [single camera example](../../productions/recording_videos/single_camera1/readme.md) for the install steps (`brom-drake[video]` and the EGL libraries on Linux).

## Resources

For more details, see the `Features > Recording Videos` section of the [docs](https://kwesirutledge.github.io/brom_drake-py/).
