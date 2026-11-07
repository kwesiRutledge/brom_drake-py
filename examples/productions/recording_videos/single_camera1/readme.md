# Recording a Production With One Camera

The smallest recording example: a UR10e shown by `ShowMeThisModel`, filmed by one fixed camera.

```python
production.add_cameras([CameraSpec.looking_at("front", position=[1.6, 0.9, 1.1], target=[0.0, 0.0, 0.55])])
diagram, diagram_context = production.add_cast_and_build()
Simulator(diagram, diagram_context).AdvanceTo(3.0)
production.save_videos()  # -> brom/videos/single_camera1/front.mp4
```

Recording is opt-in. A production does no rendering unless you call `add_cameras()` before building it.

## Requirements

Recording needs the optional video dependencies:

```shell
pip install "brom-drake[video]"
```

On Linux without a display (e.g., CI or a container), Drake renders with EGL. Install the system libraries with:

```shell
sudo apt-get install libegl1 libegl-mesa0 libgl1-mesa-dri libopengl0
```

## Resources

For more details, see the `Features > Recording Videos` section of the [docs](https://kwesirutledge.github.io/brom_drake-py/).
