# suggested_use1

This example shows how you can test a *grasp selection algorithm* with the
`GraspSelection1` production.

A grasp selection algorithm is any function that takes a description of the object
being grasped and returns either:
- a list of poses of the gripper's base frame with respect to the object
  (the last pose is the grasp pose), or
- a single grasp pose.

The type hint on the function's first parameter chooses what it receives:
- `ObjectGeometry` gives it the object's collision shapes, with their poses in the object frame.
- `ObjectDescription` gives it that geometry and the path to the object's model.

The production calls the algorithm once when you call `easy_cast_and_build()`,
has the gripper follow the resulting trajectory, closes the gripper, and then drops
the floor out from under the object. If the object stays in the gripper, the grasp worked!

With the poses in this example, the flask stays in the gripper when the floor drops.
Try swapping in your own grasp selection algorithm and see whether your grasps hold too!

The generated trajectory is also recorded in the production's build metadata
(look for `build_metadata_*.json` in the `brom` directory).

Run it with:

```bash
python select_and_attempt.py
```

and open Meshcat at http://localhost:7001 to watch the grasp attempt.
