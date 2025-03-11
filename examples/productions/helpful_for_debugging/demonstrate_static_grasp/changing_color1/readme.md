# Advanced Usage #1 of DemonstrateStaticGrasp with Color

In this example, we show you how to use the `DemonstrateStaticGrasp` production
to visualize a candidate grasp with a specific color.

The color is sometimes used to visualize a grasp in a way that is easy to distinguish from
the rest of a scene/robot.

In this example we provide:
- `path_to_object`: The fißle path to the object that we want to pick up.
- `path_to_gripper`: The file path to the gripper that we want to use to pick up the object.
- `X_ObjectTarget`: The rigid transform of the target frame on the gripper (by default, this is the base frame) with respect to the base frame of the object. This "object-centric" representation of the gripper's pose is common in many places.
- `gripper_color`: The color of the desired gripper model (written as a vector of 4 valuers; RGBA).