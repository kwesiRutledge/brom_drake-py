# Suggested Use #1 of DemonstrateStaticGrasp

In this example, we show you how to use the `DemonstrateStaticGrasp` production
to visualize a candidate grasp.

In this example we provide:
- `path_to_object`: The file path to the object that we want to pick up.
- `path_to_gripper`: The file path to the gripper that we want to use to pick up the object.
- `X_ObjectTarget`: The rigid transform of the target frame on the gripper (by default, this is the base frame) with respect to the base frame of the object. This "object-centric" representation of the gripper's pose is common in many places.