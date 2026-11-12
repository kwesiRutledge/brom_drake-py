# Kinova Gen3 (6-DoF)

`gen3_6dof.urdf` and `meshes/` come from Kinova's
[ros_kortex](https://github.com/Kinovarobotics/ros_kortex) repository (branch `noetic-devel`):

- `kortex_description/arms/gen3/6dof/urdf/GEN3-6DOF_NO-VISION_URDF_ARM_V01.urdf`
- `kortex_description/arms/gen3/6dof/meshes/*.STL`

They are distributed under the BSD-3-Clause license (see `LICENSE`).

## Changes for brom_drake

- Mesh paths are relative (`./meshes/...`) instead of `package://kortex_description/...`.
- Joints 1, 4 and 6 are `continuous` (unlimited) on the real arm. Here they are
  `revolute` with limits of [-pi, pi], so that sampling-based motion planners have
  finite bounds to sample from.
- The robot is named `kinova_gen3_6dof` and the links are white.

The URDF has no `<transmission>` elements; `DrakeReadyURDFConverter` adds an actuator
for each of the 6 joints when the model is loaded (as `KinovaGen3Station` does).

Useful frames:

- `base_link`: the base of the arm (welded to the world by `KinovaGen3Station`).
- `end_effector_link`: the tool flange; its z-axis points out of the flange.
