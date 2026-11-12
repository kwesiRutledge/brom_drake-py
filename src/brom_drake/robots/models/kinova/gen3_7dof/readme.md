# Kinova Gen3 (7-DoF)

`gen3_7dof.urdf` and `meshes/` come from Kinova's
[ros_kortex](https://github.com/Kinovarobotics/ros_kortex) repository (branch `noetic-devel`):

- `kortex_description/arms/gen3/7dof/urdf/GEN3-7DOF-NOVISION_FOR_URDF_ARM_V12.urdf`
- `kortex_description/arms/gen3/7dof/meshes/*.STL`

They are distributed under the BSD-3-Clause license (see `LICENSE`).

## Changes for brom_drake

- Mesh paths are relative (`./meshes/...`) instead of `package://kortex_description/...`.
- Joints 1, 3, 5 and 7 are `continuous` (unlimited) on the real arm. Here they are
  `revolute` with limits of [-pi, pi], so that sampling-based motion planners have
  finite bounds to sample from.
- The robot is named `kinova_gen3_7dof` and the links are white.

The URDF has no `<transmission>` elements; `DrakeReadyURDFConverter` adds an actuator
for each of the 7 joints when the model is loaded (as `KinovaGen3Station` does).

Useful frames:

- `base_link`: the base of the arm (welded to the world by `KinovaGen3Station`).
- `end_effector_link`: the tool flange; its z-axis points out of the flange.
