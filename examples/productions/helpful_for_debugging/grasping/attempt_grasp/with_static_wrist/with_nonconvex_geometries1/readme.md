# with_nonconvex_geometries1

This example shows how you can combine the `drakeify_my_urdf` feature
(and its ability to generate more complicated contact geometries)
in a test with the `AttemptGrasp` production.

Specifically, we use the `collision_mesh_replacement_strategy` input to `drakeify_my_urdf` to request that the nonconvex geometry of the object be transformed into multiple convex parts. This can be done by providing the value `MeshReplacementStrategy.kWithConvexDecomposition` (see example for exact details).