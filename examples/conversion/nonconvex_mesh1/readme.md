# nonconvex_mesh1

This example shows how you can replace non-convex meshes
in a given URDF with a set of convex meshes using a built-in
feature of the `DrakeReadyURDFConverter`.

Note: This is not required. If you have a URDF with non-convex meshes, then `drakeify_my_urdf` and `DrakeReadyURDFConverter` will still do something reasonable. 

This example is mainly relevant when *you care about precise collision properties*.