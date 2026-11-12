# Cornell Tech workcell

Simple models of the robot workcell used by the `CornellTech1` production.
They are built from boxes and cylinders, with dimensions estimated from a photo of the
real workcell (they are not measured).

- `robot_cart.sdf`: a mobile cart made of aluminum extrusions (1.50 m x 0.75 m, top
  surface 0.42 m above the floor), with an emergency stop on the rails at the back and a
  cardboard box at the front-left. The frame `cart_body` is at the center of the cart's
  top surface; +x points along the cart's length and +y toward the back.
- `office_desk.sdf`: the desk behind the cart, with two monitors and a pedestal. The frame
  `desk_body` is on the floor, below the center of the desktop; +y points away from the cart.

Both models only have one (static) link each and should be welded to the world.
