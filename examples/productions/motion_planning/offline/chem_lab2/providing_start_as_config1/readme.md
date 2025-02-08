# Providing Start As Configuration

## How To Run This Example

From this directory, run:
```python
python3 provide_start_config1.py
```

## Description

You may have noticed that in most of the `Production` examples, we provide the start
as a "Pose" that the end effector of the robot must be in.

This is not necessary for all cases!

For several examples, you can provide your starting location as a set of joint positions (a.k.a. a configuration of the robot) instead. This can make it easier to
randomize the starting pose. (If you pick a random pose in 3D space, then the scene
may complain that the pose is not feasible for the robot to reach. In configuration
space, it is much easier to select random points because ALL configurations within the 
joint limits are feasible.)

To put the robot into your desired start configuration, use:
```python
production = ChemLab2(
    meshcat_port_number=meshcat_port_number, # Use None for CI
    start_configuration=np.zeros((6,)),
)
```

and to set the goal configuration use:

```
production = ChemLab2(
    meshcat_port_number=meshcat_port_number, # Use None for CI
    goal_configuration=np.zeros((6,)),
)
```