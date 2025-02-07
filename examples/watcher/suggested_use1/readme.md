# Suggested Use of the DiagramWatcher

This example can be run by entering:
```python
python3 create_monitor.py
```

## Explanation 

In the script, we use the convenience function
`add_watcher_and_build` to create a DiagramWatcher for a small Drake diagram.

The diagram contains:
- the `MultibodyPlant` which contains a single cube that is free to move,
- a "block handler" object which monitors the state of the cube and allows one to change
the cube's state, and
- an `AffineSystem` which provides the commands that change the cube's pose.

Thus, there are a few components to the diagram that you might want to monitor/check when debugging or testing the diagram. In order to monitor these components, you would normally need to manually define `VectorLogger` objects for each of the components. Instead of doing that, you can monitor all of the components using
the `add_watcher_and_build` function. It will automatically add loggers to the
relevant blocks and build the diagram for you.

Then, when the simulation is done, brom saves the logged data
into several figures in the `brom` directory.

## Notes

