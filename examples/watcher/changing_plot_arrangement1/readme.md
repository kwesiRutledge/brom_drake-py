# Readme

In this script, we use the convenience function
`add_watcher_and_build` to create a DiagramWatcher and build it into an example Drake diagram.

The system is a cube-like object whose dynamics are governed
by an `AffineSystem`. While the user would normally have to add loggers
to most of the blocks in the diagram to monitor the state of the system,
the `add_watcher_and_build` function will automatically add loggers to the
relevant blocks and build the diagram for you.

Then, when the simulation is done, brom saves the logged data
into several figures in the `brom` directory.
## Notes
