# Readme

In this example, we illustrate how you can create a DiagramWatcher WITHOUT the convenience function (i.e., `add_watcher_and_build()`).
This might be useful for more advanced developers in the Drake ecosystem.

Note, that you must set the `diagram_context` and `diagram` member variables in the watcher after building
the diagram yourself.

## Notes

> Why are all of the ports in the connection map
> of the Drake diagram InputPorts? Shouldn't some be outputs?

It looks like I was first trying to do this with a `DiagramBuilder` object.

This object doesn't seem to have a finalized diagram. What if we finalized the
Diagram first?

Testing. So after building the Diagram, we still can't
get unique values as outputs of the connection map.
