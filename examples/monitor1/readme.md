# Readme

## Notes

> Why are all of the ports in the connection map
> of the Drake diagram InputPorts? Shouldn't some be outputs?

It looks like I was first trying to do this with a `DiagramBuilder` object.

This object doesn't seem to have a finalized diagram. What if we finalized the
Diagram first?

Testing. So after building the Diagram, we still can't
get unique values as outputs of the connection map.
