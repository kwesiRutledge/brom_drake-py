"""
add_watcher.py
Description:

    This function defines a couple of convenience functions for adding in the watcher
    to a target diagram builder. These functions should be useful for 99% of the
    users of the DiagramWatcher class.
"""

from pydrake.all import DiagramBuilder, Diagram

from brom_drake.DiagramWatcher import DiagramWatcher


def add_watcher(builder: DiagramBuilder) -> DiagramWatcher:
    """
    add_watcher
    Description:

        This function adds a DiagramWatcher to a DiagramBuilder.

    :param builder: DiagramBuilder. The diagram builder to which we want to add the watcher.
    :return: DiagramWatcher. The watcher that we have added to the diagram builder.
    """
    watcher = DiagramWatcher(builder)
    return watcher


def add_watcher_and_build(builder: DiagramBuilder) -> (DiagramWatcher, Diagram):
    """
    add_watcher_and_build
    Description:

        This function adds a DiagramWatcher to a DiagramBuilder and then builds the diagram.

    :param builder: DiagramBuilder. The diagram builder to which we want to add the watcher.
    :return: DiagramWatcher. The watcher that we have added to the diagram builder.
    """
    watcher = add_watcher(builder)

    # Build the diagram and add a reference to the watcher
    diagram = builder.Build()
    diagram_context = diagram.CreateDefaultContext()
    watcher.diagram = diagram
    watcher.diagram_context = diagram_context

    return watcher, diagram, diagram_context

