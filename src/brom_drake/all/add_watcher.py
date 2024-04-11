"""
add_watcher.py
Description:

    This function defines a couple of convenience functions for adding in the watcher
    to a target diagram builder. These functions should be useful for 99% of the
    users of the DiagramWatcher class.
"""
from typing import List, Tuple, Union

from pydrake.all import DiagramBuilder, Diagram

from brom_drake.DiagramTarget import DiagramTarget
from brom_drake.DiagramWatcher import DiagramWatcher

PotentialTargetTypes = List[
    Union[
        str,
        Tuple[str, int],
        Tuple[int, int],
        Tuple[str, List[int]],
        Tuple[int, List[int]],
    ],
]


def add_watcher(
    builder: DiagramBuilder,
    targets: PotentialTargetTypes = None,
) -> DiagramWatcher:
    """
    add_watcher
    Description:

        This function adds a DiagramWatcher to a DiagramBuilder.

    Example Usage:

        watcher = add_watcher(builder)

        watcher = add_watcher(builder, [("plant", 0), ("controller", 0)])


    :param builder: DiagramBuilder. The diagram builder to which we want to add the watcher.
    :param targets: List[Tuple[Union[str, int]]]. The targets that we want to watch.
    :return: DiagramWatcher. The watcher that we have added to the diagram builder.
    """
    # Parse targets list if it exists
    if targets is not None:
        targets = parse_list_of_simplified_targets(builder, targets)

    watcher = DiagramWatcher(builder, targets=targets)
    return watcher


def add_watcher_and_build(
    builder: DiagramBuilder,
    targets: PotentialTargetTypes = None,
) -> (DiagramWatcher, Diagram):
    """
    add_watcher_and_build
    Description:

        This function adds a DiagramWatcher to a DiagramBuilder and then builds the diagram.

    Example Usage:

        watcher = add_watcher(builder)

        watcher = add_watcher(builder, [("plant",)])

        watcher = add_watcher(builder, [("plant", 0), ("controller", 0)])

    :param builder: DiagramBuilder. The diagram builder to which we want to add the watcher.
    :param targets: List[Tuple[Union[str, int]]]. The targets that we want to watch.
    :return: DiagramWatcher. The watcher that we have added to the diagram builder.
    """
    watcher = add_watcher(builder, targets=targets)

    # Build the diagram and add a reference to the watcher
    diagram = builder.Build()
    diagram_context = diagram.CreateDefaultContext()
    watcher.diagram = diagram
    watcher.diagram_context = diagram_context

    return watcher, diagram, diagram_context


def parse_list_of_simplified_targets(
    builder: DiagramBuilder,
    targets: PotentialTargetTypes,
) -> List[DiagramTarget]:
    """
    parse_list_of_simplified_targets
    Description:

        This function takes a list of simplified targets and converts them to the full form.

    Example Usage:

        targets = [("plant", 0), ("controller", 0)]
        parsed_targets = parse_list_of_simplified_targets(targets)

    :param builder: DiagramBuilder. The diagram builder to which we want to add the watcher.
    :param targets: List[Tuple[Union[str, int]]]. The list of simplified targets.
    :return: List[Tuple[str, int]]. The list of full targets.
    """
    # Setup
    parsed_targets = []
    system_list = builder.GetSystems()

    # If input is not a list, then raise an error
    if not isinstance(targets, list):
        raise ValueError("the input \"targets\" must be a list.")

    if len(targets) == 0:
        raise ValueError(
            "the input \"targets\" must be a non-empty list containing tuples or strings.\n" +
            "If you want to watch the entire diagram, then pass in None as the value of targets."
        )

    # Parse each element in targets
    for target in targets:
        target_name, ports_list = None, None

        if isinstance(target, str):
            target_name = target
        elif isinstance(target, tuple):
            # Parse the first element in the tuple
            if isinstance(target[0], str):
                target_name = target[0]
            elif isinstance(target[0], int):
                # Check if the index is valid
                if target[0] < 0 or target[0] >= len(system_list):
                    raise ValueError(
                        f"the index {target[0]} is not a valid index for the list of systems (has length {len(system_list)})."
                    )
                target_name = system_list[target[0]].get_name()
                print(system_list[target[0]].get_output_port())

            else:
                raise ValueError(
                    "the first element of the tuple must be a string or an integer."
                )

            # Parse the second element in the tuple
            if isinstance(target[1], int):
                ports_list = [target[1]]
            elif isinstance(target[1], list):
                ports_list = []
                for ii, elt_ii in enumerate(target[1]):
                    if isinstance(elt_ii, int):
                        ports_list.append(elt_ii)
                    elif isinstance(elt_ii, str):
                        raise NotImplementedError(
                            f"target_list[{ii}] of the tuple is not an integer (it is of type {type(elt_ii)})!.\n" +
                            "If you want this feature to be implemented, send a message to the maintainer \n" +
                            "or create a pull request on the github repository."
                        )
                    else:
                        raise ValueError(
                            f"the target_list[{ii}] of the tuple is not an integer!."
                        )
            else:
                raise ValueError(
                    "the second element of the tuple must be either a: \n" +
                    "- a list of integers\n" +
                    "- an integer\n" +
                    "- None.\n" +
                    f"Received type {type(target[1])}"
                )

        # Create the diagram target object
        dt = DiagramTarget(target_name, ports_list)
        parsed_targets.append(dt)

    return parsed_targets
