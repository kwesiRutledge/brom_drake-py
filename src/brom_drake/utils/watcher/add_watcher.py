"""
add_watcher.py
Description:

    This function defines a couple of convenience functions for adding in the watcher
    to a target diagram builder. These functions should be useful for 99% of the
    users of the DiagramWatcher class.
"""
from typing import List, Tuple, Union

import numpy as np
from pydrake.all import DiagramBuilder, Diagram
from pydrake.systems.framework import Context

from brom_drake.DiagramTarget import DiagramTarget
from brom_drake.DiagramWatcher import DiagramWatcher, DiagramWatcherOptions
from brom_drake.directories import DEFAULT_PLOT_DIR, DEFAULT_RAW_DATA_DIR, DEFAULT_WATCHER_DIR
from brom_drake.PortWatcher.port_watcher_options import (
    PortFigureArrangement,
    PortWatcherPlottingOptions,
    PortWatcherRawDataOptions,
    FigureNamingConvention,
    PortWatcherOptions,
)

PotentialTargetTypes = List[
    Union[
        str,
        Tuple[str, int],
        Tuple[int, int],
        Tuple[str, str],
        Tuple[str, List[int]],
        Tuple[int, List[int]],
        Tuple[str, List[str]],
    ],
]


def add_watcher(
    builder: DiagramBuilder,
    targets: PotentialTargetTypes = None,
    watcher_dir: str = DEFAULT_WATCHER_DIR,
    plot_arrangement: PortFigureArrangement = PortFigureArrangement.OnePlotPerPort,
    figure_naming_convention: FigureNamingConvention = FigureNamingConvention.kFlat,
    file_format: str = "png",
) -> DiagramWatcher:
    """
    Description:

        This function adds a DiagramWatcher to a DiagramBuilder.

    Example Usage:

        watcher = add_watcher(builder)

        watcher = add_watcher(builder, [("plant", 0), ("controller", 0)])

    :param builder: DiagramBuilder. The diagram builder to which we want to add the watcher.
    :param targets: List[Tuple[Union[str, int]]]. The targets that we want to watch.
    :param data_dir: str. The directory in which we will store the data collected by the DiagramWatcher.
    :param plot_arrangement: PortFigureArrangement. The arrangement of the plots.
        (Can be PortFigureArrangement.OnePlotPerPort OR PortFigureArrangement.OnePlotPerDim)
    :param figure_naming_convention: FigureNamingConvention. The naming convention for the figures.
        (Can be FigureNamingConvention.kFlat OR FigureNamingConvention.kHierarchical)
    :return: DiagramWatcher. The watcher that we have added to the diagram builder.
    """
    # Input Processing
    if not isinstance(plot_arrangement, PortFigureArrangement):
        raise ValueError(
            f"plot_arrangement must be of type PortFigureArrangement; received {plot_arrangement} of type" +
            f" {type(plot_arrangement)}."
        )

    # Parse targets list if it exists
    if targets is not None:
        targets = parse_list_of_simplified_targets(builder, targets)

    watcher = DiagramWatcher(
        builder,
        targets=targets,
        options=DiagramWatcherOptions(
            base_directory=watcher_dir,
            plotting_options=PortWatcherPlottingOptions(
                plot_arrangement=plot_arrangement,
                figure_naming_convention=figure_naming_convention,
                file_format=file_format,
            ),
            raw_data_options=PortWatcherRawDataOptions(
                save_to_file=True,
            ),
        )
    )
    return watcher


def add_watcher_and_build(
    builder: DiagramBuilder,
    targets: PotentialTargetTypes = None,
    watcher_dir: str = DEFAULT_WATCHER_DIR,
    plot_arrangement: PortFigureArrangement = PortFigureArrangement.OnePlotPerPort,
    figure_naming_convention: FigureNamingConvention = FigureNamingConvention.kFlat,
    file_format: str = "png",
) -> Tuple[DiagramWatcher, Diagram, Context]:
    """
    add_watcher_and_build
    Description:

        This function adds a DiagramWatcher to a DiagramBuilder and then builds the diagram.

    Example Usage:

        watcher = add_watcher(builder)

        watcher = add_watcher(builder, [("plant",)])

        watcher = add_watcher(builder, [("plant", 0), ("controller", 0)])

    Arguments
    ---------
    builder: DiagramBuilder
        The diagram builder to which we want to add the watcher.
    targets: List[Tuple[Union[str, int]]]
        The targets that we want to watch.
    data_dir: str
        The directory in which we will store the data collected by the DiagramWatcher.
    plot_arrangement: PortFigureArrangement
        The arrangement of the plots.
        (Can be PortFigureArrangement.OnePlotPerPort OR PortFigureArrangement.OnePlotPerDim)
    figure_naming_convention: FigureNamingConvention
        The naming convention for the figures.
        (Can be FigureNamingConvention.kFlat OR FigureNamingConvention.kHierarchical)
    file_format: str
        The file format for the figures.
    
    Returns
    -------
    :return: DiagramWatcher. The watcher that we have added to the diagram builder.
    """
    watcher = add_watcher(
        builder,
        targets=targets,
        watcher_dir=watcher_dir,
        plot_arrangement=plot_arrangement,
        figure_naming_convention=figure_naming_convention,
        file_format=file_format,
    )

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
    Description
    -----------
    This function takes a list of simplified targets and converts them to the full form.

    Example Usage
    -------------
    targets = [("plant", 0), ("controller", 0)]
    parsed_targets = parse_list_of_simplified_targets(targets)

    Arguments
    ---------
    builder : DiagramBuilder
        The diagram builder to which we want to add the watcher.
    targets : List[Tuple[Union[str, int]]
        The list of simplified targets.

    Returns
    -------
    List[DiagramTarget]
        A list of all the targets that we want to watch.
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

            else:
                raise ValueError(
                    "the first element of the tuple must be a string or an integer; received {} (type {}).".format(
                        target[0], type(target[0])
                    )
                )

            # Parse the second element in the tuple
            if isinstance(target[1], int):
                ports_list = [target[1]]
            elif isinstance(target[1], str):
                # Find the index of port with this name
                port_name = target[1]

                # Find the system
                system = None
                for ii, system_ii in enumerate(system_list):
                    if system_ii.get_name() == target_name:
                        system = system_ii
                        break

                # for jj in range(system.num_output_ports()):
                #     print(system.get_output_port(jj).get_name())

                if system.HasOutputPort(port_name):
                    ports_list = [int(system.GetOutputPort(port_name).get_index())]
                else:
                    raise ValueError(
                        f"the system {target_name} does not have an output port named {port_name}."
                    )


            elif isinstance(target[1], list):
                ports_list = []
                for ii, elt_ii in enumerate(target[1]):
                    if isinstance(elt_ii, int):
                        ports_list.append(elt_ii)
                    elif isinstance(elt_ii, str):
                        port_name = elt_ii
                        # Find the system
                        system = None
                        for ii, system_ii in enumerate(system_list):
                            if system_ii.get_name() == target_name:
                                system = system_ii
                                break

                        if system.HasOutputPort(port_name):
                            ports_list += [int(system.GetOutputPort(port_name).get_index())]
                        else:
                            raise ValueError(
                                f"the system {target_name} does not have an output port named {port_name}."
                            )

                    else:
                        raise ValueError(
                            f"the target_list[{ii}] of the tuple is not an integer or a string! " +
                            f"Received {elt_ii} of type {type(elt_ii)}."
                        )
            else:
                raise ValueError(
                    "the second element of the tuple must be either a: \n" +
                    "- an integer\n" +
                    "- a list of integers\n" +
                    "- a string\n" +
                    "- a list of strings\n" +
                    "- None.\n" +
                    f"Received type {type(target[1])}"
                )

        # Create the diagram target object
        dt = DiagramTarget(target_name, ports_list)
        parsed_targets.append(dt)

    return parsed_targets
