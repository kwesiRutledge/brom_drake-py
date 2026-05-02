"""
PortWatcherOptions.py
Description:

    This class defines the options used when making a PortWatcher object.
"""

from enum import Enum, IntEnum
from typing import NamedTuple

# Internal Imports
from brom_drake.directories import (
    DEFAULT_PLOT_DIR,
    DEFAULT_RAW_DATA_DIR,
    DEFAULT_WATCHER_DIR,
)
from brom_drake.watchers.port_watcher.file_naming_convention import PathOrganizationConvention
from .port_figure_arrangement import PortFigureArrangement


class FigureNamingConvention(IntEnum):
    """
    **Description**

    This enum is used to define the naming convention for the figures.
    """

    kFlat = 0  # e.g. "plant_generalized_output_dim_0.png"
    kHierarchical = 1  # e.g. "system_plant/port_generalized_output/dim_0.png"


class PortWatcherPlottingOptions(NamedTuple):
    plot_arrangement: PortFigureArrangement = PortFigureArrangement.OnePlotPerPort
    plot_dpi: int = 300
    save_to_file: bool = True
    file_format: str = "png"
    figure_naming_convention: FigureNamingConvention = FigureNamingConvention.kFlat


class PortWatcherRawDataOptions(NamedTuple):
    save_to_file: bool = True
    file_format: str = "npy"
    file_organization_convention: PathOrganizationConvention = PathOrganizationConvention.kHierarchical


class PortWatcherOptions(NamedTuple):
    plotting: PortWatcherPlottingOptions = PortWatcherPlottingOptions()
    raw_data: PortWatcherRawDataOptions = PortWatcherRawDataOptions()
