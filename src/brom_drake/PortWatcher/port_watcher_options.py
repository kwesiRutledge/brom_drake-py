"""
PortWatcherOptions.py
Description:

    This class defines the options used when making a PortWatcher object.
"""

from enum import Enum, IntEnum
from typing import NamedTuple

from .port_figure_arrangement import PortFigureArrangement

class FigureNamingConvention(IntEnum):
    kFlat = 0 # e.g. "plant_generalized_output_dim_0.png"
    kHierarchical = 1 # e.g. "system_plant/port_generalized_output/dim_0.png"

class PortWatcherOptions(NamedTuple):
    plot_arrangement: PortFigureArrangement = PortFigureArrangement.OnePlotPerPort
    plot_dpi: int = 300
    save_to_file: bool = True
    file_format: str = "png"
    figure_naming_convention: FigureNamingConvention = FigureNamingConvention.kFlat
