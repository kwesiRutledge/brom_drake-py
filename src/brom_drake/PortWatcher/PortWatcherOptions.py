"""
PortWatcherOptions.py
Description:

    This class defines the options used when making a PortWatcher object.
"""

from enum import Enum, IntEnum
from typing import NamedTuple


class PortFigureArrangement(Enum):
    OnePlotPerDim = 0
    OnePlotPerPort = 1

class FigureNamingConvention(IntEnum):
    kFlat = 0 # e.g. "plant_generalized_output_dim_0.png"
    kHierarchical = 1 # e.g. "system_plant/port_generalized_output/dim_0.png"

class PortWatcherOptions(NamedTuple):
    plot_arrangement: PortFigureArrangement = PortFigureArrangement.OnePlotPerPort
    plot_dpi: int = 300
    save_to_file: bool = True
    figure_naming_convention: FigureNamingConvention = FigureNamingConvention.kFlat
