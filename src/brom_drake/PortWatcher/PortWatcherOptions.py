"""
PortWatcherOptions.py
Description:

    This class defines the options used when making a PortWatcher object.
"""

from enum import Enum
from typing import NamedTuple


class PortFigureArrangement(Enum):
    OnePlotPerDim = 0
    OnePlotPerPort = 1


class PortWatcherOptions(NamedTuple):
    plot_arrangement: PortFigureArrangement = PortFigureArrangement.OnePlotPerPort
    plot_dpi: int = 300
    save_to_file: bool = True
