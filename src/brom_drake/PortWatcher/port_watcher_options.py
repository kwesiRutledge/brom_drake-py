"""
PortWatcherOptions.py
Description:

    This class defines the options used when making a PortWatcher object.
"""

from enum import Enum, IntEnum
from typing import NamedTuple

# Internal Imports
from brom_drake.directories import DEFAULT_PLOT_DIR, DEFAULT_RAW_DATA_DIR, DEFAULT_WATCHER_DIR
from .port_figure_arrangement import PortFigureArrangement

class FigureNamingConvention(IntEnum):
    """
    Description
    -----------
    This enum is used to define the naming convention for the figures.
    """
    kFlat = 0 # e.g. "plant_generalized_output_dim_0.png"
    kHierarchical = 1 # e.g. "system_plant/port_generalized_output/dim_0.png"

class PortWatcherPlottingOptions(NamedTuple):
    plot_arrangement: PortFigureArrangement = PortFigureArrangement.OnePlotPerPort
    plot_dpi: int = 300
    save_to_file: bool = True
    file_format: str = "png"
    figure_naming_convention: FigureNamingConvention = FigureNamingConvention.kFlat

class PortWatcherRawDataOptions(NamedTuple):
    save_to_file: bool = True
    file_format: str = "npy"

class PortWatcherOptions(NamedTuple):
    base_directory: str = DEFAULT_WATCHER_DIR
    plotting: PortWatcherPlottingOptions = PortWatcherPlottingOptions()
    raw_data: PortWatcherRawDataOptions = PortWatcherRawDataOptions()

    def plot_dir(self) -> str:
        """
        Description
        -----------
        This function returns the directory where the plots will be saved.

        Returns
        -------
        str
            The directory where the plots will be saved.
        """
        return self.base_directory + "/plots"

    def raw_data_dir(self):
        """
        Description
        -----------
        This function returns the directory where the raw data will be saved.

        Returns
        -------
        str
            The directory where the raw data will be saved.
        """
        return self.base_directory + "/raw_data"
