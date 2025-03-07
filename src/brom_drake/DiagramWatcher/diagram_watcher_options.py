"""
Description
-----------
    This file contains the DiagramWatcherOptions class.
"""

from dataclasses import dataclass

# Internal Imports
from brom_drake.directories import DEFAULT_WATCHER_DIR
from brom_drake.PortWatcher.port_watcher_options import PortWatcherOptions, PortWatcherPlottingOptions, PortWatcherRawDataOptions

@dataclass(frozen=True)
class SuppressDiagramWatcherRules:
    """
    Description
    -----------
    This class contains the rules for suppressing the DiagramWatcher.
    """
    # The list of rules to suppress the DiagramWatcher
    during_port_watcher_connection: bool = False

@dataclass
class DiagramWatcherOptions:
    """
    Description
    -----------
    This class contains the options for the DiagramWatcher.
    """
    # The base directory where the watcher will save the data
    base_directory: str = DEFAULT_WATCHER_DIR

    # Options Used for Each PortWatcher
    plotting_options: PortWatcherPlottingOptions = PortWatcherPlottingOptions()
    raw_data_options: PortWatcherRawDataOptions = PortWatcherRawDataOptions()

    # Flags for hiding the outputs of the DiagramWatcher
    hide_messages: SuppressDiagramWatcherRules = SuppressDiagramWatcherRules()

    def to_port_watcher_options(self) -> PortWatcherOptions:
        """
        Description
        -----------
        This function converts the DiagramWatcherOptions to a PortWatcherOptions object.

        Returns
        -------
        PortWatcherOptions
            The PortWatcherOptions object.
        """
        return PortWatcherOptions(
            base_directory=self.base_directory,
            plotting=self.plotting_options,
            raw_data=self.raw_data_options,
        )
    