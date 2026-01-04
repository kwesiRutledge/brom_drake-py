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
    *Description*
    
    A signal that defines which types of messages to suppress when
    using the DiagramWatcher.

    *Attributes*

    during_port_watcher_connection: bool
        If True, suppress log messages that occur during the connection
        of PortWatchers to the DiagramWatcher.
    """
    # The list of rules to suppress the DiagramWatcher
    during_port_watcher_connection: bool = False

@dataclass
class DiagramWatcherOptions:
    """
    *Description*
    
    This class contains the options that configure the following properties of the DiagramWatcher:
    - Where it saves its data (plots, raw data, etc.)
    - the format that it uses to save its data/plots
    - what types of messages to suppress during its operation

    .. tip::

        The DiagramWatcherOptions class has useful defaults defined for ALL
        of its attributes, so in most cases, the user will not need to modify any of them.
        You can simply create a DiagramWatcherOptions object with no arguments
        and it will work out of the box. (e.g., `options = DiagramWatcherOptions()`)

    *Attributes*

    base_directory: str, optional
        The base directory where the watcher will save any data (e.g., plots, or dataframes)
        during its operation as well as logs.

    plotting_options: PortWatcherPlottingOptions, optional
        The options that configure how the PortWatcher will save its plots.

    raw_data_options: PortWatcherRawDataOptions, optional
        The options that configure how the PortWatcher will save its raw data.

    hide_messages: SuppressDiagramWatcherRules, optional
        The rules that define which types of messages to suppress during the operation of the DiagramWatcher.
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
        *Description*
        
        This function converts the DiagramWatcherOptions to a PortWatcherOptions object.

        *Returns*
        
        PortWatcherOptions
            The PortWatcherOptions object.
        """
        return PortWatcherOptions(
            plotting=self.plotting_options,
            raw_data=self.raw_data_options,
        )
    