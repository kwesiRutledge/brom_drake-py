from dataclasses import dataclass
from pathlib import Path

@dataclass
class PortWatcherFileManager:
    """
    *Description*
    
    This class manages file paths and directories for saving data
    collected by the PortWatcher system.
    """
    base_directory: Path
    raw_data_file_format: str = "npy"

    def compute_safe_system_name(self, system_name: str) -> str:
        """
        *Description*
        
        This function returns a filesystem-safe version of the system name.

        *Returns*

        safe_system_name: str
            The filesystem-safe version of the system name.
        """
        return system_name.replace("/", "_").replace(" ", "_")

    @property
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
        return self.base_directory / "plots"

    @property
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
        return self.base_directory / "raw_data"

    def raw_data_file_name(self, system_name: str, port_name: str) -> Path:
        """
        *Description*
        
        This function returns the file name for saving raw data.

        *Returns*

        raw_data_file_name: Path
            The file name for saving raw data.
        """
        safe_system_name = self.compute_safe_system_name(system_name)
        return self.raw_data_dir / f"system_{safe_system_name}_port_{port_name}_data.{self.raw_data_file_format}"

    def time_data_file_name(self, system_name: str, port_name: str) -> Path:
        """
        *Description*
        
        This function returns the file name for saving time data.

        *Returns*

        time_data_file_name: Path
            The file name for saving time data.
        """
        safe_system_name = self.compute_safe_system_name(system_name)
        return self.raw_data_dir / f"system_{safe_system_name}_port_{port_name}_times.{self.raw_data_file_format}"
    