from dataclasses import dataclass
import os
from pathlib import Path
import shutil
from typing import Union

# Internal Imports
from brom_drake.directories import DEFAULT_BROM_MODELS_DIR

@dataclass
class DrakeReadyURDFConverterFileManager:
    """
    Description
    -----------
    A dataclass that specifies how the DrakeReadyURDFConverter
    should manage files.
    """
    original_urdf_path: Path
    models_directory: Path = Path(DEFAULT_BROM_MODELS_DIR)
    log_file_name: str = None
    _output_urdf_path: Union[str, Path] = None

    def clean_up_models_dir(self):
        """
        Description
        -----------
        This method will clean up the models directory.
        """
        # Make sure that the models directory exists
        os.makedirs(self.models_directory, exist_ok=True) # TODO: Can we just use Pathlib here to check for existence?
        shutil.rmtree(self.models_directory, ignore_errors=True)

    def output_file_directory(self) -> Path:
        """
        Description
        -----------
        Returns the directory where the output URDF file will be saved.
        If the output_urdf_file_path is not specified, it defaults to the
        models_directory.

        Returns
        -------
        Path
            The directory where the output URDF file will be saved.
        """
        return self.output_urdf_path.parent
    
    def file_path_in_context_of_urdf_dir(self, file_path: Union[str, Path]) -> Path:
        """
        Description
        -----------
        This method returns the given file_path's Path when taken into context
        of where the urdf lies.

        Parameters
        ----------
        file_path: str or Path
            The file path to convert

        Returns
        -------
        Path
            The file path with added context of the INPUT URDF file.
        """
        # Setup
        if type(file_path) == str:
            file_path = Path(file_path)

        original_urdf_directory = self.original_urdf_path.parent
        return original_urdf_directory / file_path

    def output_urdf_file_name(self) -> str:
        """
        Description
        -----------
        Creates the output filename based on the original filename.

        Returns
        -------
        str
            The name of the output URDF file
        """
        # Setup
        original_file_path = self.original_urdf_path
        output_urdf_file_path = self._output_urdf_path

        if output_urdf_file_path is None: # If the output file path was not given.
            # Use Pathlib to extract the filename, if it exists
            original_name = original_file_path.name
            original_name = original_name.replace(".urdf", "")

            return f"{original_name}.drake.urdf"
        else:
            return Path(output_urdf_file_path).name

    @property
    def output_urdf_path(self) -> Path:
        """
        Description
        -----------
        Returns the path where the output URDF file will be saved.
        If the output_urdf_file_path is not specified, it defaults to the
        models_directory with the same name as the original URDF file.

        Returns
        -------
        Path
            The path where the output URDF file will be saved.
        """
        # Setup
        original_urdf_path = self.original_urdf_path

        # Algorithm
        if self._output_urdf_path is not None:
            if type(self._output_urdf_path) == str:
                self._output_urdf_path = Path(self._output_urdf_path)

            return self._output_urdf_path
        else:
            original_urdf_filename = original_urdf_path.name
            new_subdirectory_name = original_urdf_filename.replace(".urdf", "")
            return self.models_directory / new_subdirectory_name / self.output_urdf_file_name()

    


        
