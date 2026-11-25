"""
Helpers for locating model files (URDF/SDF) given by callers.
"""

from pathlib import Path
from typing import Union

PACKAGE_URL_PREFIX = "package://"


def is_package_url(path: Union[str, Path]) -> bool:
    """
    *Description*

    Whether ``path`` is a Drake ``package://`` URL (for example
    ``package://drake_models/ycb/010_potted_meat_can.sdf``).
    """
    return isinstance(path, str) and path.startswith(PACKAGE_URL_PREFIX)


def resolve_model_path(path: Union[str, Path]) -> str:
    """
    *Description*

    Turns a model location given by a caller into a file path.

    - A ``package://`` URL is resolved with Drake's default ``PackageMap``,
      which knows ``drake``, ``drake_models`` and any package on
      ``ROS_PACKAGE_PATH``. Resolving a ``drake_models`` URL for the first
      time downloads the models archive that Drake pins (cached in
      ``~/.cache/drake/package_map``), so it needs network access once.
    - Anything else is treated as a file path and returned unchanged (as a
      ``str``).

    *Parameters*

    path: str | Path
        A file path or a ``package://`` URL.

    *Returns*

    resolved_path: str
        A path to a file on disk (for a URL) or ``path`` itself.

    *Raises*

    ValueError
        If ``path`` is a ``package://`` URL that Drake cannot resolve.
    """
    if not is_package_url(path):
        return str(path)

    # Imported here so this module can be imported without loading Drake.
    from pydrake.multibody.parsing import PackageMap

    try:
        return PackageMap().ResolveUrl(path)
    except RuntimeError as e:
        raise ValueError(f"Could not resolve the model URL {path!r}: {e}") from e
