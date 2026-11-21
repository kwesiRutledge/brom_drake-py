import os
from pathlib import Path


def get_brom_dir() -> Path:
    """Resolve the output root at use time, honoring BROM_DIR when non-empty."""
    return Path(os.environ.get("BROM_DIR") or "./brom")


DEFAULT_BROM_DIR = Path("./brom")  #: Default directory for BROM Drake data storage
DEFAULT_WATCHER_DIR = (
    DEFAULT_BROM_DIR / "watcher"
)  #: Default subdirectory for :py:class:`PortWatcher<brom_drake.PortWatcher.port_watcher.PortWatcher>` data
DEFAULT_PLOT_DIR = (
    DEFAULT_WATCHER_DIR / "plots"
)  #: Default subdirectory for :py:class:`PortWatcher<brom_drake.PortWatcher.port_watcher.PortWatcher>` plots
DEFAULT_RAW_DATA_DIR = (
    DEFAULT_WATCHER_DIR / "raw_data"
)  #: Default subdirectory for :py:class:`PortWatcher<brom_drake.PortWatcher.port_watcher.PortWatcher>` raw data
DEFAULT_BROM_MODELS_DIR = (
    DEFAULT_BROM_DIR / "models"
)  #: Default subdirectory for BROM Drake models
DEFAULT_NETWORKX_FSM_DIR = (
    DEFAULT_BROM_DIR / "networkx_fsm"
)  #: Default subdirectory for NetworkX FSM data
DEFAULT_PRODUCTION_REPORTS_DIR = (
    DEFAULT_BROM_DIR / "productions"
)  #: Default subdirectory for production reports
DEFAULT_VIDEO_DIR = (
    DEFAULT_BROM_DIR / "videos"
)  #: Default subdirectory for videos recorded by :py:class:`VideoRecorder<brom_drake.systems.video.VideoRecorder>`
