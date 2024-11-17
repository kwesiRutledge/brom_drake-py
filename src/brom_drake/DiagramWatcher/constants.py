from pydrake.geometry import SceneGraph
from pydrake.systems.primitives import (
    VectorLogSink, ConstantVectorSource,
)

INELIGIBLE_SYSTEM_TYPES = [SceneGraph, VectorLogSink, ConstantVectorSource]

DEFAULT_BROM_DIR = "./brom"
DEFAULT_PLOT_DIR = "./brom/watcher_plots"
DEFAULT_BROM_MODELS_DIR = "./brom/models"