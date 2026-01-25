from pydrake.geometry import SceneGraph
from pydrake.systems.primitives import (
    VectorLogSink, ConstantVectorSource,
)

# List of system types that should be ineligible for diagram watching
INELIGIBLE_SYSTEM_TYPES = [SceneGraph, VectorLogSink, ConstantVectorSource]