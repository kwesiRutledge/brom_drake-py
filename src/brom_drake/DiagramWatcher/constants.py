from pydrake.geometry import SceneGraph
from pydrake.systems.primitives import (
    VectorLogSink, ConstantVectorSource,
)

INELIGIBLE_SYSTEM_TYPES = [SceneGraph, VectorLogSink, ConstantVectorSource]
