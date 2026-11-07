from pydrake.geometry import SceneGraph
from pydrake.systems.sensors import RgbdSensor
from pydrake.systems.primitives import (
    VectorLogSink,
    ConstantVectorSource,
)

# List of system types that should be ineligible for diagram watching
# (RgbdSensor is excluded so that loggers do not force a render at every time step.)
INELIGIBLE_SYSTEM_TYPES = [SceneGraph, VectorLogSink, ConstantVectorSource, RgbdSensor]
