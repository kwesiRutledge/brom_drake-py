"""
DiagramTarget.py
Description:

    Creates a DiagramTarget class that can be used to monitor the state of a Diagram.
"""
from typing import NamedTuple, List, Union


class DiagramTarget(NamedTuple):
    name: str
    ports: Union[None, List[int]] = None
