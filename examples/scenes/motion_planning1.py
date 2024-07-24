"""
manipulation1.py
Description:

    In this script, we create a UR10e robot and place it in an empty environment that we'll use for motion_planning
    work.
"""
import ipdb
import typer

from brom_drake.robots import UR10eStation
from brom_drake.control import CartesianArmController

def main():
    # Create UR10e object
    station = UR10eStation()


    # Create a Cartesian controller
    controller = CartesianArmController(station.plant, None)



if __name__ == "__main__":
    with ipdb.launch_ipdb_on_exception():
        typer.run(main)
