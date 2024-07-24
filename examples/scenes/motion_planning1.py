"""
manipulation1.py
Description:

    In this script, we create a UR10e robot and place it in an empty environment that we'll use for motion_planning
    work.
"""
import ipdb
import typer

from brom_drake.robots import UR10eStation, GripperType
from brom_drake.control import CartesianArmController, GripperController

def main():
    # Create a gripper controller for the 2f85 gripper
    gripper_controller = GripperController(GripperType.Robotiq_2f_85)

    # Create UR10e object
    station = UR10eStation()


    # Create a Cartesian controller
    # controller = CartesianArmController(station.plant, station.arm)


if __name__ == "__main__":
    with ipdb.launch_ipdb_on_exception():
        typer.run(main)
