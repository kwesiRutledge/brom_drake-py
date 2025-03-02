from importlib import resources as impresources
import ipdb
import numpy as np
from pydrake.all import (
    Simulator,
)
import typer

# Internal Imports
from brom_drake.all import (
    drakeify_my_urdf,
)
from brom_drake.motion_planning.algorithms.rrt.bidirectional_connect import (
    BidirectionalRRTConnectPlanner,
    BidirectionalRRTConnectPlannerConfig,
    BiRRTConnectSamplingProbabilities,
)
from brom_drake.productions import ChemLab3
from brom_drake import robots


def main(meshcat_port_number: int = 7001):
    # # Setup
    # flask_path = impresources.files(robots) / "models/erlenmeyer_flask/500ml.urdf"

    # # Convert the URDF
    # new_urdf_path = drakeify_my_urdf(
    #     flask_path,
    #     overwrite_old_logs=True,
    # )

    # # Create the production
    # production = ShowMeThisModel(
    #     str(impresources.files(robots) / "models/cupboard/cupboard.sdf"),
    #     base_link_name="cupboard_body",
    #     with_these_joint_positions=np.array([-np.pi*(3.0/4.0), np.pi*(3.0/4.0)])
    # )

    raise NotImplementedError(
        "This example is not working yet. " +
        "Please wait for future versions of brom_drake to fix this."
    )

    # Setup
    if meshcat_port_number < 0:
        meshcat_port_number = None

    # Create the production
    production = ChemLab3(
        meshcat_port_number=meshcat_port_number, # Use None for CI
    )

    # Create a planner object which will be used to plan the motion
    config = BidirectionalRRTConnectPlannerConfig(
        steering_step_size=0.1,
        debug=False,
    )
    planner2 = BidirectionalRRTConnectPlanner(
        production.arm,
        production.plant,
        production.scene_graph,
        config=config,
    )

    # To build the production, we only need to provide a planning function
    # (can come from anywhere, not just a BaseRRTPlanner object)
    diagram, diagram_context = production.easy_cast_and_build(
        planner2.plan,
        with_watcher=True,
    )

    print("Simulating...")

    # Simulate the diagram
    simulator = Simulator(diagram, diagram_context)
    simulator.set_target_realtime_rate(1.0)

    # Run simulation
    simulator.Initialize()
    simulator.AdvanceTo(20.0)


if __name__ == "__main__":
    with ipdb.launch_ipdb_on_exception():
        typer.run(main)
