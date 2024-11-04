import unittest

from pydrake.multibody.plant import MultibodyPlant, AddMultibodyPlantSceneGraph
from pydrake.systems.framework import DiagramBuilder

from brom_drake.example_helpers import BlockHandlerSystem
from brom_drake.robots import UR10eStation
from brom_drake.scenes.roles import Role, RolePortAssignment


class RoleTest(unittest.TestCase):
    def test_connect_performer_to_system1(self):
        """
        Description
        -----------
        This test checks the connect_performer_to_system method of the Role class.
        It checks that the method returns the proper error if the performer does
        not have the required input ports.
        :return:
        """
        # Setup
        builder = DiagramBuilder()
        performer = builder.AddSystem(
            MultibodyPlant(time_step=0.01)
        )

        # Define a bad role
        bad_role = Role(
            name="Bad Role",
            description="This role has a weird input definition that shouldraise an error.",
            input_definitions=[
                RolePortAssignment(
                    external_target_name="joint_positions",
                    performer_port_name="wile out",
                )
            ],
            output_definitions=[],
        )

        # Call the method
        try:
            bad_role.connect_performer_to_diagram(builder, performer)
            self.assertTrue(False)
        except AssertionError as e:
            self.assertIn(
                str(e),
                "Performer does not have input port \"wile out\""
            )

    def test_connect_performer_to_system2(self):
        """
        Description
        -----------
        This test checks the connect_performer_to_system method of the Role class.
        It checks that the method returns the proper error if the performer does
        not have the required output ports.
        :return:
        """
        # Setup
        builder = DiagramBuilder()
        performer = builder.AddSystem(
            MultibodyPlant(time_step=0.01)
        )

        # Define a bad role
        bad_role = Role(
            name="Bad Role",
            description="This role has a weird output definition that should raise an error.",
            input_definitions=[],
            output_definitions=[
                RolePortAssignment(
                    external_target_name="joint_positions",
                    performer_port_name="wile out",
                )
            ],
        )

        # Call the method
        try:
            bad_role.connect_performer_to_diagram(builder, performer)
            self.assertTrue(False)
        except AssertionError as e:
            self.assertIn(
                str(e),
                "Performer does not have output port \"wile out\""
            )

    def test_connect_performer_to_system3(self):
        """
        Description
        -----------
        This test checks the connect_performer_to_system method of the Role class.
        It checks that the method returns the proper error if the external systems do
        not have the required outpput ports.
        :return:
        """
        # Setup
        builder = DiagramBuilder()
        ur10e_station = UR10eStation()
        ur10e_station.Finalize()
        performer = builder.AddSystem(ur10e_station)

        # Add a system to the builder
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
        plant.set_name("plant2")
        block_handler = builder.AddSystem(
            BlockHandlerSystem(plant, scene_graph)
        )

        # Define a bad role
        bad_role = Role(
            name="Bad Role",
            description="This role has a weird input definition that shouldraise an error.",
            input_definitions=[
                RolePortAssignment(
                    external_target_name="joint_positions",
                    performer_port_name="ee_target",
                )
            ],
            output_definitions=[],
        )

        # Call the method
        try:
            bad_role.connect_performer_to_diagram(builder, performer)
            self.assertTrue(False)
        except AssertionError as e:
            expected_error = "Expected 1 system to have port \"joint_positions\", but found 0 systems with that output port."
            self.assertIn(
                str(e),
                expected_error,
            )

if __name__ == "__main__":
    unittest.main()