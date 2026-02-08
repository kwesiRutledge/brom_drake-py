import unittest

from pydrake.multibody.plant import MultibodyPlant, AddMultibodyPlantSceneGraph
from pydrake.systems.framework import DiagramBuilder

from brom_drake.example_helpers import BlockHandlerSystem
from brom_drake.stations.kinematic import UR10eStation
from brom_drake.productions.roles.all import Role, RolePortAssignment, PairingType


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
        performer = MultibodyPlant(time_step=0.01)

        # Define a bad role
        rpa1 = RolePortAssignment(
            external_target_name="joint_positions",
            performer_port_name="wile out",
            pairing_type=PairingType.kInput,
        )
        bad_role = Role(
            name="Bad Role",
            description="This role has a weird input definition that shouldraise an error.",
            port_assignments=[
                rpa1,
            ],
        )

        # Call the method
        try:
            bad_role.connect_performer_ports_to(builder, performer)
            self.assertTrue(False)
        except Exception as e:
            self.assertIn(str(e), str(rpa1.create_assignment_port_unavailable_error()))

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
        performer = MultibodyPlant(time_step=0.01)

        # Define a bad role
        rpa2 = RolePortAssignment(
            external_target_name="joint_positions",
            performer_port_name="wile out",
            pairing_type=PairingType.kOutput,
        )
        bad_role = Role(
            name="Bad Role",
            description="This role has a weird output definition that should raise an error.",
            port_assignments=[rpa2],
        )

        # Call the method
        try:
            bad_role.connect_performer_ports_to(builder, performer)
            self.assertTrue(False)
        except Exception as e:
            self.assertIn(
                str(e),
                str(rpa2.create_assignment_port_unavailable_error()),
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
        ur10e_station = UR10eStation(
            meshcat_port_number=None,  # Use None for CI
        )
        ur10e_station.Finalize()
        performer = ur10e_station

        # Add a system to the builder
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
        plant.set_name("plant2")
        block_handler = builder.AddSystem(BlockHandlerSystem(plant, scene_graph))

        # Define a bad role
        rpa3 = RolePortAssignment(
            external_target_name="block_handler",
            performer_port_name="desired_joint_positions",
            pairing_type=PairingType.kInput,
        )
        bad_role = Role(
            name="Bad Role",
            description="This role has a weird input definition that should raise an error.",
            port_assignments=[rpa3],
        )

        # Call the method
        try:
            bad_role.connect_performer_ports_to(builder, performer)
            self.assertTrue(False)
        except Exception as e:
            self.assertIn(
                str(e),
                str(rpa3.create_no_target_found_error()),
            )


if __name__ == "__main__":
    unittest.main()
