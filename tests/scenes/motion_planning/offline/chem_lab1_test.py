import unittest

from pydrake.systems.analysis import Simulator

from brom_drake.scenes.motion_planning.offline.chem_lab1 import ChemLab1Scene


class ChemLab1Test(unittest.TestCase):
    def test_add_all_secondary_cast_members_to_builder1(self):
        # Setup
        scene = ChemLab1Scene(meshcat_port_number=None)

        # Call the method
        scene.add_all_secondary_cast_members_to_builder()

        # Build the diagram
        diagram = scene.builder.Build()
        diagram_context = diagram.CreateDefaultContext()

        # Simulate the diagram
        simulator = Simulator(diagram, diagram_context)
        simulator.set_target_realtime_rate(1.0)
        simulator.Initialize()
        simulator.AdvanceTo(1.0)

        self.assertTrue(True)


if __name__ == '__main__':
    unittest.main()