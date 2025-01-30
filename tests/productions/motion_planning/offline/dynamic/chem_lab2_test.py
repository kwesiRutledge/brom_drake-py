import unittest

# Internal Imports
from brom_drake.productions.motion_planning.offline.dynamic.chem_lab2 import ChemLab2

class ChemLab2Test(unittest.TestCase):
    def test_init1(self):
        """
        Description
        -----------
        This test verifies that we can initialize the ChemLab2 without any errors.
        """
        # Setup
        production = ChemLab2(
            meshcat_port_number=None,
        )

        self.assertTrue(True)

if __name__ == "__main__":
    unittest.main()