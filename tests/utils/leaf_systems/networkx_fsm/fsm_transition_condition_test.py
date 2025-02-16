import unittest
# Internal Imports
from brom_drake.utils.leaf_systems.network_fsm.fsm_transition_condition import (
    FSMTransitionConditionType, FSMTransitionCondition
)

class FSMTransitionConditionTest(unittest.TestCase):
    def test_init1(self):
        """
        Description
        -----------
        This test verifies that the constructor throws an error if
        we provide:
        - NO INPUT PORT NAME 
        - the condition_type is kGreaterThanEqual
        - the condition_value is a float
        """
        # Setup

        # Try to construct the object
        try:
            condition = FSMTransitionCondition(
                input_port_name=None,
                condition_type=FSMTransitionConditionType.kGreaterThanEqual,
                condition_value=0.0,
            )
            self.assertTrue(False)
        except Exception as e:
            self.assertTrue(True)

    def test_init2(self):
        """
        Description
        -----------
        This test verifies that the constructor does not throw an error if
        we provide:
        - an INPUT PORT NAME
        - the condition_type is kLessThanEqual
        - the condition_value is a float
        """
        # Setup

        # Try to construct the object
        try:
            condition = FSMTransitionCondition(
                input_port_name="dummy_input_port_name",
                condition_type=FSMTransitionConditionType.kLessThanEqual,
                condition_value=0.0,
            )
            self.assertTrue(True)
        except Exception as e:
            self.assertTrue(False)

if __name__ == "__main__":
    unittest.main()