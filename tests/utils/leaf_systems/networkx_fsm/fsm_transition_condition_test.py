import numpy as np
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

    def test_evaluate_comparison1(self):
        """
        Description
        -----------
        This test verifies that the evaluate_comparison method throws an error if
        we call it using the kAfterThisManySeconds condition type.
        """
        # Setup
        condition = FSMTransitionCondition(
            input_port_name="dummy_input_port_name",
            condition_type=FSMTransitionConditionType.kAfterThisManySeconds,
            condition_value=0.1,
        )

        # Try to evaluate the comparison
        try:
            condition.evaluate_comparison(0.1)
            self.assertTrue(False)
        except Exception as e:
            self.assertTrue(True)

    def test_evaluate_comparison2(self):
        """
        Description
        -----------
        This test verifies that the evaluate_comparison method returns True if
        we call it using the kEqual condition type and the input port value is
        a boolean.
        """
        # Setup
        condition = FSMTransitionCondition(
            input_port_name="dummy_input_port_name",
            condition_type=FSMTransitionConditionType.kEqual,
            condition_value=True,
        )

        # Evaluate the comparison
        result = condition.evaluate_comparison(True)
        self.assertTrue(result)

    def test_evaluate_comparison3(self):
        """
        Description
        -----------
        This test verifies that the evaluate_comparison method returns False if
        we call it using the kEqual condition type and the input port value is
        a numpy array.
        """
        # Setup
        condition = FSMTransitionCondition(
            input_port_name="dummy_input_port_name",
            condition_type=FSMTransitionConditionType.kEqual,
            condition_value=np.array([1, 2, 3]),
        )

        # Evaluate the comparison
        result = condition.evaluate_comparison(np.array([1, 2, 4]))
        self.assertFalse(result)

if __name__ == "__main__":
    unittest.main()