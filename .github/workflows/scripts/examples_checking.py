import ipdb
import subprocess
import sys
import typer
from typing import List, Tuple

# Internal Imports
def get_example_tuples() -> List[Tuple[str, str]]:
    return [
        ("examples/conversion/suggested_use1/drakeify.py", "ConversionExample1"),
        ("examples/conversion/converter_advanced_usage1/demo.py", "ConversionExample2-Advanced"),
        ("examples/productions/helpful_for_debugging/demonstrate_static_grasp/suggested_use1/demonstrate.py", "StaticGraspExample1"),
        ("examples/productions/helpful_for_debugging/show_me_this_model/suggested_use1/show.py", "StaticGraspExample1"),
    ]

def main():
    # Setup

    # Create links to all the examples to run
    examples_to_run = get_example_tuples()

    # Run all the examples
    example_ran_successfully = []
    for example_path, example_name in examples_to_run:
        print(f"Running {example_name} ({example_path})...")
        try:
            # Run the example
            script_result = subprocess.run(
                ["python3", example_path],
            )
        except Exception as e:
            print(f"Error running {example_name} ({example_path}): {e}")
            example_ran_successfully.append(False)
            continue
    
        print(f"Finished running {example_name} ({example_path}).")
        # Check the result
        if script_result.returncode != 0:
            print(f"Error running {example_name} ({example_path}).")
            example_ran_successfully.append(False)
            continue

        example_ran_successfully.append(True)

    # Check that all examples ran successfully
    if all(example_ran_successfully):
        print("All examples ran successfully!")
    else:
        print("Some examples failed to run successfully.")
        raise RuntimeError("Some examples failed to run successfully.")

if __name__ == "__main__":
    main()