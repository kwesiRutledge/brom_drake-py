import ipdb
import subprocess
import sys
import typer
from typing import List, Tuple

# Internal Imports
def get_example_tuples() -> List[Tuple[str, str]]:
    # Setup
    debugging_production_dir = "examples/productions/helpful_for_debugging/"
    show_me_production_dir = debugging_production_dir + "show_me_this_model/"
    offline_mp_examples_dir = 'examples/productions/motion_planning/offline/'

    # Create a list of tuples containing the path to the example and the name of the example
    return [
        ("examples/conversion/suggested_use1/drakeify.py", "ConversionExample1"),
        ("examples/conversion/converter_advanced_usage1/demo.py", "ConversionExample2-Advanced"),
        ("examples/watcher/suggested_use1/create_monitor.py", "WatcherExample1"),
        ("examples/watcher/wo_convenience1/create_monitor_wo_convenience.py", "WatcherExample2"),
        ("examples/watcher/changing_plot_arrangement1/create_monitor_w_different_arrangement.py", "WatcherExample3"),
        (debugging_production_dir + "demonstrate_static_grasp/suggested_use1/demonstrate.py", "StaticGraspExample1"),
        (show_me_production_dir + "suggested_use1/show.py", "ShowMeThisModelExample1"),
        (show_me_production_dir + "suggested_use2/show.py", "ShowMeThisModelExample2"),
        (show_me_production_dir + "show_collision_geometries1/show.py", "ShowMeThisModelExample3-ShowCollisionGeometries"),
        (offline_mp_examples_dir + "chem_lab1/example.py", "OfflineMotionPlanningExample1 - ChemLab1"),
        (offline_mp_examples_dir + "shelf/shelf1.py", "OfflineMotionPlanningExample2 - Shelf1"),
        (offline_mp_examples_dir + "chem_lab2/suggested_use1/example.py", "OfflineMotionPlanningExample3 - ChemLab2"),
        (offline_mp_examples_dir + "chem_lab2/providing_start_as_config1/provide_start_config1.py", "OfflineMotionPlanningExample4 - ChemLab2 w/ Start Config"),
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