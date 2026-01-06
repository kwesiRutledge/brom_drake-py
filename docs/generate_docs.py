from importlib import resources as impresources
from importlib import import_module
import importlib.util
import inspect
import os
from pathlib import Path
import pkgutil
import shutil
import subprocess
import ipdb
import typer
from typing import List, Tuple

import brom_drake

def generate_all_discoverable_docs(
    output_dir: Path,
    package_dir: Path = Path("../src/brom_drake"),
):
    """
    *Description*

    Iterates through all submodules in the given package directory,
    generating documentation files for each submodule, and finally
    generating an index file for the entire package.
    
    :param output_dir: Place to create the doc file for Sphinx.
    :type output_dir: Path
    :param package_dir: Location of the code we are seeking to document.
    :type package_dir: Path
    """

    # Announce beginning of function
    print(f"- Generating docs for package in directory: {package_dir}")

    # Identify the package name
    package_name = package_dir.name

    # Identify the directories and files within this package's directory
    files_found = []
    directories_found = []
    for child in package_dir.iterdir():
        if child.is_file():
            # Only consider .py files
            if child.suffix == ".py":
                files_found.append(child)
            else:
                print(f"- Ignoring non-.py file: {child.name}")

        elif child.is_dir():
            # Ignore __pycache__ directories
            if child.name == "__pycache__":
                print(f"- Ignoring __pycache__ directory ({child})")
                continue

            directories_found.append(child)
    
    #TODO(Kwesi): Remove this later
    submodule_names = [elt.name for elt in directories_found]

    # Iterate through all files in the package directory
    # and retrieve all classes AND functions defined within each
    available_functions = []
    available_classes = []
    available_variables = []
    for file_path_i in files_found:
        print(f"- Found file: {file_path_i.name}")

        if file_path_i.name == "__init__.py":
            # Ignore the __init__.py file
            print("  + Ignoring __init__.py file")
            continue

        # Identify available functions in this file
        temp_functions, temp_variables, temp_classes = identify_available_functions_and_variables_in_file(
            target_file_path=file_path_i
        )

        available_functions.extend(temp_functions)
        available_variables.extend(temp_variables)
        available_classes.extend(temp_classes)

    # Make all lists unique
    available_functions = list(set(available_functions))
    available_variables = list(set(available_variables))
    available_classes = list(set(available_classes))

    # Identify all functions in this package
    package_dir_contents = os.listdir(package_dir)
    files_in_package_dir = [
        elt
        for elt in package_dir_contents
        if ".py" in elt
    ]

    for mod in submodule_names:
        print(f"Generating docs for module: {mod}")
        # typer.run(
        #     [
        #         "sphinx-apidoc",
        #         "-o",
        #         str(output_dir),
        #         "--force",
        #         "--module-first",
        #         "--separate",
        #         mod,
        #     ]
        # )

        # Generate the docs for this module
        generate_all_discoverable_docs(
            output_dir=output_dir / mod,
            package_dir=package_dir / mod,
        )

    # TODO(Kwesi): Implement the actual doc generation logic
    write_rst_file_for_package(
        package_name=package_name,
        output_dir=output_dir,
        submodules=submodule_names,
        functions=available_functions,
        classes=available_classes,
        variables=available_variables,
    )

def identify_available_functions_and_variables_in_file(
    target_file_path: Path,
) -> Tuple[List[str], List[str], List[str]]:
    """
    *Description*

    This method returns all of the:
    1. functions,
    2. variables, and
    3. classes
    defined within the given file.

    *Returns*

    available_functions: List[str]
        A list of all functions defined within the given file.

    available_variables: List[str]
        A list of all variables defined within the given file.

    available_classes: List[str]
        A list of all classes defined within the given file.
    """
    # Announce the beginning of this function
    print(f"Determining the functions in the following file: {target_file_path}")


    canonicalized_file_name = package_path_to_module_name(target_file_path)
    print(f"- Canonicalized file: {canonicalized_file_name}")

    # Use importlib to import all contents of this file path
    spec = importlib.util.spec_from_file_location(
        canonicalized_file_name,
        target_file_path
    ) # 1. Create a spec from the file path
    module = importlib.util.module_from_spec(spec) # 2. Create a new module based on the spec
    spec.loader.exec_module(module) # 3. Execute the module (this populates it with functions/classes)
    pkg_contents = dir(module)

    # Collect all contents of package
    available_functions = []
    available_classes = []
    available_variables = []
    for name, obj in inspect.getmembers(module):
        # Do not add to the list any functions that are built-in
        # (i.e., those that start with double underscores)
        if name[:2] == "__":
            continue

        # If all other checks pass, then add the fcn to the available_functions list
        if inspect.isfunction(obj):

            if obj.__module__ != module.__name__:
                # Ignore functions that are imported from other modules
                print(f"  + Ignoring imported function: {name}")
                continue

            available_functions.append(target_file_path.stem + "." + name)
        elif inspect.isclass(obj):

            if obj.__module__ != module.__name__:
                # Ignore classes that are imported from other modules
                print(f"  + Ignoring imported class: {name}")
                continue

            available_classes.append(target_file_path.stem + "." + name)
        elif inspect.ismodule(obj):
            # Ignore modules
            print(f"  + Ignoring module: {name}")
            continue
        else:
            available_variables.append(target_file_path.stem + "." + name)

    print("- Available functions:")
    for candidate in available_functions:
        print(f"  + {candidate}")

    print("- Available classes:")
    for candidate in available_classes:
        print(f"  + {candidate}")

    print("- Available variables:")
    for candidate in available_variables:
        print(f"  + {candidate}")
    
    return available_functions, available_variables, available_classes

def get_canonicalized_function_name(
    output_dir: Path,
    function_name: str,
) -> str:
    """
    Docstring for get_canonicalized_function_name
    
    :param output_dir: Description
    :type output_dir: Path
    :param function_name: Description
    :type function_name: str
    :return: Description
    :rtype: str
    """
    # Create output string container
    out = "brom_drake"

    # Extract the component of the output_dir that should
    # mirror the path in the canonical import
    # "source/generated/control/arms" -> "control/arms"
    expected_prefix = "source/generated"
    if expected_prefix not in str(output_dir):
        raise ValueError(f"`output_dir` does not contain the expected prefix: {expected_prefix}")

    # Add the component that should be relevant to the expected output
    # e.g., adds "control.arms" to out
    out += \
        str(output_dir)[str(output_dir).find(expected_prefix)+len(expected_prefix):]
    out = out.replace("/",".")

    assert ".." not in out, f"Invalid canonicalized name generated: {out}"

    if out[-1] != ".":
        out += "."

    return out + function_name

def package_path_to_module_name(package_dir: Path) -> str:
    """
    *Description*

    Converts a package directory path to its canonicalized
    module name (i.e., the name used in imports).

    :param package_dir: The package directory path.
    :type package_dir: Path
    :return: The canonicalized module name.
    :rtype: str
    """
    # Convert the package directory to a string
    trimmed_package_dir = package_dir.parent / package_dir.stem # Remove any trailing suffixes like ".py"
    package_dir_str = str(trimmed_package_dir)

    extended_package_name = package_dir_str[
        package_dir_str.find("brom_drake-py/src/brom_drake") + len("brom_drake-py/src/"):
    ]

    # Replace slashes with dots to form the module name
    module_name = extended_package_name.replace("/", ".")

    return module_name

def write_rst_file_for_package(
    package_name: str,
    output_dir: Path,
    submodules: list[str],
    functions: list[str],
    classes: list[str],
    variables: list[str],
):
    """
    Writes an .rst file that is compatible with
    Sphinx's autodoc and autosummary features, containing:
    - hyperlinks to all submodules in `submodules`, and
    - a summary table of all classes and functions in `module_name`.

    :param module_name: Description
    :type module_name: str
    :param output_dir: Description
    :type output_dir: Path
    :param submodules: Description
    :type submodules: list[str]
    """
    # Create the output directory if it doesn't exist
    if not output_dir.exists():
        output_dir.mkdir(parents=True, exist_ok=True)

    # Create the .rst file
    rst_file_path = output_dir / f"{package_name}.rst"
    
    # Fix the name of the .rst file 
    # For the overall package (this is important so that the index always points to a clear place)
    if package_name == "brom_drake": 
        rst_file_path = output_dir / f"api.rst"
        
    with open(rst_file_path, "w") as rst_file:
        # Write the module title
        title = package_name
        if package_name == "brom_drake":
            # The title for the "overall" page should be API, not brom_drake
            title = "API"

        rst_file.write(f"{title}\n")
        rst_file.write("=" * len(title) + "\n\n")

        # Write the list of submodules
        rst_file.write("Submodules\n")
        rst_file.write("----------\n\n")

        # Write hyperlinks to submodules
        rst_file.write(".. toctree::\n")
        rst_file.write("   :maxdepth: 1\n\n")
        for submodule in sorted(submodules, key=str.lower):
            rst_file.write(f"   {submodule}/{submodule}\n")
        rst_file.write("\n")

        # Write the list of classes
        rst_file.write("Classes\n")
        rst_file.write("-------\n\n")

        if len(classes) == 0:
            rst_file.write("(None found)\n\n")
        else:
            for cls in sorted(classes, key=str.lower):
                canonical_cls_name = get_canonicalized_function_name(output_dir, function_name=cls)
                rst_file.write(f".. autoclass:: {canonical_cls_name}\n")
                rst_file.write(f"   :members:\n")
            rst_file.write("\n")

        # Write the list of functions
        rst_file.write("Functions\n")
        rst_file.write("---------\n\n")

        if len(functions) == 0:
            rst_file.write("(None found)\n\n")
        else:
            for fcn in sorted(functions, key=str.lower):
                canonical_fcn_name = get_canonicalized_function_name(output_dir, function_name=fcn)
                rst_file.write(f".. autofunction:: {canonical_fcn_name}\n")
            rst_file.write("\n")

        # Write the list of variables
        rst_file.write("Variables\n")
        rst_file.write("---------\n\n")

        if len(variables) == 0:
            rst_file.write("(None found)\n\n")
        else:
            for var in sorted(variables, key=str.lower):
                canonical_var_name = get_canonicalized_function_name(output_dir, function_name=var)
                rst_file.write(f".. autodata:: {canonical_var_name}\n")
            rst_file.write("\n")

        # # Write autosummary table
        # rst_file.write(".. autosummary::\n")
        # rst_file.write("   :toctree: generated\n\n")
        # rst_file.write(f"   {package_name}.*\n")

    print(f"Wrote RST file for module {package_name} at:")
    print(f"- {rst_file_path}")

def main(
    output_dir: Path = Path("source"),
    clean_up_afterward: bool = False,
):
    # Create directory for storing all of the "autogenerated"
    # docs
    generated_file_dir = output_dir / "generated"
    if not generated_file_dir.exists():
        output_dir.mkdir(parents=True, exist_ok=True)

    # Create all docs using a recursive function
    generate_all_discoverable_docs(
        output_dir=generated_file_dir,
        package_dir=impresources.files(brom_drake),
    )

    # Build the docs using sphinx-build
    subprocess.run(
        [
            "sphinx-build",
            "-M",
            "html",
            "source", # Source directory
            "build",  # Build directory

        ]
    )

    print("Completed generating the documentation using sphinx.\n")

    # Clean Up "temporary directory"
    if clean_up_afterward:
        shutil.rmtree(generated_file_dir)
        print("- Removed temporary directory")

    # subprocess.run(
    #     [
    #         "sphinx-build",
    #         "-M",
    #         "html",
    #         str(output_dir),
    #         "../build/html",
    #     ],
    #     check=True,
    # )

if __name__ == "__main__":
    with ipdb.launch_ipdb_on_exception():
        typer.run(main)