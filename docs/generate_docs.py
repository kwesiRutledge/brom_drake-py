from importlib import resources as impresources
from importlib import import_module
import inspect
import os
from pathlib import Path
import pkgutil
import shutil
import subprocess
import typer
from typing import List

import brom_drake

def generate_all_discoverable_docs(
    output_dir: Path,
    package_dir: Path = Path("../src/brom_drake"),
):
    """
    Docstring for generate_all_discoverable_docs
    
    :param output_dir: Place to create the doc file for Sphinx.
    :type output_dir: Path
    :param package_dir: Location of the code we are seeking to document.
    :type package_dir: Path
    """

    # Announce beginning of function
    print(f"Generating docs for package in directory: {package_dir}")

    package_name = package_dir.name

    extended_package_name = str(package_dir)
    extended_package_name = extended_package_name[
        extended_package_name.find("brom_drake-py/src/brom_drake") + len("brom_drake-py/src/"):
    ]
    extended_package_name = extended_package_name.replace("/", ".")

    # Identify all submodules in this package
    canonicalized_modules, submodule_names = [], []
    for _, name, is_pkg in pkgutil.walk_packages([package_dir]):
        # Only do the recursive call if it's a package
        if not is_pkg:
            continue

        # Create the module name as it would appear in imports (i.e., "parent.child.grandchild")
        full_module_path = package_dir / name
        full_module_name = str(full_module_path.relative_to(package_dir.parent)).replace("/", ".")
        canonicalized_modules.append(full_module_name)

        # Also save the simple module name
        submodule_names.append(name)

        print(f"Found submodule: {full_module_name}")
    
    # Identify all functions in this package
    package_dir_contents = os.listdir(package_dir)
    files_in_package_dir = [
        elt
        for elt in package_dir_contents
        if ".py" in elt
    ]

    available_functions = identify_available_functions_in_package(
        extended_package_name,
        subpackage_names=submodule_names,
        python_file_names=files_in_package_dir
    )
    available_classes = identify_available_classes_in_package(extended_package_name, subpackage_names=submodule_names)

    # Organize modules alphabetically
    canonicalized_modules = sorted(canonicalized_modules, key=str.lower)

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
        classes=available_classes
    )

def identify_available_classes_in_package(
    canonicalized_package_name: str,
    subpackage_names: List[str],
) -> List[str]:
    # Announce the beginning of this function
    print("Determining the functions (i.e., the things which are not subpackages) in:")
    print(f"- Canonicalized package: {canonicalized_package_name}")
    print(f"- Subpackages to avoid adding:")
    for subpkg in subpackage_names:
        print(f"  + {subpkg}")

    # Collect all contents of package
    pkg_contents = dir(import_module(canonicalized_package_name))
    available_classes = []
    for candidate in pkg_contents:
        # Do not add to the list any functions that are built-in
        # (i.e., those that start with double underscores)
        if candidate[:2] == "__":
            continue

        # Do not add subpackages to the list of available functions
        if candidate in subpackage_names:
            continue

        # Do not add files to the list of available functions
        # candidate_as_python_file = candidate + ".py"
        # if candidate_as_python_file in python_file_names:
        #     continue

        # Do not add to the list something that is detected as not a "class"
        candidate_obj = eval(f"{canonicalized_package_name}.{candidate}")
        if not inspect.isclass(candidate_obj):
            continue

        # If all other checks pass, then add the fcn to the available_functions list
        available_classes.append(candidate)

    print("- Available functions:")
    for candidate in available_classes:
        print(f"  + {candidate}")
    
    return available_classes

def identify_available_functions_in_package(
    canonicalized_package_name: str,
    subpackage_names: List[str],
    python_file_names: List[str]
) -> List[str]:
    # Announce the beginning of this function
    print("Determining the functions (i.e., the things which are not subpackages) in:")
    print(f"- Canonicalized package: {canonicalized_package_name}")
    print(f"- Subpackages to avoid adding:")
    for subpkg in subpackage_names:
        print(f"  + {subpkg}")

    # Collect all contents of package
    pkg_contents = dir(import_module(canonicalized_package_name))
    available_functions = []
    for candidate in pkg_contents:
        # Do not add to the list any functions that are built-in
        # (i.e., those that start with double underscores)
        if candidate[:2] == "__":
            continue

        # Do not add subpackages to the list of available functions
        if candidate in subpackage_names:
            continue

        # Do not add files to the list of available functions
        # candidate_as_python_file = candidate + ".py"
        # if candidate_as_python_file in python_file_names:
        #     continue

        # Do not add to the list something that is detected as a "module"
        candidate_obj = eval(f"{canonicalized_package_name}.{candidate}")
        if not inspect.isfunction(candidate_obj):
            continue

        # If all other checks pass, then add the fcn to the available_functions list
        available_functions.append(candidate)

    print("- Available functions:")
    for candidate in available_functions:
        print(f"  + {candidate}")
    
    return available_functions

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
    out = "brom_drake."

    # Extract the component of the output_dir that should
    # mirror the path in the canonical import
    # "source/generated/control/arms" -> "control/arms"
    expected_prefix = "source/generated/"
    if expected_prefix not in str(output_dir):
        raise ValueError(f"`output_dir` does not contain the expected prefix: {expected_prefix}")

    # Add the component that should be relevant to the expected output
    # e.g., adds "control.arms" to out
    out += \
        str(output_dir)[str(output_dir).find(expected_prefix)+len(expected_prefix):]
    out = out.replace("/",".")

    # Make sure to add the trailing period and then the function name
    return out + "." + function_name

def write_rst_file_for_package(
    package_name: str,
    output_dir: Path,
    submodules: list[str],
    functions: list[str],
    classes: list[str],
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

        # for submodule in sorted(submodules, key=str.lower):
        #     rst_file.write(f"- `{submodule} <{submodule}/{submodule}.html>`_\n")
        # rst_file.write("\n")

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

        # # Write autosummary table
        # rst_file.write(".. autosummary::\n")
        # rst_file.write("   :toctree: generated\n\n")
        # rst_file.write(f"   {package_name}.*\n")

    print(f"Wrote RST file for module {package_name} at:")
    print(f"- {rst_file_path}")

def main(output_dir: Path = Path("source"), clean_up_afterward: bool = False):
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
            "make",
            "html",
        ]
    )

    print("Completed generating the documentation using sphinx.")

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
    typer.run(main)