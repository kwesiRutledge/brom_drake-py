import typer

def main(version: str):
    """
    *Description*
    
    This script updates the version number in the configuration files.

    *Parameters*

    version: str
        The new version number to set in the configuration files.
    """

    # List of configuration files to update
    config_files = [
        "source/conf.py",
    ]

    for config_file in config_files:
        with open(config_file, "r") as file:
            content = file.read()

        # Replace the line beginning with 'release =' with the new version
        lines = content.split("\n")
        for i, line in enumerate(lines):
            if line.strip().startswith("release ="):
                lines[i] = f"release = '{version}'"
        content = "\n".join(lines)

        # Write the updated content back to the file
        with open(config_file, "w") as file:
            file.write(content)

    print(f"Updated version to \"{version}\" in configuration files.")

if __name__ == "__main__":
    typer.run(main)