from setuptools import find_packages, setup
import codecs
import os

if __name__ == "__main__":
    here = os.path.abspath(os.path.dirname(__file__))

    with codecs.open(os.path.join(here, "README.md"), encoding="utf-8") as f:
        long_description = "\n" + f.read()

    setup(
        name="brom_drake",
        version="{{VERSION_PLACEHOLDER}}",
        python_requires=">=3.14",
        author="Kwesi Rutledge",
        author_email="thesolitaryecrivain@gmail.com",
        url="https://github.com/kwesiRutledge/brom_drake-py",
        description="A set of convenient logging and testing tools for the Drake robotics toolbox.",
        long_description_content_type="text/markdown",
        long_description=long_description,
        # packages=find_packages(where='src/brom_drake'),
        include_package_data=True,
        install_requires=[
            "drake>=1.54.0",
            "meshcat",
            "matplotlib",
            "networkx",
            "numpy",
            "pycollada",
            "scipy",
            "trimesh",
            "coacd",
        ],
        extras_require={
            "test": [
                "pytest",
                "pytest-cov",
            ],
            "dev": ["ipdb", "typer", "black"],
            "ros2": ["ament-index-python"],
        },
        keywords=["drake", "robotics", "testing", "logging"],
        classifiers=[
            "Programming Language :: Python :: 3",
        ],
    )
