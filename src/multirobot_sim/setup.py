import os
from glob import glob

from setuptools import find_packages, setup

PACKAGE = "multirobot_sim"


def tree(directory):
    """data_files entries for every file under directory, preserving subfolders
    (the world expects textures/ next to it and ../protos relative to it)."""
    entries = {}
    for path in glob(f"{directory}/**/*", recursive=True):
        if os.path.isfile(path):
            entries.setdefault(os.path.join("share", PACKAGE, os.path.dirname(path)), []).append(path)
    return list(entries.items())


setup(
    name=PACKAGE,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE}"]),
        (f"share/{PACKAGE}", ["package.xml"]),
        (f"share/{PACKAGE}/resource", glob("resource/*.urdf")),
        *tree("worlds"),
        *tree("protos"),
        *tree("controllers"),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="UCAB Robotica team",
    maintainer_email="team@example.com",
    description="Webots simulation of the collector robots.",
    license="MIT",
    tests_require=["pytest"],
)
