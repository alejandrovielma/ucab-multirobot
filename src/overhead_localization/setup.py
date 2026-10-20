from glob import glob

from setuptools import find_packages, setup

PACKAGE = "overhead_localization"

setup(
    name=PACKAGE,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE}"]),
        (f"share/{PACKAGE}", ["package.xml"]),
        (f"share/{PACKAGE}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="UCAB Robotica team",
    maintainer_email="team@example.com",
    description="Overhead ArUco localizer for the collector robots.",
    license="MIT",
    extras_require={"test": ["pytest"]},
    entry_points={"console_scripts": ["aruco_localizer = overhead_localization.aruco_localizer_node:main"]},
)
