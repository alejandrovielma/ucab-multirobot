from glob import glob

from setuptools import find_packages, setup

package_name = "vision"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/web", glob("web/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="TODO",
    maintainer_email="TODO@ucab.edu.ve",
    description="Collector camera and colour-based cube detection.",
    license="MIT",
    # colcon runs pytest when it finds it here (`tests_require` is gone from
    # recent setuptools and is silently ignored).
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "camera_node = vision.camera_node:main",
            "cube_detector = vision.cube_detector:main",
            "web_viewer = vision.web_viewer:main",
        ],
    },
)
