from glob import glob

from setuptools import setup

package_name = "bringup"

setup(
    name=package_name,
    version="0.0.0",
    packages=[],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="TODO",
    maintainer_email="TODO@ucab.edu.ve",
    description="Collector launch files",
    license="MIT",
    # colcon runs pytest when it finds it here (`tests_require` is gone from
    # recent setuptools and is silently ignored).
    extras_require={"test": ["pytest"]},
)
