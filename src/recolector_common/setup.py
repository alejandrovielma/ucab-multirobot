from setuptools import find_packages, setup

package_name = "recolector_common"

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
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="TODO",
    maintainer_email="TODO@ucab.edu.ve",
    description="Helpers shared by the collector packages.",
    license="MIT",
    # colcon runs pytest when it finds it here (`tests_require` is gone from
    # recent setuptools and is silently ignored).
    extras_require={"test": ["pytest"]},
)
