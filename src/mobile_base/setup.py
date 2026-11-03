from glob import glob

from setuptools import find_packages, setup

package_name = "mobile_base"

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
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="TODO",
    maintainer_email="TODO@ucab.edu.ve",
    description="Collector mecanum base: motors, sensors, scoop and behaviour.",
    license="MIT",
    # colcon runs pytest when it finds it here (`tests_require` is gone from
    # recent setuptools and is silently ignored).
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "motor_driver = mobile_base.motor_driver:main",
            "line_sensors = mobile_base.line_sensors:main",
            "ultrasonic = mobile_base.ultrasonic:main",
            "obstacle_guard = mobile_base.obstacle_guard:main",
            "scoop_driver = mobile_base.scoop_driver:main",
            "color_sensor = mobile_base.color_sensor:main",
            "behavior_fsm = mobile_base.behavior_fsm:main",
            "fleet_link = mobile_base.fleet_link:main",
        ],
    },
)
