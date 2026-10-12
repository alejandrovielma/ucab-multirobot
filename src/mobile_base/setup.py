from glob import glob

from setuptools import find_packages, setup

PACKAGE = "mobile_base"

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
    description="Shared behavior, safety, vision and hardware nodes for the collector robots.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "behavior = mobile_base.nodes.behavior_node:main",
            "safety_gate = mobile_base.nodes.safety_gate_node:main",
            "cube_detector = mobile_base.nodes.cube_detector_node:main",
            "hat_motor_driver = mobile_base.nodes.hat_motor_driver_node:main",
            "line_sensor = mobile_base.nodes.line_sensor_node:main",
            "camera = mobile_base.nodes.camera_node:main",
        ]
    },
)
