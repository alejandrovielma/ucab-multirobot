from setuptools import find_packages, setup

PACKAGE = "coordinator"

setup(
    name=PACKAGE,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE}"]),
        (f"share/{PACKAGE}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="UCAB Robotica team",
    maintainer_email="team@example.com",
    description="Delivery-square zone arbiter.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["zone_arbiter = coordinator.zone_arbiter_node:main"]},
)
