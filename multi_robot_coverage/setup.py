import os
from glob import glob
from setuptools import find_packages, setup

package_name = "multi_robot_coverage"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "maps"), glob("maps/*.yaml")),
        (os.path.join("share", package_name, "maps"), glob("maps/*.pgm")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Shady Siam",
    maintainer_email="shadysiam42@gmail.com",
    description=(
        "Multi-robot coverage path planning with Boustrophedon Cellular Decomposition"
        " and frontier-based exploration."
    ),
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "map_server = multi_robot_coverage.nodes.map_server:main",
            "robot_agent = multi_robot_coverage.nodes.robot_agent:main",
            "coverage_coordinator = multi_robot_coverage.nodes.coverage_coordinator:main",
            "visualizer = multi_robot_coverage.nodes.visualizer:main",
            "generate_maps = multi_robot_coverage.map_generator:main",
        ],
    },
)
