from glob import glob
from setuptools import find_packages, setup


package_name = "racer_3d"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/isaac_sim", glob("isaac_sim/*.py")),
        ("share/" + package_name + "/scripts", glob("scripts/*.sh")),
        ("share/" + package_name + "/test_results", glob("test_results/*")),
    ],
    install_requires=["setuptools", "numpy"],
    extras_require={"test": ["pytest"]},
    zip_safe=False,
    maintainer="RACER 3D Maintainer",
    maintainer_email="maintainer@example.com",
    description="Three-dimensional RACER for ROS 2 Humble and Isaac Sim",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "racer_3d_agent = racer_3d.agent_node:main",
            "racer_3d_monitor = racer_3d.monitor_node:main",
            "racer_3d_mock_sim = racer_3d.mock_simulator:main",
        ],
    },
)
