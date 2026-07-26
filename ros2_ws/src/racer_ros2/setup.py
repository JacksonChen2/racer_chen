from glob import glob
from setuptools import find_packages, setup


package_name = "racer_ros2"

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
        ("share/" + package_name + "/test_results", glob("test_results/*.md")),
    ],
    install_requires=["setuptools", "numpy"],
    extras_require={"test": ["pytest"]},
    zip_safe=False,
    maintainer="RACER ROS 2 Maintainer",
    maintainer_email="maintainer@example.com",
    description="RACER decentralized multi-UAV exploration for ROS 2 and Isaac Sim",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "racer_agent = racer_ros2.agent_node:main",
            "racer_mock_sim = racer_ros2.mock_simulator:main",
            "racer_monitor = racer_ros2.monitor_node:main",
        ],
    },
)
