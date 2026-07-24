from setuptools import find_packages, setup

package_name = "racer_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="Kong-huihui",
    maintainer_email="kong-huihui@users.noreply.github.com",
    description="rclpy integration nodes for RACER.",
    license="LicenseRef-RACER-Upstream",
    entry_points={
        "console_scripts": [
            "exploration_node = racer_ros.exploration_node:main",
            "trajectory_server = racer_ros.trajectory_server:main",
            "isaac_command_adapter = racer_ros.isaac_command_adapter:main",
            "lkh_service = racer_ros.lkh_service:main",
        ],
    },
)
