from glob import glob
from setuptools import find_packages, setup

package_name = "racer_isaac"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/isaac", glob("isaac/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Kong-huihui",
    maintainer_email="kong-huihui@users.noreply.github.com",
    description="Isaac Sim ROS 2 Bridge graph builder for RACER.",
    license="LicenseRef-RACER-Upstream",
)
