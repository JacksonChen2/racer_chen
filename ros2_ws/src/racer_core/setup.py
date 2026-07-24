from setuptools import find_packages, setup


package_name = "racer_core"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy", "scipy"],
    zip_safe=True,
    maintainer="Kong-huihui",
    maintainer_email="kong-huihui@users.noreply.github.com",
    description="ROS-independent Python implementation of RACER algorithms.",
    license="LicenseRef-RACER-Upstream",
)
