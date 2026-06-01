from setuptools import setup
from glob import glob

package_name = "vpi_robot_client"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "requests", "pillow", "numpy"],
    zip_safe=True,
    maintainer="bi",
    maintainer_email="juniormoo87@gmail.com",
    description="ROS2 client that drives HDR35 + DG5F from the visuomotor policy HTTP server.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "policy_control = vpi_robot_client.policy_control_node:main",
        ],
    },
)
