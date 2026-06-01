from setuptools import setup
from glob import glob

package_name = "episode_manager"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "numpy", "requests"],
    zip_safe=True,
    maintainer="bi",
    maintainer_email="juniormoo87@gmail.com",
    description="Episode lifecycle orchestrator for the visuomotor policy stack.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "episode_manager = episode_manager.episode_manager_node:main",
        ],
    },
)
