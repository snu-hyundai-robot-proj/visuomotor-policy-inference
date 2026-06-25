from glob import glob

from setuptools import find_packages, setup

package_name = "lerobot_system"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    # Runtime deps (lerobot, torch, ruckig, numpy) are pip-installed in the node's
    # Python env — not declared here, so `colcon build` never tries to fetch them.
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="aiis",
    maintainer_email="todo@todo.todo",
    description="LeRobot inference bridge for left/right robot systems.",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "lerobot_system = lerobot_system.node:main",
            "lerobot_system_left = lerobot_system.node:left_main",
            "lerobot_system_right = lerobot_system.node:right_main",
        ],
    },
)
