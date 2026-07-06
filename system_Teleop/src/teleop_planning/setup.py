import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'teleop_planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ngseo',
    maintainer_email='juniormoo87@gmail.com',
    description='Standalone perception-to-motion-planning for HDR35 via MoveIt.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'plan_to_hook = teleop_planning.plan_to_hook:main',
            'joint_state_relay = teleop_planning.joint_state_relay:main',
            'hdr_followjoint_bridge = teleop_planning.hdr_followjoint_bridge:main',
        ],
    },
)
