from setuptools import find_packages, setup

package_name = 'snu_hdr_task_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dyros',
    maintainer_email='dyros@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'joint_move = snu_hdr_task_manager.joint_move_control:main',
            'admittance = snu_hdr_task_manager.admittance_control:main',

        ],
    },
)
