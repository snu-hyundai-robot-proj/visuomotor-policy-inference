from setuptools import find_packages, setup

package_name = 'inspire_driver'

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
    maintainer='pin',
    maintainer_email='pin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [ 
            'inspire_driver_node = inspire_driver.inspire_driver:main',
            'inspire_bridge_node = inspire_driver.inspire_bridge:main',
        ],
    },
)
