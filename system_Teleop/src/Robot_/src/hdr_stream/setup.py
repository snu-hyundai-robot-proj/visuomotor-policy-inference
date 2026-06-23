from setuptools import setup, find_packages

package_name = 'hdr_stream'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=[]),
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
    extras_require={
    },
    entry_points={
        'console_scripts': [
            "hdr_stream_node = hdr_stream.hdr_stream:main"
        ],
    },
)
