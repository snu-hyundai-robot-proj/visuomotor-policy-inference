from setuptools import find_packages, setup

package_name = 'teleop_vision'

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
    maintainer='aiis',
    maintainer_email='hochan0115@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	# 'vision_recorder = teleop_vision.vision_recorder:main',
        	'hook_pose_estimator = teleop_vision.hook_pose_estimator:main',
        	'system_vision_right = teleop_vision.vision_node_right:main',
        	'system_vision_left = teleop_vision.vision_node_left:main',
        	'handeye_calibrate = teleop_vision.fusion.handeye_calibrate:main',
        	'fusion_node = teleop_vision.fusion.fusion_node:main',
        	# 'system_vision_rf = teleop_vision.vision_node_rf:main',
        ],
    },
)
