from setuptools import setup

package_name = 'ucsd_robodog'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	('share/' + package_name + '/launch', ['launch/robodog.launch.py', 'launch/robodog_noVESC.launch.py',]), 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='djnighti@ucsd.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'gesture_detector_node = ucsd_robodog.gesture_detector_node:main',
		'gesture_cmd_node = ucsd_robodog.gesture_cmd_node:main',
		'gun_detection_node = ucsd_robodog.gun_detection_node:main',
		'speaker_node = ucsd_robodog.speaker_node:main',
        ],
    },
)
