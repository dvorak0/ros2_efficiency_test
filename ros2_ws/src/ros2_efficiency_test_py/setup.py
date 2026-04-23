from glob import glob
from setuptools import setup

package_name = 'ros2_efficiency_test_py'

setup(
    name=package_name,
    version='0.0.0',
    py_modules=[
        'camera_node',
        'control_node',
        'imu_node',
        'perception_node',
        'planning_node',
    ],
    package_dir={'': 'src'},
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yzf',
    maintainer_email='yzf@example.com',
    description='Python ROS 2 efficiency test nodes for camera, IMU, perception, planning, and control pipelines',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = camera_node:main',
            'control_node = control_node:main',
            'imu_node = imu_node:main',
            'perception_node = perception_node:main',
            'planning_node = planning_node:main',
        ],
    },
)
