from setuptools import setup
import os
from glob import glob

package_name = 'mapping_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Launch files',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tf_namespace_republisher = mapping_launch.tf_namespace_republisher:main',
            'camera_init_tf_from_raw_lidar = mapping_launch.camera_init_tf_from_raw_lidar:main',
            'cloud_world_aligned_republisher = mapping_launch.cloud_world_aligned_republisher:main',
        ],
    },
)
