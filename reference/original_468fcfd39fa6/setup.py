from glob import glob

from setuptools import find_packages, setup

package_name = 'atarm_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name,                     ['package.xml']),
        ('share/' + package_name + '/launch',         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tianyuan Wang',
    maintainer_email='tianyuan.wang@york.ac.uk',
    description='ROS2 package for controlling the Agricultural Tensegrity Arm (ATArm)',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'end_effector_controller = atarm_controller.end_effector_controller:main',
            'arm_controller = atarm_controller.arm_controller_node:main',
        ],
    },
)
