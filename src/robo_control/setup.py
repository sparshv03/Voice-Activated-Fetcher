from glob import glob

from setuptools import find_packages, setup

package_name = 'robo_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rtk',
    maintainer_email='rtk@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'arm_control = robo_control.arm_control:main',
            'ultrasonic_arm_mapper = robo_control.arm_control:main',
            'chassis_control = robo_control.chassis_control:main',
            'step_bridge_arm = robo_control.step_bridge_arm:main',
        ],
    },
)
