from setuptools import find_packages, setup

package_name = 'drone_control'

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
    maintainer='an',
    maintainer_email='an@todo.todo',
    description='Drone control with smooth S-curve servo trajectories',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'circle_mission = drone_control.circle_mission:main',
            'takeoff_land_mission = drone_control.takeoff_land_mission:main',
            'arm_mission = drone_control.arm_mission:main',
            'path_logger = drone_control.path_logger:main',
            'pid_ann_tuner = drone_control.pid_ann_tuner:main',
        ],
    },
)
