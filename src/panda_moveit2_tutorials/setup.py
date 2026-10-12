from setuptools import setup, find_packages

package_name = 'panda_moveit2_tutorials'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/panda_demo.launch.py']),
        ('share/' + package_name + '/config', ['config/ompl_planning.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='JosiahSK',
    maintainer_email='your.email@example.com',
    description='MoveIt 2 Panda tutorial Python package (ROS 2 Humble)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'basic_motion_planner = panda_moveit2_tutorials.basic_motion_planner:main',
            'pose_goal_planner = panda_moveit2_tutorials.pose_goal_planner:main',
            'pick_and_place = panda_moveit2_tutorials.pick_and_place:main',
        ],
    },
)
