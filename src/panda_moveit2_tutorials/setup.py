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
            'cartesian_path_planner = panda_moveit2_tutorials.cartesian_path_planner:main',
            'collision_avoidance_demo = panda_moveit2_tutorials.collision_avoidance_demo:main',
            'constrained_motion_demo = panda_moveit2_tutorials.constrained_motion_demo:main',
            'pick_and_place = panda_moveit2_tutorials.pick_and_place:main',
            'planner_benchmark = panda_moveit2_tutorials.planner_benchmark:main',
        ],
    },
)
