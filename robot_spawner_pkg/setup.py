import os 
from glob import glob
from setuptools import setup

package_name = 'robot_spawner_pkg'
cur_directory_path = os.path.abspath(os.path.dirname(__file__))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name,'worlds/'), glob('./worlds/*')),
        (os.path.join('share', package_name,'models/Maze_ql_1/'), glob('./models/Maze_ql_1/*')),
        (os.path.join('share', package_name,'models/basic_robot'), glob('./models/basic_robot/*')),
        (os.path.join('share', package_name,'models/globe'), glob('./models/globe/*'))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spawn_demo = robot_spawner_pkg.spawn_demo:main',
            'spawn_scenario = robot_spawner_pkg.spawn_scenario:main'

        ],
    },
)
