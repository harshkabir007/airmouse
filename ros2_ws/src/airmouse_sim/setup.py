from glob import glob

from setuptools import setup

package_name = 'airmouse_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/mazes', glob('mazes/*.txt')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AirMouse team',
    maintainer_email='team@airmouse.local',
    description='AirMouse Gazebo simulation assets',
    license='TODO',
    entry_points={
        'console_scripts': [
            'gen_maze = airmouse_sim.maze_gen:main',
        ],
    },
)
