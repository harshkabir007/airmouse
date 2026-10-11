from glob import glob

from setuptools import setup

package_name = 'airmouse'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/web', glob('web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AirMouse team',
    maintainer_email='team@airmouse.local',
    description='NIDAR AirMouse autonomy stack',
    license='TODO',
    entry_points={
        'console_scripts': [
            'pose_bridge = airmouse.pose_bridge_node:main',
            'mapper = airmouse.mapper_node:main',
            'commander = airmouse.commander_node:main',
            'survivor_detector = airmouse.survivor_node:main',
            'gcs_server = airmouse.gcs_server_node:main',
        ],
    },
)
