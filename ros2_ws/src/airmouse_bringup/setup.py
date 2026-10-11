import os
from glob import glob

from setuptools import setup

package_name = 'airmouse_bringup'


def tree(folder):
    """Install every file under `folder`, keeping sub-directories."""
    out = []
    for root, _, files in os.walk(folder):
        if files:
            out.append((os.path.join('share', package_name, root), [os.path.join(root, f) for f in files]))
    return out


setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ] + tree('config'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AirMouse team',
    maintainer_email='team@airmouse.local',
    description='AirMouse launch + config',
    license='TODO',
)
