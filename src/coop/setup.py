from setuptools import setup
import os
from glob import glob

package_name = 'coop'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],  # this expects src/coop/coop/__init__.py
    data_files=[
        # so ROS 2 can discover the package
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        # install package.xml
        ('share/' + package_name, ['package.xml']),
        # install launch files if you have any
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kevin Dang',
    maintainer_email='kevin@example.com',
    description='Cooperative multi-drone coordination for FIREWARDENBOT.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            # this is what should create install/coop/lib/coop/coverage_manager
            'coverage_manager = coop.coverage_manager:main',
        ],
    },
)
