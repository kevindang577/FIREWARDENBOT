from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'coop'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name]),
    data_files=[
        # make ROS 2 aware of this package
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        # install package.xml
        ('share/' + package_name, ['package.xml']),
        # install launch files
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kevin Dang',
    maintainer_email='kevin@example.com',
    description='Cooperative multi-drone coordination for FIREWARDENBOT.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # this is our actual node
            'coverage_manager = coop.coverage_manager:main',
        ],
    },
)
