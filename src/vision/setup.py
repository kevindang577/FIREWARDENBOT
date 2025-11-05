from setuptools import setup
import os
from glob import glob

package_name = 'vision'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@example.com',
    description='Computer vision package for Fire Warden Bot leaf detection and classification',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'leaf_detector = vision.leaf_detector:main',
            'leaf_monitor = vision.leaf_monitor:main',
        ],
    },
)
