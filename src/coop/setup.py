from setuptools import setup

package_name = 'coop'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # install our script files where ros2 run expects them
        ('lib/' + package_name, [
            'scripts/lidar_reactive',
            'scripts/coverage_node',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@example.com',
    description='Cooperative / reactive drone nodes',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'coverage_node = coop.coverage_node:main',
            'lidar_reactive = coop.lidar_reactive:main',
        ],
    },
)
