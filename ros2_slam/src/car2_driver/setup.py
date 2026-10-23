from setuptools import find_packages, setup

package_name = 'car2_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='perpet',
    maintainer_email='perpet99@gmail.com',
    description=(
        'car2 (4-wheel ST3215 skid-steer) ROS 2 serial driver: '
        'cmd_vel -> serial (or car2_web REST API), feedback -> odometry/TF.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'car2_serial_node = car2_driver.car2_serial_node:main',
            'ld_lidar_node = car2_driver.ld_lidar_node:main',
        ],
    },
)
