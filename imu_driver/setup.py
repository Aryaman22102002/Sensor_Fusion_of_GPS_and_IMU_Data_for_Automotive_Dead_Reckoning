from setuptools import find_packages, setup
import os 
from glob import glob

package_name = 'imu_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools','pyserial', 'rclpy'],
    zip_safe=True,
    maintainer='aryaman',
    maintainer_email='lnu.arya@northeastern.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
         'driver = imu_driver.driver:main'
        ],
    },
)
