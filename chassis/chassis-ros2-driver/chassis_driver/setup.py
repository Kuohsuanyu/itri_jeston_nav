from setuptools import find_packages, setup

package_name = 'chassis_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer="Chih-Pin, Huang",
    maintainer_email="itriB40528@itri.org.tw",
    description='ROS2 driver node for ITRI differential-drive chassis',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'chassis_driver_node = chassis_driver.chassis_driver_node:main',
        ],
    },
)
