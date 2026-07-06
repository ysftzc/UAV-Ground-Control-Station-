from setuptools import find_packages, setup

package_name = 'uav_gcs_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yusuf Tuzcu',
    maintainer_email='ysftzc8@gmail.com',
    description='ROS2 nodes consuming PX4 uXRCE-DDS topics fed by real STM32 sensor data',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'attitude_listener = uav_gcs_bridge.attitude_listener:main',
        ],
    },
)
