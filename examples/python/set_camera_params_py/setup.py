from setuptools import setup

package_name = 'set_camera_params_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nodar',
    maintainer_email='support@nodarsensor.com',
    description='This example demonstrates how to control the camera gain and exposure in realtime by using the ROS2 interface for hammerhead',
    license='NODAR Limited Copyright License',
    license_files=['LICENSE'],
    project_urls={
    'License': 'https://github.com/nodarhub/hammerhead_ros2/blob/main/LICENSE',
    },
    entry_points={
        'console_scripts': [
            f'exposure = {package_name}.{package_name}:main_exposure',
            f'gain = {package_name}.{package_name}:main_gain',
        ],
    },
)
