from setuptools import setup

package_name = 'generate_rosbag2_py'

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
    description='This package converts the data recorded by hammerhead into a ROS2 bag.',
    license='NODAR Limited Copyright License',
    # license_files=['LICENSE'],
    project_urls={
    'License': 'https://github.com/nodarhub/hammerhead_ros2/blob/main/LICENSE',
    },
    entry_points={
        'console_scripts': [
            f'everything = {package_name}.everything:main',
            f'xyz = {package_name}.xyz:main',
        ],
    },
)
