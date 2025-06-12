from setuptools import setup

package_name = 'image_viewer_py'

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
    description='This example is a simple OpenCV viewer for ROS2 images published by hammerhead.',
    license='NODAR Limited Copyright License',
    # license_files=['LICENSE'],
    project_urls={
    'License': 'https://github.com/nodarhub/hammerhead_ros2/blob/main/LICENSE',
    },
    entry_points={
        'console_scripts': [
            f'{package_name} = {package_name}.{package_name}:main',
        ],
    },
)
