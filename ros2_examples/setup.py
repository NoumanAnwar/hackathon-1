from setuptools import find_packages, setup

package_name = 'ros2_examples'

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
    maintainer='User',
    maintainer_email='user@example.com',
    description='Python examples for the ROS 2 book module',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'minimal_publisher = ros2_examples.publisher_node:main',
            'minimal_subscriber = ros2_examples.subscriber_node:main',
        ],
    },
)
