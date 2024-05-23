from setuptools import find_packages, setup

package_name = 'connection'

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
    maintainer='hj',
    maintainer_email='xxbb96@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'msg_publisher = connection.msg_publisher:main',
            'msg_subscriber = connection.msg_subscriber:main',
            'action_server = connection.action_server:main',
            'action_client = connection.action_client:main',
            'talker = connection.talker:main',
            'robot_command_server = connection.robot_command_server:main',
            'Admin_Manager = connection.Admin_Manager:main',
            'Robot_Manager = connection.Robot_Manager:main',
            'User_GUI = connection.User_GUI:main',
        ],
    },
)
