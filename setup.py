from setuptools import find_packages, setup

package_name = 'pam_server'

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
    maintainer='djy0404',
    maintainer_email='dydrltk1919@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pam_server = pam_server.pam_server:main',
            'webcam_publisher = pam_server.web_cam_pub:main',
            'yolo_test_node = pam_server.yolo_test_node:main',
            'robot_command_server = pam_server.robot_command_server:main'
        ],
    },
)
