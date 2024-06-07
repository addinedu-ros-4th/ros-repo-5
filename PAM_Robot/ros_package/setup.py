from setuptools import find_packages, setup
import os
import glob

package_name = 'ros_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch' , glob.glob(os.path.join('launch','*.launch.py'))),
    ],
    py_modules=['ros_package.routes'],  # routes 모듈 추가
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jongchanjang',
    maintainer_email='mawinerttt@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'Robot_Topic = ros_package.Robot_Topic:main',
            'Robot_Service = ros_package.Robot_Service:main',
            'Robot_Driving = ros_package.Robot_Driving:main',
            'camera = ros_package.camera:main',
            'voice_recognize = ros_package.voice_recognize:main',
        ],
    },
)
