from setuptools import setup
import glob

package_name = 'mic_client'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rosidl_default_runtime'],
    zip_safe=True,
    maintainer='ito',
    maintainer_email='ito@example.com',
    description='Example package with simple service client and server',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'srv_server = mic_client.service_server:main',
            'srv_client = mic_client.service_client:main',
        ],
    },
)