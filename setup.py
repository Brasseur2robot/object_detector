import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'object_detector'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=[
        'setuptools',
        'pyserial',
    ],
    zip_safe=True,
    maintainer='Brasseur2robot',
    maintainer_email='brasseur@brasseur.beer',
    description='ROS2 package to detect object using LDROBOT Lidar',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'object_detector = object_detector.object_detector:main',
            'uart_sender = object_detector.uart_sender:main'
        ],
    },
)
