import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'qr_autonomy'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='emna',
    maintainer_email='you@example.com',
    description='Table-free autonomous navigation: the robot follows commands written in QR codes',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autonomy_node = qr_autonomy.autonomy_node:main',
        ],
    },
)
