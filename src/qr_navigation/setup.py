import os
from glob import glob
from setuptools import setup

package_name = 'qr_navigation'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ichrak',
    maintainer_email='ichrak.ayadi@etudiant-enit.utm.tn',
    description='QR localization, guidance and mission nodes',
    license='MIT',
    entry_points={
        'console_scripts': [
            'localization_node = qr_navigation.localization_node:main',
            'guidance_node = qr_navigation.guidance_node:main',
            'mission_node = qr_navigation.mission_node:main',
        ],
    },
)
