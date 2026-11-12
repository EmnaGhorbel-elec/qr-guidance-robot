from setuptools import setup

package_name = 'robot_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='emna',
    maintainer_email='you@example.com',
    description='Gazebo counterpart of robot_hardware',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'sim_base_node = robot_sim.sim_base_node:main',
    ]},
)
