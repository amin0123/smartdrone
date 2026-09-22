from glob import glob
from setuptools import setup

package_name = 'drone_docking'
setup(
    name=package_name, version='0.1.0', packages=[package_name],
    data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                ('share/' + package_name, ['package.xml']),
                ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
                ('share/' + package_name + '/config', glob('config/*'))],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Smartdrone maintainers', maintainer_email='maintainers@example.invalid',
    description='PX4 agricultural inspection: drone_docking', license='Apache-2.0',
    entry_points={'console_scripts': ['marker_detector = drone_docking.node:main']},
)

