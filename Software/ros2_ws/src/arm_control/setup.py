from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'arm_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        #kopiert alle .yaml files aus seup in das shared verzeichnis install
        (os.path.join('share', package_name, 'config'), glob('config/*')),

        #kopiert alle .launch.py files aus launch in das shared verzeichnis install
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='matteo',
    maintainer_email='matteo207@gmx.at',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'step_publisher = arm_control.step_publisher:main',
            'step_converter = arm_control.step_to_angle_converter:main',
            'serial_bridge = arm_control.serial_bridge:main',
        ],
    },
    
)
