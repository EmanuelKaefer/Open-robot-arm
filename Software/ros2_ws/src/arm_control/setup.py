from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'arm_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # installs all files from config/ into share/arm_control/config (empty for now)
        (os.path.join('share', package_name, 'config'), glob('config/*')),

        # installs all *.launch.py files from launch/ into share/arm_control/launch (empty for now)
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='matteo',
    maintainer_email='matteo207@gmx.at',
    description='High-level helper nodes for the robot arm (e.g. homing service). '
                'The UART link to the STM itself is the ros2_control plugin in arm_hardware.',
    license="""
        * SPDX-License-Identifier: GPL-3.0-or-later
        * Copyright (c) 2026 Emanuel Käfer, Matteo Gassler, and Alexander Dick and contributors
        * This program is free software: you can redistribute it and/or modify
        * it under the terms of the GNU General Public License as published by
        * the Free Software Foundation, either version 3 of the License, or
        * (at your option) any later version.
        * This program is distributed WITHOUT ANY EXPRESS OR IMPLIED WARRANTY.
        * See LICENSE-SOFTWARE in the project root for details. """,

    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # LATER: enable once arm_control/home_service.py has a main()
            # 'home_service = arm_control.home_service:main',
        ],
    },
)
