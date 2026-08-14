# Copyright 2026 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Woojin Wie

from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'ai_sapiens_bringup'


def package_files(directory):
    paths = []
    for path in glob(os.path.join(directory, '**', '*'), recursive=True):
        if os.path.isfile(path):
            install_dir = os.path.join('share', package_name, os.path.dirname(path))
            paths.append((install_dir, [path]))
    return paths


setup(
    name=package_name,
    version='0.1.2',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        *package_files('config'),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pyo',
    maintainer_email='pyo@robotis.com',
    author='Woojin Wie',
    author_email='wwj@robotis.com',
    description='Launch and controller configuration for the AI Sapiens K1 robot stack',
    license='Apache 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
