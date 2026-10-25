# Copyright 2026 Resilient Autonomy Lab
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

from glob import glob

from setuptools import setup

package_name = 'ral_faults'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # fault_catalogue.yaml and capabilities.yaml are RUNTIME INPUTS
        # resolved through the share directory (ral_faults/catalogue.py), so
        # a node reads the installed copy belonging to the build it runs, not
        # a source path that may not exist in the container.
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Resilient Autonomy Lab',
    maintainer_email='noreply@example.com',
    description='Fault-injection boundary nodes that sit on real data paths.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scan_fault_relay = ral_faults.scan_fault_relay:main',
            'fault_manager = ral_faults.fault_manager:main',
        ],
    },
)
