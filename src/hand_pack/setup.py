from setuptools import setup
import os
from glob import glob

package_name = 'hand_pack'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', glob('launch/*.py')),
    ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),
    ('share/' + package_name + '/urdf/meshes', glob('urdf/meshes/*')),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lucas-pc',
    maintainer_email='lucaspmartins14@gmail.com',
    description='Pacote para controle da mão robótica no ROS2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hand_node = hand_pack.hand_node:main',
        ],
    },
)
