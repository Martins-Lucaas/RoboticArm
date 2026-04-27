import os
from glob import glob
from setuptools import setup

package_name = 'hand_pack'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name], # APENAS o nome do pacote aqui
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.xml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'urdf', 'linear_meshes'), glob('urdf/linear_meshes/*.STL')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lucas-pc',
    maintainer_email='lucaspmartins14@gmail.com',
    description='Digital Twin da COVVI Hand',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hand_gui = hand_pack.hand_gui:main',
        ],
    },
)