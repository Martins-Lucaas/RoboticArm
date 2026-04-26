import os
from glob import glob
from setuptools import setup

package_name = 'hand_pack'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Inclui os Launch Files (Python e XML)
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.xml')),
        
        # Inclui o URDF base
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        
        # --- A LINHA QUE FALTAVA: Copia as malhas visuais (STLs) ---
        (os.path.join('share', package_name, 'urdf', 'linear_meshes'), glob('urdf/linear_meshes/*.STL')),
        
        # Inclui o Mundo do Gazebo
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        # Inclui as configurações de Controle
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lucas-pc',
    maintainer_email='lucaspmartins14@gmail.com',
    description='Digital Twin da COVVI Hand para Gazebo e ROS 2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)