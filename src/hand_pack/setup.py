from setuptools import setup
import os
from glob import glob

package_name = 'hand_pack'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Copia os launchers
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*.py'))),  
        # Copia os modelos URDF e XACRO
        ('share/' + package_name + '/urdf', glob(os.path.join('urdf', '*.xacro'))),  
        ('share/' + package_name + '/urdf', glob(os.path.join('urdf', '*.urdf'))),  
        # Copia as pastas de malhas 3D (Meshes)
        ('share/' + package_name + '/urdf/mesh', glob(os.path.join('urdf', 'mesh', '*.*'))),  
        ('share/' + package_name + '/urdf/linear_meshes', glob(os.path.join('urdf', 'linear_meshes', '*.*'))), # <-- ADICIONADO PARA A NOVA MÃO
        # Copia configurações do RViz (Opcional, mas muito recomendado)
        ('share/' + package_name + '/rviz', glob(os.path.join('rviz', '*.rviz'))), 
        ('share/' + package_name + '/config', glob(os.path.join('config', '*.yaml'))),
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