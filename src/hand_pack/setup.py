from setuptools import setup, find_packages
from glob import glob

package_name = 'hand_pack'

setup(
    name=package_name,
    version='0.0.0',
    # procura pacotes na pasta atual (onde está este setup.py)
    packages=find_packages(exclude=['test']),
    # mapeia o “root” dos pacotes para o diretório atual
    package_dir={'': '.'},

    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', glob('launch/*.py')),
        (f'share/{package_name}/worlds', glob('worlds/*.wbt')),
        (f'share/{package_name}/urdf', glob('urdf/*.urdf')),
        (f'share/{package_name}/urdf/linear_meshes', glob('urdf/linear_meshes/*')),
        (f'share/{package_name}/controllers/arm_controller',
         glob('controllers/arm_controller/*')),
        (f'share/{package_name}/controllers/hand_controller',
         glob('controllers/hand_controller/*')),
        (f'share/{package_name}/controllers/conveyor_belt',
         glob('controllers/conveyor_belt/*')),
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
            'hand_node       = hand_pack.hand_node:main',
            'hand_controller = hand_pack.controllers.hand_controller:main',
        ],
    },
)
