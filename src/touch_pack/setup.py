from setuptools import setup
import os
from glob import glob

package_name = 'touch_pack'

setup(
    name=package_name,
    version='0.3.0',
    packages=['touch_pack'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lucas Martins',
    maintainer_email='lucaspmartins14@gmail.com',
    description=('Plataforma de palpação tátil — CR10 + COVVI Index FT, '
                 'reproduzindo o protocolo de Gupta et al. 2021.'),
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'palpation_gui     = touch_pack.palpation_gui:main',
            'tactile_explorer  = touch_pack.tactile_explorer:main',
        ],
    },
)
