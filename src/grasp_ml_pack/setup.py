from setuptools import setup
import os
from glob import glob

package_name = 'grasp_ml_pack'

setup(
    name=package_name,
    version='0.2.0',
    packages=['grasp_ml_pack', 'grasp_ml_pack.scripts'],
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
        (os.path.join('share', package_name, 'models'),
            glob('models/*') + ['models/.gitkeep']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lucas Martins',
    maintainer_email='lucaspmartins14@gmail.com',
    description='Conveyor cell pick-and-sort system — CR10 + COVVI Hand',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_detector      = grasp_ml_pack.object_detector:main',
            'grasp_executor       = grasp_ml_pack.grasp_executor:main',
            'conveyor_controller  = grasp_ml_pack.conveyor_controller:main',
            'gui_control          = grasp_ml_pack.gui_control_node:main',
            'manual_control       = grasp_ml_pack.manual_control_node:main',
            'teach_pendant        = grasp_ml_pack.teach_pendant:main',
            'pipeline             = grasp_ml_pack.pipeline:main',
            'test_kin             = grasp_ml_pack.scripts.test_kinematics:main',
        ],
    },
)
