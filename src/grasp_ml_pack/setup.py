from setuptools import setup
import os
from glob import glob

package_name = 'grasp_ml_pack'

setup(
    name=package_name,
    version='0.1.0',
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
    description='ML-based autonomous grasp system for CR10 + COVVI Hand',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_detector    = grasp_ml_pack.object_detector:main',
            'pose_estimator     = grasp_ml_pack.pose_estimator:main',
            'grasp_planner      = grasp_ml_pack.grasp_planner:main',
            'grasp_executor     = grasp_ml_pack.grasp_executor:main',
            'pipeline           = grasp_ml_pack.pipeline:main',
            'generate_data      = grasp_ml_pack.scripts.generate_training_data:main',
            'train_model        = grasp_ml_pack.scripts.train_grasp_model:main',
            'test_kin           = grasp_ml_pack.scripts.test_kinematics:main',
        ],
    },
)
