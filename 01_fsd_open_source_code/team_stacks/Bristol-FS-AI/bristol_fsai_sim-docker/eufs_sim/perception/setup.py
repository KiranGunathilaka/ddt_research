from setuptools import setup
import os
from glob import glob

package_name = 'perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ('share/' + package_name + '/models', [package_name + '/models/yolov11n_640.pt']),
        # ('share/' + package_name + '/models', [package_name + '/models/yolov5n_640.pt']),
        ('share/' + package_name + '/models', glob(package_name + '/models/*.pt')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'opencv-python',
        'torch',
        'ultralytics',
        'PyYAML',
    ],
    zip_safe=True,
    maintainer='bristol_fsai',
    maintainer_email='tom.lam@odns.hk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'YOLONode = perception.YOLONode:main',
            'ConePositionEstimator = perception.ConePositionEstimator:main',
        ],
    },
)
