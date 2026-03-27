from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'servera_percept'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', 'servera_percept', 'models', 'yolov8n_openvino_model'),
        glob('models/yolov8n_openvino_model/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kobe',
    maintainer_email='koustavbetal@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'detect = servera_percept.detect:main'
        ],
    },
)
