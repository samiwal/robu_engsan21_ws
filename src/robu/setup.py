from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'robu'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
        (os.path.join('share', package_name, 'urdf'),glob('urdf/*.*')),
        (os.path.join('share', package_name, 'stl-file'),glob('stl-file/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='samuel',
    maintainer_email='engsan21@htl-kaindorf.at',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'remotectrl=robu.remote_control:main',
            'led_strip_pub=robu.plf01_ledstrip_pub_engsan21:main'
            'led_strip_sub=robu.ledstrip_sub_engsan:main'
        ],
    },
)
