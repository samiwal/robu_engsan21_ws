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
            'led_strip_pub_engsan=robu.plf01_ledstrip_pub_engsan21:main',
            'led_strip_sub_engsan=robu.ledstrip_sub_engsan:main',
            'ledstrip_pub=robu.ledstrip_pub:main',
            'ledstrip_sub=robu.ledstrip_sub:main',
            'distance_sensor_pub=robu.ue08_distance:main_distance_sensor',
            'obstacle_avoider_sub=robu.ue08_distance:main_obstacle_avoider',
            'simple_kinematics=robu.ue09:main',
        ],
    },
)
