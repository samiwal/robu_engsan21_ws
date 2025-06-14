from setuptools import find_packages, setup

package_name = 'motor_ctrl'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'publisher=motor_ctrl.int_publisher:main',
            'subscriber=motor_ctrl.motor_subscriber:main',
            'motor_ctrl=motor_ctrl.motor_ctrl:main',
        ],
    },
)
