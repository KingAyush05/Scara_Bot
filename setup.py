import os
from glob import glob
from setuptools import setup
from setuptools import find_packages
from setuptools import setup

package_name = 'scara_bot_2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name), glob('urdf/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kingayush',
    maintainer_email='kingayush@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_pub = scara_bot_2.state_publisher:main',
            'sp = scara_bot_2.sp2:main',
            'angle_pub = scara_bot_2.angle_publisher:main',
            'angle_sub = scara_bot_2.angle_subscriber:main',
        ],
    },
)
