from setuptools import setup
import os
from glob import glob

package_name = 'tank_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'srv'), glob('srv/*.srv')),

    ],
    install_requires=['setuptools', 'route_initializer'],
    zip_safe=True,
    maintainer='hicsea',
    maintainer_email='hicsea@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tank_controller_node = tank_controller.tank_controller_node:main',
        ],
    },
)
