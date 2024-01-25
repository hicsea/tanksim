from setuptools import setup
import os
from glob import glob

package_name = 'route_initializer'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Directly include the .srv files
        # (os.path.join('share', package_name), glob('package.xml')),
        (os.path.join('share', package_name, 'srv'), glob('srv/*.srv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hicsea',
    maintainer_email='hicsea@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'route_initializer_node = route_initializer.route_initializer_node:main'
        ],
    },
)
