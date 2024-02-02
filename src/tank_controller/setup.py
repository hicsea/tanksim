from setuptools import setup

package_name = 'tank_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

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
            'tank_controller_node = tank_controller.tank_controller_node:main',
            'route_initializer_node = tank_controller.route_initializer_node:main',
            'obstacle_initializer_node = tank_controller.obstacle_initializer_node:main',    
        ],
    },
)
