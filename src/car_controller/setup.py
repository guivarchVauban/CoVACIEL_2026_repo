from setuptools import find_packages, setup

package_name = 'car_controller'

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
    maintainer='ros',
    maintainer_email='ros@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
	entry_points={
        'console_scripts': [
            'car_node = car_controller.car_node:main',
            'arduino_bridge = car_controller.arduino_bridge:main',
        ],
    },
)
