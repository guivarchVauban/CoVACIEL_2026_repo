from setuptools import find_packages, setup

package_name = 'laser_scan_to_point_cloud'

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
    maintainer='sguivarch',
    maintainer_email='sebastien.guivarc-h@ac-rennes.fr',
    description='Convertit un topic LaserScan en PointCloud2 pour utilisation avec KISS-ICP',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'laser_scan_to_pc = laser_scan_to_point_cloud.laser_scan_to_point_cloud:main',
        ],
    },
)
