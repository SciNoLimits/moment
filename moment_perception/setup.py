from glob import glob
from setuptools import setup

package_name = 'moment_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/sim', glob('sim/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Charan Kalyana Kumar',
    maintainer_email='charan.kalyana-kumar@stud.th-deg.de',
    description='RGB-D colour detection for the MOMENT robot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'color_detector = moment_perception.detector_node:main',
        ],
    },
)
