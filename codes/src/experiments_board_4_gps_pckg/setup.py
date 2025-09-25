from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'experiments_board_4_gps_pckg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Amaury Saint-Jore',
    maintainer_email='amausj@gmail.com',
    description='Package containing nodes and launch for the experiments with the board and the 4 RTK-GPS.',
    license='GPL-3.0-or-later',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calculate_distance_gps = experiments_board_4_gps_pckg.calculate_distance_gps:main'
        ],
    },
)
