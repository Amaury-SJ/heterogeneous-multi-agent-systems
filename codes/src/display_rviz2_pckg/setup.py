import os
from glob import glob
from setuptools import setup

package_name = 'display_rviz2_pckg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Amaury Saint-Jore',
    maintainer_email='amausj@gmail.com',
    description='Data visualization in rviz2.',
    license='GPL-3.0-or-later',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rviz2_map_visualization = display_rviz2_pckg.rviz2_map_visualization:main',
            'rviz2_gps_visualization = display_rviz2_pckg.rviz2_gps_visualization:main'
        ],
    },
)
