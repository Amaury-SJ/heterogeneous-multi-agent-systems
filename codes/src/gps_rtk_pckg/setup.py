from setuptools import setup

package_name = 'gps_rtk_pckg'

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
    maintainer='Amaury Saint-Jore',
    maintainer_email='amausj@gmail.com',
    description='Package for GNSS/GPS receivers (like Emlid RTK).',
    license='GPL-3.0-or-later',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_talker_wifi = gps_rtk_pckg.gps_talker_wifi:main',
            'gps_talker_usb = gps_rtk_pckg.gps_talker_usb:main'
        ],
    },
)
