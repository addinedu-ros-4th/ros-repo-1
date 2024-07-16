from setuptools import find_packages, setup
import glob

package_name = 'path_planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + '/map/', glob.glob('map/*.*')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'nav2_simple_commander'],
    zip_safe=True,
    maintainer='gudxok_',
    maintainer_email='gudxok_@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_astar_nav = src.nav2_astar:main',
        ],
    },
)
