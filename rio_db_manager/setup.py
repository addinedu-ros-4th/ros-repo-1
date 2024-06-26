from setuptools import find_packages, setup
import glob

package_name = 'rio_db_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ("share/" + package_name + "/config/", glob.glob('rio_db_manager/config/*.json')),
    ],
    install_requires=['setuptools', "pymysql", "json"],
    zip_safe=True,
    maintainer='joe',
    maintainer_email='dlwlgh0106@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'db_manager = rio_db_manager.db_manager:main',
            'create_init_db = rio_db_manager.create_init_db:main'
        ],
    },
)
