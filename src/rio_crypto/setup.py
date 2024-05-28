import glob
from setuptools import find_packages, setup


package_name = 'rio_crypto'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + "/config/keys/", glob.glob('rio_crypto/config/keys/*.pem')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='subin',
    maintainer_email='lsv2620@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'data_decryptor = rio_crypto.data_decryptor:main',
            'data_encryptor = rio_crypto.data_encryptor:main',
            'key_save_load = rio_crypto.key_save_load:main'
        ],
    },
)
