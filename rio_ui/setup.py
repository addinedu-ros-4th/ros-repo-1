from setuptools import find_packages, setup
import glob
import os

package_name = 'rio_ui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test'], include=['rio_ui','rio_db_manager', "rio_recognition"]),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + "/ui/", glob.glob('rio_ui/ui/*.ui')),
        # ('share/' + package_name + '/data', glob.glob('rio_ui/data/*.qrc')),
        # ('share/' + package_name + '/data', glob.glob('rio_ui/data/*.png')),
        ('share/' + package_name + '/data/keys/', glob.glob('rio_ui/data/keys/*.pem')),
        ('share/' + package_name + '/data/', glob.glob('rio_ui/data/*.json')),
        ('share/' + package_name + '/data/', glob.glob('rio_ui/data/*.yaml')),
        ('share/' + package_name + '/data/', glob.glob('rio_ui/data/rio_loggo.png')),
        

    ],
    # py_modules=['rio_ui.resource_rc'],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='joe',
    maintainer_email='dlwlgh0106@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'admin_service = rio_ui.admin_service:main',
            'user_service = rio_ui.user_service:main',
            'delivery_service=rio_ui.delivery_service:main',
            'admin_gui = rio_ui.admin_gui:main',
            'user_gui = rio_ui.user_gui:main',
            'guide_service = rio_ui.guide_service:main',
            'guide_gui = rio_ui.guide_gui:main',
            'key_save_load = rio_ui.key_save_load:main',
            'face_recog_node = rio_ui.face_recog_node:main',
        ],
    },
)
