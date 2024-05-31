from setuptools import find_packages, setup
import glob

package_name = 'rio_ui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test'], include=['rio_ui','rio_db_manager']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + "/ui/", glob.glob('rio_ui/ui/*.ui')),
        

    ],
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
            "user_service = rio_ui.user_service:main",
            "delivery_service=rio_ui.delivery_service:main",
            'admin_gui = rio_ui.admin_gui:main',

        ],
    },
)
