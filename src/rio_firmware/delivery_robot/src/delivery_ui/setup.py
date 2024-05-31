from setuptools import find_packages, setup
import glob

package_name = 'delivery_ui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test'], include=['rfid_reader','delivery_ui']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + "/ui/", glob.glob('delivery_ui/ui/*.ui')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jiho',
    maintainer_email='jiho@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'delivery_service = delivery_ui.delivery_service:main',
            "delivery_ui = delivery_ui.delivery_ui:main"
        ],
    },
)
