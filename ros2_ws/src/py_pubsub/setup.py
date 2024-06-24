import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'py_pubsub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vitorfgadelha',
    maintainer_email='vitorfgadelha@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker_1 = py_pubsub.publisher_string:main',
            'listener = py_pubsub.subscriber:main',
            'talker_2 = py_pubsub.publisher_int:main',
            'id_check = py_pubsub.publish_id:main',
            'go = py_pubsub.test:main',
        ],
    },
)
