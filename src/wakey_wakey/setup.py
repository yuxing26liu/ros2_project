from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'wakey_wakey'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        ('share/wakey_wakey/assets/images', glob('assets/images/*.png')),
        ('share/wakey_wakey/assets/sounds', glob('assets/sounds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chx029',
    maintainer_email='chx029@ucsd.edu',
    description='Wakey Wakey Service',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['state_machine = wakey_wakey.state_machine:main',
                            'flee_behavior = wakey_wakey.flee_behavior:main',
                            'detection = wakey_wakey.detection:main',
                            'wakefulness = wakey_wakey.wakefulness:main',
                            'game = wakey_wakey.game:main',
                            'audio = wakey_wakey.audio:main',
        ],
    },
)
