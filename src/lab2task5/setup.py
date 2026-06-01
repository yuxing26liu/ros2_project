from setuptools import find_packages, setup

package_name = 'lab2task5'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lriek',
    maintainer_email='lriek@ucsd.edu',
    description='Go Pupper Service: Robot commands as a service',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['service = lab2task5.service_go_pupper:main',
                            'client = lab2task5.client_go_pupper:main',
                            'salsa = lab2task5.sample_controller:main',
        ],
    },
)

