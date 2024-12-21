from setuptools import setup
from glob import glob
import os

package_name = 'drone_launch_package'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    data_files=[
        # Install package.xml
        (os.path.join('share', package_name), ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='daniel',
    maintainer_email='danielmunicio360@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
