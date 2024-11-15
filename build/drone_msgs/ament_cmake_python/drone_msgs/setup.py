from setuptools import find_packages
from setuptools import setup

setup(
    name='drone_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('drone_msgs', 'drone_msgs.*')),
)
