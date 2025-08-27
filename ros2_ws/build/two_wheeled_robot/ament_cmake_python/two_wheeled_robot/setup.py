from setuptools import find_packages
from setuptools import setup

setup(
    name='two_wheeled_robot',
    version='1.0.0',
    packages=find_packages(
        include=('two_wheeled_robot', 'two_wheeled_robot.*')),
)
