from setuptools import find_packages
from setuptools import setup

setup(
    name='voice_driver',
    version='0.1.0',
    packages=find_packages(
        include=('voice_driver', 'voice_driver.*')),
)
