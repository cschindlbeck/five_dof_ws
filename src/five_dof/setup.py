# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
"""
Package setup file for five_dof
"""

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(packages=["five_dof"], package_dir={"": "src"})

setup(**setup_args)
