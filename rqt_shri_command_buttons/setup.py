#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['rqt_shri_command_buttons', 'rqt_shri_command_buttons.plugins'],
    package_dir={'': 'src'},
    scripts=['scripts/rqt_shri_command_buttons']
)

setup(**d)