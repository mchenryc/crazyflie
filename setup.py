#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

#Fetch values from package.xml when using catkin
setup_args=generate_distutils_setup(
    scripts=['scripts/cf_node', 'scripts/cf_teleop'],
    packages=['crazyflie'])

setup(**setup_args)
