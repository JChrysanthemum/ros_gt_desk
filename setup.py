#!/usr/bin/env python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup()
d['packages'] = ['wrapper_modbus', 'tools']
d['package_dir'] = {'': 'scripts', 'tools': './tools'}

setup(**d)