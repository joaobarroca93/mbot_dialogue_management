#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# for your packages to be recognized by python
d = generate_distutils_setup(
 packages=['mbot_dialogue_management', 'mbot_dialogue_management_ros'],
 package_dir={'mbot_dialogue_management': 'common/src/mbot_dialogue_management', 'mbot_dialogue_management_ros': 'ros/src/mbot_dialogue_management_ros'}
)

setup(**d)
