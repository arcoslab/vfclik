#!/usr/bin/env python

from distutils.core import setup


setup(
    name='vfclik',
    version='0.1',
    description='Vector field closed loop inverse kinematics',
    author='Federico Ruiz Ugalde',
    author_email='memeruiz@gmail.com',
    url='http://www.arcoslab.org/',
    package_dir={'vfclik': 'src'},
    packages=['vfclik'],
    scripts=['scripts/vfclik', 'scripts/bridge',
             'scripts/debug_jointlimits',
             'scripts/joint_p_controller',
             'scripts/monitor_distance', 'scripts/nullspace',
             'scripts/object_feeder', 'scripts/vf']
)
