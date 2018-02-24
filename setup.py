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
    scripts=['scripts/vfclik.py', 'scripts/bridge.py',
             'scripts/debug_jointlimits.py',
             'scripts/joint_p_controller.py',
             'scripts/monitor_distance.py', 'scripts/nullspace.py',
             'scripts/object_feeder.py', 'scripts/vf.py', 'scripts/vfclik.py']
)
