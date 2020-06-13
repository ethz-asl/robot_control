import setuptools
from setuptools import setup, Extension

setuptools.setup(name='robot_control_bullet',
                 version='0.0.1',
                 packages=['robot_control_bullet',
                           'robot_control_bullet.simulation',
                           'robot_control_bullet.simulation.sensors'],
                 package_dir={'': 'src'})

