import setuptools
from setuptools import setup

setuptools.setup(name='robot_control_bullet',
                 version='0.0.1',
                 packages=['robot_control_bullet',
                           'robot_control_bullet.simulation',
                           'robot_control_bullet.simulation.sensors'],
                 install_requires=['pybullet', 'numpy'],          
                 package_dir={'': 'src'})

