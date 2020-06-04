import setuptools

with open("README.md", 'r') as f:
    long_description = f.read()

setuptools.setup(name='robot_control',
                 version='0.0.1',
                 long_description=long_description,
                 packages=['robot_control',
                           'robot_control.controllers',
                           'robot_control.controllers.implementation',
                           'robot_control.controllers.utilities',
                           'robot_control.modeling',
                           'robot_control.optimization',
                           'robot_control.simulation',
                           'robot_control.simulation.sensors'],
                 package_dir={'': 'src'},
                 python_requires='>=3.6',
                 install_requires=['numpy', 'scipy', 'pybullet'])
