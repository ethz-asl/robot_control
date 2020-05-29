import setuptools

setuptools.setup(name='robot_control',
                 version='0.0.1',
                 packages=['robot_control',
                           'robot_control.controllers',
                           'robot_control.controllers.implementation',
                           'robot_control.controllers.utilities',
                           'robot_control.modeling',
                           'robot_control.optimization',
                           'robot_control.simulation'],
                 package_dir={'': 'src'},
                 python_requires='>=3.6')
