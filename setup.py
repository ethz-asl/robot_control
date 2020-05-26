import setuptools

setuptools.setup(name='robot_control',
                 version='0.0.1',
                 packages=['robot_control', 'robot_control.controllers'],
                 package_dir={'': 'src'},
                 python_requires='>=3.6')


