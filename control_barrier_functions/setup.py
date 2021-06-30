from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['control_barrier_functions'],
    #scripts=['scripts'],
    package_dir={'': 'src'}
)

setup(**d)
