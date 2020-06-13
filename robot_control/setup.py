import os
import setuptools
from setuptools import Extension
from setuptools.command.build_ext import build_ext as build_ext_orig


class CMakeExtension(Extension):
    def __init__(self, name):
        super(CMakeExtension).__init__(name, sources=[])


class build_ext(build_ext_orig):
    def run(self):
        for ext in self.extensions:
            self.build_cmake(ext)
        super(build_ext).run()

    def build_cmake(self, ext):
        output_dir = os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name)))

        config = 'Debug' if self.debug else 'Release'
        cmake_args = [
                '-DCMAKE_LIBRARY_OUTPUT_DIRECTORY=' + output_dir,
                '-DCMAKE_BUILD_TYPE=' + config,
                '-DCATKIN_BUILD=FALSE'
        ]

        build_args = [
            '--config', config,
            '--', '-j4'
        ]

        build_temp = os.path.abspath(self.build_temp)
        os.makedirs(build_temp, exist_ok=True)
        cwd = os.path.abspath(os.path.curdir)
        os.chdir(build_temp)
        self.spawn(['cmake', cwd] + cmake_args)
        if not self.dry_run:
            self.spawn(['cmake', '--build', '.'] + build_args)
        os.chdir(cwd)


with open("../README.md", 'r') as f:
    long_description = f.read()

setuptools.setup(name='robot_control',
                 version='0.0.1',
                 long_description=long_description,
                 python_requires='>=3.6',
                 ext_modules=[CMakeExtension('rc')],
                 cmdclass={
                     'build_ext': build_ext
                     })
