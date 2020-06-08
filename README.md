### robot_control 
[![Build Status](https://jenkins.asl.ethz.ch/buildStatus/icon?job=robot_control)](https://jenkins.asl.ethz.ch/job/robot_control/)

### Installation

The best way to use `pinocchio` python bindings is using Anaconda. Create a `conda` virtual environment and then install `pinocchio`
using the instructions reported [here](https://github.com/conda-forge/pinocchio-feedstock). To install this package in the current
conda environment (after conda has been activate) navigate to this directory and enter `pip install .`

The current python-based version of the package requires installation of Anaconda and creation of a separate environment containing
all the necessary dependencies. 
1. Install _Anaconda_
    - Install _Anaconda_ for Python 3.x as explained [here](https://docs.anaconda.com/anaconda/install/linux/). 
The installer prompts `“Do you wish the installer to initialize Anaconda3 by running conda init?”` Select “no” to avoid conda
to be used as the default python interpreter.
    - Export the path to the conda binaries. Add the following line to your `.bashrc`: `PATH="$PATH:<path-to-conda-bin-folder>"`.
If, for instance, conda is installed in `/home/user/Programs/anaconda` this is generally `/home/user/Programs/anaconda`
2. Create a new _virtual environmet_ 
    - Make sure the `PYTHONPATH` is updated with ROS packages: `source /opt/ros/melodic/setup.bash`
    - `conda config --add channels conda-forge`
    - `conda create -n robot_control python=3.6 pinocchio`
    - `conda activate robot_control`
3. Install `robot_control`
    - cd into the current directory and type `pip install .` This will also install additional dependencies.
4. Build the ROS package
    - `catkin build robot_control`

### Examples

This will launch a task space controller for the end effector launch both RViz and the OpenGL bullet gui.
`roslaunch robot_control test_controller.launch`

### TODO 

Check the Github project!
