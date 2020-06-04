### Installation

To support both python 2 and 3, we use anaconda to provide the python 3 environment. Create a `conda` virtual environment with `conda create -n <name> Python=3.7`. Install `pinocchio`
using the instructions reported [here](https://github.com/conda-forge/pinocchio-feedstock). To install this package in the current
conda environment (after conda has been activate) navigate to this directory and enter `pip install .`

The package has to be built using `catkin b -DPYTHON_ENV=<path-to-conda-env>`. The path to the conda environment would be e.g. `/home/<user>/miniconda3/envs/<env_name>` if using miniconda.

Full disclosure: This will build the c++ library python bindings and copy it to the anaconda environment. Let us know if you can't import the library after building.

### Dependencies
- `numpy`
- `scipy`
- `pybullet` (for testing against a simulator)

### Usage with Ros

Ros uses by default python2.7 while this package requires `python3.6`. The quickest method to use the library together with
ros was to set the correct python executable at the beginning of each file. Look at [this script](test/controllers/test_op_space_controller.py) for an example.
You will need to change this to your corresponding path. Additionally, I had to install rospkg for python 3.x (`pip3 install rospkg`)

### Examples

Although `pybullet` is not a necessary dependency, it has been used for testing. An example is available [here](test/controllers/test_op_space_controller.py)
where a robotic arm is controller and results are checked in a pybullet simulation.

### TODO
[x] Test with basic controller

[ ] End effector motion controller

[ ] Urdf based actuation limiter

[ ] Updated setup script

[ ] Better way to handle python2.7 / python3.x

[ ] CI

[ ] Arm generic test

[ ] Cpp implementation

