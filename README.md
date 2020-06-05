### Installation

Install the `pinocchio` library following the [instructions](https://stack-of-tasks.github.io/pinocchio/download.html) from the official documentation.

#### Installing `pinocchio` bindings (Optional)

Follow the instructions reported [here](https://github.com/conda-forge/pinocchio-feedstock). To install the python scripts for this package in the current conda environment (after conda has been activate) navigate to this directory and enter `pip install .`

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

