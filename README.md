### Installation

The best way to use `pinocchio` python bindings is using Anaconda. Create a `conda` virtual environment and then install `pinocchio`
using the instructions reported [here](https://github.com/conda-forge/pinocchio-feedstock). 

### Dependencies
- `numpy`
- `scipy`
- `pybullet` (for testing against a simulator)
 
### Usage with Ros

Ros uses by default python2.7 while this package requires `python3.6`. The quickest method to use the library together with 
ros was to set the correct python executable at the beginning of each file. Look at [this script](test/pinocchio_control_test.py) for an example. 
You will need to change this to your corresponding path. Additionally, I had to install rospkg for python 3.x (`pip3 install rospkg`)

### Examples

Although `pybullet` is not a necessary dependency, it has been used for testing. An example is available [here](test/pinocchio_control_test.py) 
where a robotic arm is controller and results are checked in a pybullet simulation.  

### TODO 
[x] Test with basic controllers

[ ] Updated setup script 

[ ] Better way to handle python2.7 / python3.x

[ ] CI

[ ] Arm generic test
