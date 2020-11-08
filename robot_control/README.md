## Build

`catkin build robot_control`

## Examples

### IK Solver
Use the interactive marker to move the target for the end effector pose. Once the target is place in the 
desired configuration, send the target to the end effector IK solver using the `/set_target` service. 
- Terminal #1: `roslaunch robot_control ik_debug.launch`
- Terminal #2: `rosservice call /set_target "{}"`

## Testing

To test, go into the package directory and call

```
catkin run_tests --no-deps --this
```

To just build the tests:

```
catkin build robot_control --catkin-make-args run_tests
```

If the code depends on ROS functionality, e.g. reading parameters from the parameter server, the a test can be launched using a launch file. For example, see `robot_control/launch/cartesian_velocity.test`. It can be launch using the following command. Make sure a ROS master is running before running the test.

```
rostest robot_control cartesian_velocity.test
```
