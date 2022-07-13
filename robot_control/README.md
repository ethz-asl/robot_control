## Dependencies

- `ros-melodic-orocos-kdl`
- `ros-melodic-kdl-parser`

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
