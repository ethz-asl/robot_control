Dependencies:
- `ros-melodic-orocos-kdl`
- `ros-melodic-kdl-parser`

To test, go into the package directory and call

```
catkin run_tests --no-deps --this
```

To just build the tests:

```
catkin build robot_control --catkin-make-args run_tests
```
