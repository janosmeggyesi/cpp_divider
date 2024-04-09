# CPP Divider

This package provides a node that performs a simple mathematical operation. The node subscribes to a ROS2 topic (/input_numbers), receives two floating-point numbers, divides them, and then publishes the result on a new topic (/division_result).

## Build

### Linux
To build this project run the following commands in the root of your workspace:

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
rosdep update
rosdep install -i --from-path src --rosdistro ${ROS_DISTRO} -y
colcon build --packages-select cpp_divider
```

## Usage
To run the node, open a new terminal and run the following commands in the root of your workspace:

```bash
source install/setup.bash
ros2 run cpp_divider cpp_divider_node
```

To check the output, you can echo the data published on the /division_result. Open a new terminal and run the following commands in the root of your workspace:

```bash
source install/setup.bash
ros2 topic echo /division_result
```

To trigger the output, you should publish new data to the /input_numbers topic. Open a new terminal and run the following commands in the root of your workspace:

```bash
source install/setup.bash
ros2 topic pub --once /input_numbers cpp_divider/msg/DivisionInputFloats "{dividend: {data: 4.2}, divisor: {data: 2.0}}"
```
