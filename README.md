# picopi
example of ROS2 node in C++ to manage a serial port to a raspberry pi pico

## command lines

to build :
```
colcon build --packages-select picopi
```

to run :
```
ros2 run picopi publisher_picopi --ros-args -p port:=/dev/ttyACM0
```

## where to start

The node is defined in src/publisher_picopi.cpp
