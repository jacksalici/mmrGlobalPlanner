To build ROS with the needed custom messages please copy the `.msg` files from the current folder to `/src/0_common/mmr_base/msg`. 
_Please note that this is just for our own architecure that is not included in this repo._

Moreover, add the following lines to the `cmakefile.txt`.

```txt
find_package(ackermann_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SpeedProfilePoint.msg"
  "msg/SpeedProfilePoints.msg")
  DEPENDENCIES std_msgs geometry_msgs ackermann_msgs
```

Build the package with: (from the main folder) 
```sh
colcon build --packages-select mmr_base --symlink-install
source ./install/setup.bash
```