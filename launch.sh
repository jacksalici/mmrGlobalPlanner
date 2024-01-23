#!/bin/bash
## Script to exectute all the nodes needed for the global plannel as well as the global planner itself.
declare -a commands=(
    "ros2 launch waypoint_generator waypoint_generator.launch.py"
    "ros2 launch lap_counter lap_counter.launch.py"
    "ros2 run global_planner runner"
    "ros2 bag play ~/Downloads/ROS_2/test_04_06/test_engine_10hz_2 -r 4"
)
for i in "${commands[@]}"
    do
        echo "$i"
        gnome-terminal --tab --title "$1" -- bash -c "$i; exec bash"
    done



