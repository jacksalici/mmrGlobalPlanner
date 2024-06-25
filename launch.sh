#!/bin/bash
## Script to exectute all the nodes needed for the global plannel as well as the global planner itself.
declare -a commands=(
    "ros2 launch waypoint_generator waypoint_generator.launch.py"
    "ros2 launch lap_counter lap_counter.launch.py"
    "ros2 launch global_planner global_planner.launch.py"
    "ros2 bag play /home/jack/Downloads/BAG24/05_11_test/v1 -r 6"
    "ros2 topic echo /planning/speedProfilePoints"
)
for i in "${commands[@]}"
    do
        echo "$i"
        gnome-terminal --tab --title "$1" -- bash -c "$i; exec bash"
    done



