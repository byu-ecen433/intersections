#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------

# On the robot, next to your Lab 4 lane_following stack. THIS ONE RUNS ON THE BOT:
#
#     dts devel build -H DUCKIEBOT_NAME -f
#     dts devel run   -H DUCKIEBOT_NAME -L intersections
#
# stop_line_filter_node starts switched off, so turn it on first. Waits here
# until the Lab 4 stack is up.
until rosservice call "/${VEHICLE_NAME}/stop_line_filter_node/switch" "data: true" > /dev/null 2>&1; do
    echo "waiting for stop_line_filter_node - is your Lab 4 lane_following running?"
    sleep 2
done
dt-exec roslaunch intersection_control intersections.launch test:=false

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
