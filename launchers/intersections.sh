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
dt-exec roslaunch intersection_control intersections.launch test:=false


# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
