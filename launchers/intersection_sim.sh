#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------

# Against the intersection simulator, with a live plot. Run it on your laptop:
#
#     dts devel run -X -L intersection_sim
#
dt-exec roslaunch intersection_control intersections.launch test:=true

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
