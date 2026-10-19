#!/usr/bin/env python3
"""
ECEN 433 - Lab 6
================

Turns on the Duckietown nodes that start switched off on the robot:

    stop_line_filter_node     (part of your Lab 4 lane_following stack)
    apriltag_detector_node    (started by intersections.launch)

Each one has a ~switch service (std_srvs/SetBool). This node waits for each
service to come up, calls it with data: true, and then exits. You do not
need to change it.
"""

import rospy
from std_srvs.srv import SetBool

SERVICES = [
    "stop_line_filter_node/switch",
    "apriltag_detector_node/switch",
]


def switch_on(name):
    while not rospy.is_shutdown():
        try:
            rospy.wait_for_service(name, timeout=2.0)
            rospy.ServiceProxy(name, SetBool)(True)
            rospy.loginfo("switched on %s", rospy.resolve_name(name))
            return
        except (rospy.ROSException, rospy.ServiceException):
            rospy.logwarn("waiting for %s", rospy.resolve_name(name))


if __name__ == "__main__":
    rospy.init_node("switch_on_node")
    try:
        for name in SERVICES:
            switch_on(name)
    except rospy.ROSInterruptException:
        pass
