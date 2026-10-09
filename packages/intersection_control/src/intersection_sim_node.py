#!/usr/bin/env python3
"""
ECEN 433 - Lab 6
================

A simulated Duckiebot driving up to one intersection after another.

It stands in for everything on the robot that is not yours, on the same
relative topics, so your nodes cannot tell the two apart:

    publishes   left/right_wheel_encoder_driver_node/tick   (cumulative ticks)
                lane_filter_node/lane_pose
                stop_line_filter_node/stop_line_reading
                apriltag_detector_node/detections            (2 Hz, like the real one)
                fsm_node/mode                                 (LANE_FOLLOWING once, = pressing f)
    drives on   car_cmd_switch_node/cmd

The real car_cmd_switch_node runs next to it, so what drives the robot is
decided by fsm_node/mode exactly as on the bot.

Each episode starts the robot about a meter before a stop line. The episode
ends once the robot is `exit_check` m down an exit road (or off the road, or
out of time). Then it is logged and the robot is put in front of the next
intersection in `types`. The log also checks the turn against `route`, the
left/straight/right sequence with illegal turns skipped.
"""

import math
import random
from collections import deque

import rospy
from duckietown_msgs.msg import (AprilTagDetection, AprilTagDetectionArray, FSMState,
                                 LanePose, StopLineReading, Twist2DStamped,
                                 WheelEncoderStamped)
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Header, String

import intersection_world as W

DEFAULT_TYPES = ["4-way-intersect", "T-intersection", "right-T-intersect", "left-T-intersect"]

# Must match the Lab 3 odometry: 135 ticks per wheel turn, R, half the baseline.
TICKS_PER_REV = 135
WHEEL_RADIUS = 0.0318
HALF_BASELINE = 0.05

CAM_FORWARD = 0.10   # camera ahead of the wheel axle (m)
CAM_HEIGHT = 0.10


class Episode:
    """What happened at one intersection."""

    def __init__(self, index, sign_type, expected, start_time):
        self.index = index
        self.sign_type = sign_type
        self.expected = expected
        self.start_time = start_time
        self.stop_gap = None   # wheel axle to stop line center while in STOP, as stop_line_filter measures it
        self.turn_start = None
        self.result = None

    def log(self, d_tol, phi_tol):
        took, s, d, phi, why = self.result
        stop = ("never stopped" if self.stop_gap is None
                else f"stopped {self.stop_gap * 100:.0f} cm before the line")
        if why:
            ok = False
            exit_txt = why
        else:
            in_lane = abs(d) < d_tol and abs(phi) < phi_tol
            ok = in_lane and took == self.expected and self.stop_gap is not None
            exit_txt = f"exited {took}, d={d:+.3f} m, phi={phi:+.2f} rad"
        expected = f"expected {self.expected}"
        log = rospy.loginfo if ok else rospy.logwarn
        log("intersection %d (%s): %s, %s (%s) -> %s",
            self.index, self.sign_type, stop, exit_txt, expected, "PASS" if ok else "FAIL")
        return ok


class IntersectionSimNode:
    def __init__(self):
        rospy.init_node("intersection_sim_node")
        self.rate_hz = rospy.get_param("~rate", 30.0)
        self.types = rospy.get_param("~types", DEFAULT_TYPES)
        self.route = rospy.get_param("~route", ["left", "straight", "right"])
        self.start_distance = rospy.get_param("~start_distance", 1.0)
        self.delay = rospy.get_param("~delay", 0.1)
        self.v_max = rospy.get_param("~v_max", 0.6)
        self.omega_max = rospy.get_param("~omega_max", 8.0)
        self.right_radius_error = rospy.get_param("~right_radius_error", 0.01)
        self.noise_d = rospy.get_param("~noise_d", 0.01)
        self.noise_phi = rospy.get_param("~noise_phi", 0.05)
        self.noise_stop = rospy.get_param("~noise_stop", 0.01)
        self.stop_distance = rospy.get_param("~stop_distance", 0.12)
        self.stop_line_range = rospy.get_param("~stop_line_range", [0.08, 0.6])
        self.tag_range = rospy.get_param("~tag_range", 0.9)
        self.tag_fov = math.radians(rospy.get_param("~tag_fov_deg", 120.0))
        self.tag_rate = rospy.get_param("~tag_rate", 2.0)
        self.tag_miss = rospy.get_param("~tag_miss", 0.1)
        self.exit_check = rospy.get_param("~exit_check", 0.4)
        self.exit_d_tol = rospy.get_param("~exit_d_tol", 0.06)
        self.exit_phi_tol = rospy.get_param("~exit_phi_tol", 0.3)
        self.timeout = rospy.get_param("~episode_timeout", 40.0)

        self.x = self.y = self.theta = 0.0
        self.ticks = [0.0, 0.0]
        self.mode = None
        self.started = False
        self.route_index = 0
        self.episode = None
        self.n_episodes = 0
        self.n_passed = 0
        self.step_count = 0
        delay_steps = max(0, int(round(self.delay * self.rate_hz)))
        self.latest_cmd = (0.0, 0.0)
        self.cmd_queue = deque([(0.0, 0.0)] * (delay_steps + 1), maxlen=delay_steps + 1)

        self.pub_left = rospy.Publisher("left_wheel_encoder_driver_node/tick",
                                        WheelEncoderStamped, queue_size=10)
        self.pub_right = rospy.Publisher("right_wheel_encoder_driver_node/tick",
                                         WheelEncoderStamped, queue_size=10)
        self.pub_lane = rospy.Publisher("lane_filter_node/lane_pose", LanePose, queue_size=1)
        self.pub_stop = rospy.Publisher("stop_line_filter_node/stop_line_reading",
                                        StopLineReading, queue_size=1)
        self.pub_tags = rospy.Publisher("apriltag_detector_node/detections",
                                        AprilTagDetectionArray, queue_size=1)
        self.pub_mode = rospy.Publisher("fsm_node/mode", FSMState, queue_size=1, latch=True)
        self.pub_true = rospy.Publisher("~true_pose", Pose2D, queue_size=10)
        self.pub_type = rospy.Publisher("~intersection", String, queue_size=1, latch=True)
        rospy.Subscriber("car_cmd_switch_node/cmd", Twist2DStamped, self.cmd_cb, queue_size=1)
        rospy.Subscriber("fsm_node/mode", FSMState, self.mode_cb, queue_size=10)

        self.new_episode()
        rospy.sleep(1.0)   # let car_cmd_switch_node connect before it hears the mode
        self.pub_mode.publish(FSMState(header=self.stamp(), state="LANE_FOLLOWING"))
        rospy.loginfo("intersection_sim_node: mode LANE_FOLLOWING, waiting for commands on %s",
                      rospy.resolve_name("car_cmd_switch_node/cmd"))

    def stamp(self):
        return Header(stamp=rospy.Time.now())

    def cmd_cb(self, msg):
        v = max(-self.v_max, min(self.v_max, msg.v))
        omega = max(-self.omega_max, min(self.omega_max, msg.omega))
        self.latest_cmd = (v, omega)
        if not self.started:
            rospy.loginfo("first command received, starting")
            self.started = True
            self.episode.start_time = rospy.get_time()

    def mode_cb(self, msg):
        self.mode = msg.state
        ep = self.episode
        if ep is not None and msg.state == "INTERSECTION_CONTROL" and ep.turn_start is None:
            ep.turn_start = rospy.get_time()

    def expected_turn(self, sign_type):
        """Next legal entry of `route`, and the route index after it."""
        legal = W.EXITS[sign_type]
        for k in range(len(self.route)):
            i = (self.route_index + k) % len(self.route)
            if self.route[i] in legal:
                return self.route[i], (i + 1) % len(self.route)
        return None, self.route_index

    def new_episode(self):
        sign_type = self.types[self.n_episodes % len(self.types)]
        expected, self.next_route_index = self.expected_turn(sign_type)
        self.n_episodes += 1
        self.episode = Episode(self.n_episodes, sign_type, expected, rospy.get_time())
        self.x = W.STOP_X - self.start_distance
        self.y = -W.LANE_OFFSET + random.uniform(-0.02, 0.02)
        self.theta = random.uniform(-0.1, 0.1)
        self.cmd_queue.extend([(0.0, 0.0)] * self.cmd_queue.maxlen)
        self.pub_true.publish(Pose2D(x=math.nan, y=math.nan, theta=math.nan))
        self.pub_type.publish(String(data=f"{self.n_episodes} {sign_type} {expected}"))

    def end_episode(self, took, s, d, phi, why=""):
        ep = self.episode
        ep.result = (took, s, d, phi, why)
        if ep.log(self.exit_d_tol, self.exit_phi_tol):
            self.n_passed += 1
        # The route advances past whatever was legal, whether or not the robot took it.
        self.route_index = self.next_route_index
        if self.n_episodes % len(self.types) == 0:
            rospy.loginfo("---- %d / %d intersections passed ----", self.n_passed, self.n_episodes)
        self.new_episode()

    def step(self, dt):
        self.cmd_queue.append(self.latest_cmd)
        v, omega = self.cmd_queue[0]
        v_l = v - omega * HALF_BASELINE
        v_r = v + omega * HALF_BASELINE

        # Encoders count wheel rotation exactly. The right wheel is a little
        # bigger than the radius odometry assumes, so odometry drifts.
        self.ticks[0] += v_l * dt / WHEEL_RADIUS * TICKS_PER_REV / (2 * math.pi)
        self.ticks[1] += v_r * dt / WHEEL_RADIUS * TICKS_PER_REV / (2 * math.pi)
        v_r *= 1.0 + self.right_radius_error
        v_true = (v_l + v_r) / 2
        omega_true = (v_r - v_l) / (2 * HALF_BASELINE)
        self.x += v_true * math.cos(self.theta) * dt
        self.y += v_true * math.sin(self.theta) * dt
        self.theta = W.wrap(self.theta + omega_true * dt)

        if not self.started:
            return
        ep = self.episode
        now = rospy.get_time()
        if self.mode == "STOP" and ep.turn_start is None:
            ep.stop_gap = W.STOP_X - self.x
        where = W.region(self.x, self.y)
        if where in W.ROADS:
            s, d, phi = W.to_lane(W.ROADS[where], self.x, self.y, self.theta)
            s -= W.HALF   # measured from the edge of the tile
            if where not in W.EXITS[ep.sign_type]:
                self.end_episode(where, s, d, phi, f"drove onto the {where} road, which doesn't exist")
            elif s >= self.exit_check:
                self.end_episode(where, s, d, phi)
        elif where == "off" or (where == "approach" and self.x < W.STOP_X - self.start_distance - 0.5):
            self.end_episode("nowhere", 0, 0, 0, "drove off the road")
        elif now - ep.start_time > self.timeout:
            self.end_episode("nowhere", 0, 0, 0, f"still not through after {self.timeout:.0f} s")

    def noisy_lane_pose(self):
        """(d, phi, in_lane) as the lane filter would report them here."""
        where = W.region(self.x, self.y)
        if where == "approach":
            lane = W.APPROACH
        elif where in W.ROADS:
            lane = W.ROADS[where]
        else:
            # Inside the tile there are no lane lines to fit, so the real lane
            # filter's output is close to garbage.
            _, d, phi = W.to_lane(W.APPROACH, self.x, self.y, self.theta)
            return d + random.gauss(0, 0.06), phi + random.gauss(0, 0.4), False
        _, d, phi = W.to_lane(lane, self.x, self.y, self.theta)
        return d + random.gauss(0, self.noise_d), phi + random.gauss(0, self.noise_phi), True

    def publish_lane_and_stop_line(self, stamp):
        d, phi, in_lane = self.noisy_lane_pose()
        lane = LanePose(d=d, phi=phi, in_lane=in_lane, status=LanePose.NORMAL)
        lane.header.stamp = stamp
        self.pub_lane.publish(lane)

        # Like stop_line_filter: distance from red segments ahead, (d, phi) from the lane pose.
        reading = StopLineReading()
        reading.header.stamp = stamp
        gap = W.STOP_X - self.x
        near, far = self.stop_line_range
        if W.region(self.x, self.y) == "approach" and near < gap < far and abs(self.theta) < 0.7:
            gap += random.gauss(0, self.noise_stop)
            reading.stop_line_detected = True
            reading.at_stop_line = gap < self.stop_distance
            reading.stop_pose = Pose2D(x=-gap, y=d, theta=phi)
        self.pub_stop.publish(reading)

    def publish_tags(self, stamp):
        msg = AprilTagDetectionArray()
        msg.header.stamp = stamp
        msg.header.frame_id = "camera_optical_frame"
        cx = self.x + CAM_FORWARD * math.cos(self.theta)
        cy = self.y + CAM_FORWARD * math.sin(self.theta)
        signs = [(W.SIGN_TAG_IDS[self.episode.sign_type], W.SIGN_POS), (W.STOP_TAG_ID, W.STOP_SIGN_POS)]
        for tag_id, (tx, ty, tz) in signs:
            dx, dy = tx - cx, ty - cy
            fwd = dx * math.cos(self.theta) + dy * math.sin(self.theta)
            left = -dx * math.sin(self.theta) + dy * math.cos(self.theta)
            facing = abs(W.wrap(math.atan2(-dy, -dx) - W.SIGN_FACING))
            if (fwd <= 0.05 or math.hypot(fwd, left) > self.tag_range
                    or abs(math.atan2(left, fwd)) > self.tag_fov / 2
                    or facing > math.radians(70) or random.random() < self.tag_miss):
                continue
            det = AprilTagDetection(tag_id=tag_id, tag_family="tag36h11")
            # Camera optical frame: x right, y down, z forward.
            det.transform.translation.x = -left + random.gauss(0, 0.01)
            det.transform.translation.y = CAM_HEIGHT - tz + random.gauss(0, 0.005)
            det.transform.translation.z = fwd + random.gauss(0, 0.01)
            yaw = W.wrap(W.SIGN_FACING + math.pi - self.theta)
            det.transform.rotation.y = math.sin(yaw / 2)
            det.transform.rotation.w = math.cos(yaw / 2)
            msg.detections.append(det)
        self.pub_tags.publish(msg)

    def publish_ticks(self, stamp):
        for pub, ticks in ((self.pub_left, self.ticks[0]), (self.pub_right, self.ticks[1])):
            msg = WheelEncoderStamped(data=int(ticks), resolution=TICKS_PER_REV,
                                      type=WheelEncoderStamped.ENCODER_TYPE_INCREMENTAL)
            msg.header.stamp = stamp
            pub.publish(msg)

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        lane_every = max(1, int(round(self.rate_hz / 15.0)))
        tag_every = max(1, int(round(self.rate_hz / self.tag_rate)))
        while not rospy.is_shutdown():
            self.step(1.0 / self.rate_hz)
            stamp = rospy.Time.now()
            self.publish_ticks(stamp)
            if self.step_count % lane_every == 0:
                self.publish_lane_and_stop_line(stamp)
                self.pub_true.publish(Pose2D(x=self.x, y=self.y, theta=self.theta))
            if self.step_count % tag_every == 0:
                self.publish_tags(stamp)
            self.step_count += 1
            rate.sleep()


if __name__ == "__main__":
    try:
        IntersectionSimNode().run()
    except rospy.ROSInterruptException:
        pass
