#!/usr/bin/env python3
"""
ECEN 433 - Lab 6
================

Live bird's-eye view of the simulated intersection. The robot's true path is
colored by fsm_node/mode, so you can see where each of your states took over.

If your intersection_node publishes its turn goal on ~goal (a Pose2D in your
odometry frame), it is drawn as a black arrow. The arrow lands where that goal
really is, odometry drift included.
"""

import math
import threading

import matplotlib
import rospy
from duckietown_msgs.msg import FSMState
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String

import intersection_world as W

ROAD_LENGTH = 0.7
APPROACH_LENGTH = 1.4
MODE_COLORS = {"LANE_FOLLOWING": "tab:blue", "STOP": "tab:red",
               "INTERSECTION_CONTROL": "tab:green"}


def compose(a, b):
    """Pose b, given in frame a, expressed in a's parent frame. Poses are (x, y, theta)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2])


class IntersectionGraph:
    def __init__(self):
        self.lock = threading.Lock()
        self.mode = "?"
        self.label = ""
        self.episode = 0
        self.segments = []   # [mode, xs, ys]
        self.true_pose = None
        self.odom_pose = None
        self.goal = None     # in the world frame
        rospy.Subscriber("intersection_sim_node/true_pose", Pose2D, self.true_cb, queue_size=10)
        rospy.Subscriber("intersection_sim_node/intersection", String, self.episode_cb)
        rospy.Subscriber("fsm_node/mode", FSMState, self.mode_cb, queue_size=10)
        rospy.Subscriber("pose", Pose2D, self.odom_cb, queue_size=10)
        rospy.Subscriber("intersection_node/goal", Pose2D, self.goal_cb, queue_size=10)

    def true_cb(self, msg):
        with self.lock:
            if math.isnan(msg.x):
                self.segments = []
                self.goal = None
                self.true_pose = None
                return
            self.true_pose = (msg.x, msg.y, msg.theta)
            if not self.segments or self.segments[-1][0] != self.mode:
                start = self.segments[-1] if self.segments else None
                self.segments.append([self.mode, [], []])
                if start:   # join the colors up
                    self.segments[-1][1].append(start[1][-1])
                    self.segments[-1][2].append(start[2][-1])
            self.segments[-1][1].append(msg.x)
            self.segments[-1][2].append(msg.y)

    def episode_cb(self, msg):
        with self.lock:
            n, sign_type, expected = msg.data.split()
            self.episode = int(n)
            self.sign_type = sign_type
            self.label = f"Intersection {n}: {sign_type}, expected turn: {expected}"

    def mode_cb(self, msg):
        with self.lock:
            self.mode = msg.state

    def odom_cb(self, msg):
        with self.lock:
            self.odom_pose = (msg.x, msg.y, msg.theta)

    def goal_cb(self, msg):
        with self.lock:
            if self.true_pose is None or self.odom_pose is None:
                return
            world_from_odom = compose(self.true_pose, inverse(self.odom_pose))
            self.goal = compose(world_from_odom, (msg.x, msg.y, msg.theta))


def draw_map(ax, sign_type):
    from matplotlib.patches import Rectangle
    ax.clear()
    ax.set_facecolor("0.85")
    H = W.HALF
    road = dict(color="0.3", zorder=0)
    ax.add_patch(Rectangle((-H, -H), W.TILE, W.TILE, **road))
    ax.add_patch(Rectangle((-H - APPROACH_LENGTH, -H), APPROACH_LENGTH, W.TILE, **road))
    exits = W.EXITS[sign_type]
    if "straight" in exits:
        ax.add_patch(Rectangle((H, -H), ROAD_LENGTH, W.TILE, **road))
    if "left" in exits:
        ax.add_patch(Rectangle((-H, H), W.TILE, ROAD_LENGTH, **road))
    if "right" in exits:
        ax.add_patch(Rectangle((-H, -H - ROAD_LENGTH), W.TILE, ROAD_LENGTH, **road))

    def lines(x0, x1, y0, y1, horizontal):
        """White edges and dashed yellow center of a straight road piece."""
        if horizontal:
            for y in (y0, y1 - W.WHITE_WIDTH):
                ax.add_patch(Rectangle((x0, y), x1 - x0, W.WHITE_WIDTH, color="white", zorder=1))
            ax.plot([x0, x1], [0, 0], color="gold", linewidth=2, linestyle=(0, (2, 2)), zorder=1)
        else:
            for x in (x0, x1 - W.WHITE_WIDTH):
                ax.add_patch(Rectangle((x, y0), W.WHITE_WIDTH, y1 - y0, color="white", zorder=1))
            ax.plot([0, 0], [y0, y1], color="gold", linewidth=2, linestyle=(0, (2, 2)), zorder=1)

    lines(-H - APPROACH_LENGTH, -H, -H, H, True)
    if "straight" in exits:
        lines(H, H + ROAD_LENGTH, -H, H, True)
    if "left" in exits:
        lines(-H, H, H, H + ROAD_LENGTH, False)
    if "right" in exits:
        lines(-H, H, -H - ROAD_LENGTH, -H, False)
    ax.add_patch(Rectangle((W.STOP_X - W.RED_WIDTH / 2, -H), W.RED_WIDTH, H,
                           color="tab:red", zorder=2))

    sx, sy, _ = W.SIGN_POS
    ax.plot(sx, sy, "s", color="k", markersize=7, zorder=4)
    ax.annotate(sign_type, (sx, sy), xytext=(0, -14), textcoords="offset points",
                ha="center", fontsize="small")
    ax.plot(W.STOP_SIGN_POS[0], W.STOP_SIGN_POS[1], "8", color="tab:red", markersize=8, zorder=4)

    ax.set_xlim(-H - APPROACH_LENGTH, H + ROAD_LENGTH)
    ax.set_ylim(-H - ROAD_LENGTH, H + ROAD_LENGTH)
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")


def robot_polygon(x, y, theta):
    c, s = math.cos(theta), math.sin(theta)
    return [(x + fx * c - fy * s, y + fx * s + fy * c) for fx, fy in W.ROBOT_OUTLINE]


def main():
    rospy.init_node("intersection_graph")
    output_file = rospy.get_param("~output_file", "")
    if rospy.get_param("~only_output_to_file", False):
        matplotlib.use("agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    from matplotlib.patches import Polygon

    graph = IntersectionGraph()
    fig, ax = plt.subplots(figsize=(9, 7))
    drawn_episode = None
    artists = []

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        with graph.lock:
            episode, label, mode = graph.episode, graph.label, graph.mode
            sign_type = getattr(graph, "sign_type", None)
            segments = [(m, list(xs), list(ys)) for m, xs, ys in graph.segments]
            pose, goal = graph.true_pose, graph.goal

        if sign_type and episode != drawn_episode:
            draw_map(ax, sign_type)
            handles = [Line2D([], [], color=c, linewidth=3, label=m) for m, c in MODE_COLORS.items()]
            ax.legend(handles=handles, loc="upper left", fontsize="small")
            drawn_episode = episode
            artists = []

        for a in artists:
            a.remove()
        artists = []
        for m, xs, ys in segments:
            artists += ax.plot(xs, ys, color=MODE_COLORS.get(m, "0.5"), linewidth=3, zorder=3)
        if goal is not None:
            gx, gy, gth = goal
            artists.append(ax.arrow(gx, gy, 0.08 * math.cos(gth), 0.08 * math.sin(gth),
                                    width=0.012, color="k", zorder=6))
        if pose is not None:
            robot = Polygon(robot_polygon(*pose), closed=True, facecolor="tab:orange",
                            edgecolor="k", zorder=5)
            ax.add_patch(robot)
            artists.append(robot)
        ax.set_title(f"{label}\nmode: {mode}")

        if output_file:
            fig.savefig(output_file)
        plt.pause(0.05)
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
