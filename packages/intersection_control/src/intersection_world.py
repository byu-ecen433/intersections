"""
ECEN 433 - Lab 6
================

Geometry of the simulated intersection, shared by intersection_sim_node and
intersection_graph.

World frame: origin at the center of the intersection tile, x east, y north.
The robot always arrives heading east in the southern (right-hand) lane, so its
stop line is on the west edge of the tile.
"""

import math

TILE = 0.585          # tile side (m)
LANE_OFFSET = 0.128   # road center line to lane center (m)
WHITE_WIDTH = 0.048
YELLOW_WIDTH = 0.024
RED_WIDTH = 0.048

HALF = TILE / 2
STOP_X = -HALF - RED_WIDTH / 2   # center of our stop line

# Roads leaving the tile: name -> (lane center point, heading). Each lane frame
# has x down the lane and y to its left, like the lane filter's d.
ROADS = {
    "straight": ((0.0, -LANE_OFFSET), 0.0),
    "left": ((LANE_OFFSET, 0.0), math.pi / 2),
    "right": ((-LANE_OFFSET, 0.0), -math.pi / 2),
}
APPROACH = ((0.0, -LANE_OFFSET), 0.0)

# Which roads exist at each sign type, from the robot's point of view.
EXITS = {
    "4-way-intersect": ("left", "straight", "right"),
    "T-intersection": ("left", "right"),
    "left-T-intersect": ("left", "straight"),
    "right-T-intersect": ("straight", "right"),
}

# A tag from each range in apriltagsDB.yaml, plus a stop sign as a decoy.
SIGN_TAG_IDS = {
    "4-way-intersect": 15,
    "T-intersection": 66,
    "left-T-intersect": 62,
    "right-T-intersect": 58,
}
STOP_TAG_ID = 25

# Both signs stand on the right-hand corner by the stop line, facing west.
# (x, y, height of tag center), m.
SIGN_POS = (STOP_X + 0.02, -HALF - 0.04, 0.13)
STOP_SIGN_POS = (STOP_X - 0.06, -HALF - 0.04, 0.13)
SIGN_FACING = math.pi   # direction the tag face points

# Duckiebot footprint relative to the wheel axle (m): (forward, left).
ROBOT_OUTLINE = [(-0.05, -0.065), (0.10, -0.065), (0.14, 0.0), (0.10, 0.065), (-0.05, 0.065)]


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def to_lane(lane, x, y, theta):
    """(s, d, phi) of a world pose in a lane frame."""
    (px, py), h = lane
    c, s = math.cos(h), math.sin(h)
    dx, dy = x - px, y - py
    return dx * c + dy * s, -dx * s + dy * c, wrap(theta - h)


def region(x, y):
    """Which part of the map a point is on: 'approach', a ROADS key, 'tile' or 'off'."""
    inside_x, inside_y = abs(x) <= HALF, abs(y) <= HALF
    if inside_x and inside_y:
        return "tile"
    if x < -HALF and inside_y:
        return "approach"
    if x > HALF and inside_y:
        return "straight"
    if y > HALF and inside_x:
        return "left"
    if y < -HALF and inside_x:
        return "right"
    return "off"
