"""
ECEN 433 - Lab 6
================

Duckietown's AprilTag database: which sign each tag ID is printed on.

    from tag_db import load_sign_types
    signs = load_sign_types()
    signs.get(58)   # 'right-T-intersect'

Tags that are not traffic signs (localization, vehicles, street names) are
left out, so .get() returns None for them.

To import this from your own node, put your node in this same src/ folder.
"""

import os

import rospkg
import yaml


def load_sign_types():
    path = os.path.join(rospkg.RosPack().get_path("apriltag"), "config", "apriltagsDB.yaml")
    with open(path) as f:
        entries = yaml.safe_load(f)
    return {e["tag_id"]: e["traffic_sign_type"] for e in entries if e.get("traffic_sign_type")}
