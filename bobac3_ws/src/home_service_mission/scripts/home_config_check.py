#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys

import rospy
import yaml


def main():
    rospy.init_node("home_config_check", anonymous=True)
    config_file = rospy.get_param(
        "~config_file",
        "/home/yuanbao/bobac3_ws/src/home_service_mission/config/home_waypoints.yaml",
    )
    if not os.path.isfile(config_file):
        print("HOME_CONFIG_NOT_READY missing_file=" + config_file)
        return 2
    with open(config_file, "r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    missing = []
    for name, point in config["waypoints"].items():
        if point.get("x") is None or point.get("y") is None or point.get("yaw") is None:
            missing.append(name)
    if missing:
        print("HOME_CONFIG_NOT_READY missing_points=" + ",".join(missing))
        return 2
    print("HOME_CONFIG_OK points=" + ",".join(config["waypoints"].keys()))
    return 0


if __name__ == "__main__":
    sys.exit(main())
