#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import os
import random

import rospkg
import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import DeleteModel, SetModelState, SpawnModel
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler


STATIC_OBSTACLE_SLOTS = [
    {"name": "static_slot_1", "x": 0.50, "y": 0.48, "z": 0.10, "yaw": 0.0},
    {"name": "static_slot_2", "x": 0.58, "y": 0.52, "z": 0.10, "yaw": 0.0},
    {"name": "static_slot_3", "x": 0.66, "y": 0.56, "z": 0.10, "yaw": 0.0},
    {"name": "static_slot_4", "x": 0.72, "y": 0.46, "z": 0.10, "yaw": 0.0},
    {"name": "static_slot_5", "x": 0.76, "y": 0.62, "z": 0.10, "yaw": 0.0},
]

DYNAMIC_OBSTACLE_SLOTS = [
    {"name": "dynamic_slot_1", "x": 0.94, "y": 0.78, "z": 0.20, "yaw": 0.0, "axis": "x", "amplitude": 0.16},
    {"name": "dynamic_slot_2", "x": 1.00, "y": 0.80, "z": 0.20, "yaw": 0.0, "axis": "x", "amplitude": 0.16},
    {"name": "dynamic_slot_3", "x": 1.06, "y": 0.82, "z": 0.20, "yaw": 0.0, "axis": "x", "amplitude": 0.14},
]

STATIC_MODEL_NAME = "random_red_static_obstacle"
DYNAMIC_MODEL_NAME = "random_moving_cylinder_obstacle"


def pose_from_slot(slot, offset=0.0):
    pose = Pose()
    axis = slot.get("axis", "x")
    pose.position.x = float(slot["x"]) + (offset if axis == "x" else 0.0)
    pose.position.y = float(slot["y"]) + (offset if axis == "y" else 0.0)
    pose.position.z = float(slot["z"])
    qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, float(slot.get("yaw", 0.0)))
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose


def delete_existing(delete_model):
    for name in ["cylinder_obstacle", STATIC_MODEL_NAME, DYNAMIC_MODEL_NAME]:
        try:
            delete_model(name)
        except Exception:
            pass


def read_text(path):
    with open(path, "r", encoding="utf-8") as stream:
        return stream.read()


def spawn_model(spawn_model_client, model_name, sdf_xml, pose):
    response = spawn_model_client(model_name, sdf_xml, "", pose, "world")
    if not response.success:
        raise RuntimeError("%s: %s" % (model_name, response.status_message))


def choose_slot(slots, seed, salt):
    rng = random.Random(str(seed) + ":" + salt) if seed != "" else random.Random()
    return rng.choice(list(slots))


def move_dynamic_obstacle(set_model_state, slot, speed_hz):
    rate = rospy.Rate(20.0)
    started = rospy.Time.now().to_sec()
    amplitude = float(slot.get("amplitude", 0.25))
    while not rospy.is_shutdown():
        elapsed = rospy.Time.now().to_sec() - started
        offset = amplitude * math.sin(2.0 * math.pi * speed_hz * elapsed)
        state = ModelState()
        state.model_name = DYNAMIC_MODEL_NAME
        state.reference_frame = "world"
        state.pose = pose_from_slot(slot, offset)
        set_model_state(state)
        rate.sleep()


def main():
    rospy.init_node("spawn_random_obstacles")
    seed = rospy.get_param("~obstacle_seed", "")
    enable_static = bool(rospy.get_param("~enable_static_obstacle", True))
    enable_dynamic = bool(rospy.get_param("~enable_dynamic_obstacle", True))
    dynamic_speed_hz = float(rospy.get_param("~dynamic_speed_hz", 0.10))

    package_path = rospkg.RosPack().get_path("bobac3_description")
    static_sdf_path = rospy.get_param(
        "~static_obstacle_sdf",
        os.path.join(package_path, "xacro", "red_static_obstacle.sdf"),
    )
    dynamic_sdf_path = rospy.get_param(
        "~dynamic_obstacle_sdf",
        os.path.join(package_path, "xacro", "model.sdf"),
    )

    rospy.wait_for_service("/gazebo/delete_model", timeout=20.0)
    rospy.wait_for_service("/gazebo/spawn_sdf_model", timeout=20.0)
    rospy.wait_for_service("/gazebo/set_model_state", timeout=20.0)
    delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
    spawn_model_client = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
    set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

    delete_existing(delete_model)
    result = {"seed": seed, "static": None, "dynamic": None}

    if enable_static:
        static_slot = choose_slot(STATIC_OBSTACLE_SLOTS, seed, "static")
        spawn_model(spawn_model_client, STATIC_MODEL_NAME, read_text(static_sdf_path), pose_from_slot(static_slot))
        result["static"] = static_slot
        rospy.loginfo(
            "Random red static obstacle at slot=%s x=%.3f y=%.3f",
            static_slot["name"],
            float(static_slot["x"]),
            float(static_slot["y"]),
        )

    if enable_dynamic:
        dynamic_slot = choose_slot(DYNAMIC_OBSTACLE_SLOTS, seed, "dynamic")
        spawn_model(spawn_model_client, DYNAMIC_MODEL_NAME, read_text(dynamic_sdf_path), pose_from_slot(dynamic_slot))
        result["dynamic"] = dynamic_slot
        rospy.loginfo(
            "Random moving cylinder obstacle at slot=%s x=%.3f y=%.3f axis=%s amplitude=%.3f speed_hz=%.3f",
            dynamic_slot["name"],
            float(dynamic_slot["x"]),
            float(dynamic_slot["y"]),
            dynamic_slot.get("axis", "x"),
            float(dynamic_slot.get("amplitude", 0.25)),
            dynamic_speed_hz,
        )
        rospy.set_param("/raicom/random_obstacles", result)
        rospy.loginfo("Random obstacles spawned: %s", json.dumps(result, ensure_ascii=False, sort_keys=True))
        move_dynamic_obstacle(set_model_state, dynamic_slot, dynamic_speed_hz)
    else:
        rospy.set_param("/raicom/random_obstacles", result)
        rospy.loginfo("Random obstacles spawned: %s", json.dumps(result, ensure_ascii=False, sort_keys=True))


if __name__ == "__main__":
    main()
