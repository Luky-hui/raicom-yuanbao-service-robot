#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import os
import time

import actionlib
import rospy
import yaml
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Bool, String

from face_rec.msg import face_results
from raicom_vision.srv import DetectObjects, DetectObjectsRequest
from robot_audio.srv import robot_tts, robot_ttsRequest


class HomeServiceMission:
    def __init__(self):
        rospy.init_node("home_service_mission")
        self.config_file = rospy.get_param("~config_file", "")
        self.mission_type = rospy.get_param("~mission_type", "auto")
        self.face_mode = rospy.get_param("~face_mode", "any")
        self.voice_mode = rospy.get_param("~voice_mode", "topic")
        self.target_object = rospy.get_param("~target_object", "")
        self.config = self.load_config(self.config_file)

        interaction = self.config["interaction"]
        self.last_face = None
        self.last_command = ""
        self.last_intent = None
        self.emergency_stop = False

        self.face_sub = rospy.Subscriber(
            interaction["face_topic"], face_results, self.face_callback, queue_size=10
        )
        self.command_sub = rospy.Subscriber(
            interaction["command_topic"], String, self.command_callback, queue_size=10
        )
        self.intent_sub = rospy.Subscriber(
            interaction["intent_topic"], String, self.intent_callback, queue_size=10
        )
        self.stop_sub = rospy.Subscriber(
            "/raicom/emergency_stop", Bool, self.stop_callback, queue_size=10
        )
        self.status_pub = rospy.Publisher(
            "/home_service_mission/status", String, queue_size=10, latch=True
        )
        self.speech_pub = rospy.Publisher(
            "/home_service_mission/speech", String, queue_size=10, latch=True
        )
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        navigation = self.config["navigation"]
        self.action_server_name = navigation["action_server"]
        self.fallback_action_server_name = navigation["fallback_action_server"]
        self.move_base = actionlib.SimpleActionClient(
            self.action_server_name, MoveBaseAction
        )
        self.vision_client = rospy.ServiceProxy(
            "/raicom_vision/detect", DetectObjects
        )
        self.tts_client = rospy.ServiceProxy(
            interaction["tts_service"], robot_tts
        )

    @staticmethod
    def load_config(path):
        if not path or not os.path.isfile(path):
            raise RuntimeError("Home mission config file not found: %s" % path)
        with open(path, "r", encoding="utf-8") as stream:
            return yaml.safe_load(stream)

    def face_callback(self, message):
        self.last_face = message

    def command_callback(self, message):
        self.last_command = message.data.strip()

    def intent_callback(self, message):
        try:
            data = json.loads(message.data)
        except Exception as exc:
            rospy.logwarn("Invalid local voice intent JSON: %s", exc)
            return
        self.last_intent = data

    def stop_callback(self, message):
        self.emergency_stop = bool(message.data)
        if self.emergency_stop:
            self.move_base.cancel_all_goals()
            self.cmd_vel_pub.publish(Twist())
            self.publish_status("emergency_stop")

    def publish_status(self, state, **details):
        message = {"state": state, "mission": self.mission_type}
        message.update(details)
        self.status_pub.publish(
            String(json.dumps(message, ensure_ascii=False, sort_keys=True))
        )
        rospy.loginfo("HOME_STATUS %s", message)

    def check_stopped(self):
        if self.emergency_stop:
            raise RuntimeError("Emergency stop is active")

    def run(self):
        self.validate_waypoints()
        self.connect_navigation()
        self.publish_status("started")
        self.go_to("corridor")
        self.wait_for_person()
        self.speak(self.config["interaction"]["wake_prompt"])

        selected_mission = self.mission_type
        if selected_mission == "auto":
            selected_mission, selected_target = self.wait_for_mission_intent()
            if selected_target:
                self.target_object = selected_target
        if selected_mission not in ("guide", "assistant", "find_object"):
            raise RuntimeError("Unknown home mission type: %s" % selected_mission)
        self.mission_type = selected_mission
        self.publish_status("mission_selected", selected_mission=selected_mission)

        if selected_mission == "guide":
            self.run_guide()
        elif selected_mission == "assistant":
            self.run_assistant()
        else:
            self.run_find_object()

        if self.config["navigation"]["return_to_start"]:
            self.go_to("start")
        self.publish_status("completed")

    def validate_waypoints(self):
        required = ["start", "corridor", "restaurant", "kitchen", "living_room", "bedroom"]
        missing = []
        for name in required:
            point = self.config["waypoints"].get(name)
            if not point:
                missing.append(name)
                continue
            if point.get("x") is None or point.get("y") is None or point.get("yaw") is None:
                missing.append(name)
        if missing:
            raise RuntimeError(
                "Home waypoints are not calibrated: %s. Run capture_home_waypoint.py for each point."
                % ", ".join(missing)
            )

    def connect_navigation(self):
        wait_seconds = float(self.config["navigation"]["server_wait"])
        if self.move_base.wait_for_server(rospy.Duration(wait_seconds)):
            return
        self.move_base = actionlib.SimpleActionClient(
            self.fallback_action_server_name, MoveBaseAction
        )
        if not self.move_base.wait_for_server(rospy.Duration(wait_seconds)):
            raise RuntimeError(
                "Navigation action servers unavailable: %s, %s"
                % (self.action_server_name, self.fallback_action_server_name)
            )

    def wait_for_mission_intent(self):
        self.publish_status("waiting_for_mission_intent")
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.check_stopped()
            intent = self.last_intent
            if intent and intent.get("name") == "home_service_mission":
                slots = dict(zip(intent.get("slots_name", []), intent.get("slots_value", [])))
                mission = slots.get("mission", "")
                target = slots.get("target", "")
                self.last_intent = None
                return mission, target
            rate.sleep()
        raise rospy.ROSInterruptException()

    def wait_for_person(self):
        if self.face_mode == "skip":
            return
        self.publish_status("waiting_for_person")
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.check_stopped()
            if self.last_face and self.last_face.face_data:
                return
            rate.sleep()
        raise rospy.ROSInterruptException()

    def wait_for_command(self, phrases):
        if self.voice_mode == "skip":
            return phrases[0]
        self.publish_status("waiting_for_command", phrases=phrases)
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.check_stopped()
            if self.last_command and any(phrase in self.last_command for phrase in phrases):
                command = self.last_command
                self.last_command = ""
                return command
            rate.sleep()
        raise rospy.ROSInterruptException()

    def run_guide(self):
        guide = self.config["guide"]
        self.wait_for_command(guide["trigger_phrases"])
        self.speak(self.config["interaction"]["accept"])
        for room in guide["route"]:
            self.go_to(room)
            self.speak(guide["introductions"][room])

    def run_assistant(self):
        assistant = self.config["assistant"]
        self.wait_for_command(assistant["trigger_phrases"])
        self.speak("好的，我去看看冰箱里有什么。")
        self.go_to("kitchen")
        foods = self.detect_labels(
            assistant["food_labels"],
            assistant["image_topic"],
            float(assistant["confidence"]),
            int(assistant["sample_count"]),
            float(assistant["sample_interval"]),
        )
        if foods:
            food_names = [self.config["chinese_names"][label] for label in foods]
            self.speak("冰箱里有" + "、".join(food_names) + "。")
        else:
            self.speak("没有识别到冰箱里的食材。")
        recipe_name, recipe_method = self.choose_recipe(foods)
        self.speak("推荐做%s。%s" % (recipe_name, recipe_method))

    def run_find_object(self):
        find_config = self.config["find_object"]
        target = self.target_object
        if not target:
            target = self.wait_for_find_target()
        if target not in find_config["labels"]:
            raise RuntimeError("Unsupported target object: %s" % target)
        target_cn = self.config["chinese_names"][target]
        self.speak("好的，我去找%s。" % target_cn)

        found = False
        for room in find_config["route"]:
            self.go_to(room)
            labels = self.detect_labels(
                find_config["labels"],
                find_config["image_topic"],
                float(find_config["confidence"]),
                int(find_config["sample_count"]),
                float(find_config["sample_interval"]),
            )
            if target in labels:
                self.speak("找到啦，在这里！")
                self.publish_status("target_found", target=target, room=room)
                found = True
                break
            other_labels = [label for label in labels if label != target]
            if other_labels:
                self.speak(
                    "这里找到了"
                    + "、".join(self.config["chinese_names"][label] for label in other_labels)
                )
            else:
                self.speak("这里什么都没有")
        if not found:
            self.speak("没有找到%s。" % target_cn)
            self.publish_status("target_not_found", target=target)

    def wait_for_find_target(self):
        self.publish_status("waiting_for_find_target")
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.check_stopped()
            intent = self.last_intent
            if intent and intent.get("name") == "home_service_mission":
                slots = dict(zip(intent.get("slots_name", []), intent.get("slots_value", [])))
                if slots.get("mission") == "find_object" and slots.get("target"):
                    self.last_intent = None
                    return slots["target"]
            rate.sleep()
        raise rospy.ROSInterruptException()

    def detect_labels(self, labels, image_topic, confidence, sample_count, sample_interval):
        best_scores = {}
        successful_samples = 0
        for index in range(sample_count):
            self.check_stopped()
            request = DetectObjectsRequest()
            request.image_topic = image_topic
            request.labels = labels
            request.confidence = confidence
            try:
                response = self.vision_client(request)
            except Exception as exc:
                rospy.logwarn("Vision request failed: %s", exc)
                continue
            if not response.success:
                rospy.logwarn("Vision request failed: %s", response.message)
                continue
            successful_samples += 1
            for label, score in zip(response.labels, response.confidences):
                best_scores[label] = max(best_scores.get(label, 0.0), float(score))
            if index + 1 < sample_count:
                time.sleep(sample_interval)
        if successful_samples == 0:
            raise RuntimeError("No successful vision samples")
        ordered = sorted(best_scores, key=lambda label: (-best_scores[label], labels.index(label)))
        self.publish_status("vision_result", labels=ordered, scores=best_scores)
        return ordered

    @staticmethod
    def choose_recipe(foods):
        available = set(foods)
        recipes = [
            ({"番茄", "鸡蛋"}, "番茄炒蛋", "番茄切块，鸡蛋打散炒熟盛出，再炒番茄出汁，倒回鸡蛋翻炒调味。"),
            ({"青椒", "猪肉"}, "青椒炒肉", "青椒切丝，猪肉切片腌制，先炒肉片，再放青椒炒熟并调味。"),
            ({"上海青", "豆腐"}, "上海青豆腐汤", "上海青洗净切段，豆腐切块，水开后加入豆腐和上海青煮熟调味。"),
            ({"黄瓜", "虾"}, "黄瓜炒虾仁", "虾仁腌制后炒至变色，加入黄瓜片快速翻炒并调味。"),
            ({"土豆", "猪肉"}, "土豆烧肉", "猪肉煸香后加入土豆块，加水焖熟并调味。"),
            ({"番茄", "豆腐"}, "番茄豆腐", "番茄炒出汤汁，加入豆腐块小火煮入味。"),
            ({"玉米", "猪肉"}, "玉米炒肉末", "玉米粒焯水，猪肉剁末炒香，加入玉米粒翻炒并调味。"),
            ({"茄子"}, "红烧茄子", "茄子切块煎软，加入调味汁烧至入味。"),
            ({"鱼肉"}, "清蒸鱼肉", "鱼肉处理干净后加葱姜蒸熟，淋上蒸鱼豉油。"),
            ({"虾"}, "清炒虾仁", "虾仁腌制后快速滑炒，炒熟后调味。"),
            ({"苹果", "香蕉"}, "水果拼盘", "苹果和香蕉切块，摆盘后即可食用。"),
        ]
        for required, name, method in recipes:
            if required.issubset(available):
                return name, method
        if foods:
            return "清爽拼盘", "将现有食材清洗处理，按食材特性煮熟或直接切配后组合。"
        return "家常面", "准备面条和基础调味料，将面条煮熟后调味。"

    def go_to(self, name):
        self.check_stopped()
        point = self.config["waypoints"][name]
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = point["frame_id"]
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = float(point["x"])
        goal.target_pose.pose.position.y = float(point["y"])
        goal.target_pose.pose.orientation = self.yaw_to_quaternion(float(point["yaw"]))
        self.publish_status("navigating", waypoint=name)
        self.move_base.send_goal(goal)
        timeout = float(self.config["navigation"]["goal_timeout"])
        if not self.move_base.wait_for_result(rospy.Duration(timeout)):
            self.move_base.cancel_goal()
            raise RuntimeError("Navigation timeout at waypoint: %s" % name)
        state = self.move_base.get_state()
        if state != GoalStatus.SUCCEEDED:
            raise RuntimeError("Navigation failed at %s, state=%s" % (name, state))
        self.publish_status("arrived", waypoint=name)

    def speak(self, text):
        self.check_stopped()
        self.speech_pub.publish(String(text))
        rospy.loginfo("TTS: %s", text)
        request = robot_ttsRequest()
        request.text = text
        request.play = True
        try:
            self.tts_client(request)
        except Exception as exc:
            rospy.logwarn("TTS service unavailable: %s", exc)

    @staticmethod
    def yaw_to_quaternion(yaw):
        return Quaternion(0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


if __name__ == "__main__":
    try:
        HomeServiceMission().run()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        rospy.logerr("Home mission failed: %s", exc)
        raise
