#!/usr/bin/env python
import json
import os
import zlib

import rospy
import tf2_geometry_msgs
import tf2_ros
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

from qr_map_annotator.msg import QrDetection, QrLandmark


def stable_id(text):
    return zlib.crc32(text.encode("utf-8")) & 0x7FFFFFFF


class QrMapperNode(object):
    def __init__(self):
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.confirm_hits = int(rospy.get_param("~confirm_hits", 2))
        self.one_shot_mode = bool(rospy.get_param("~one_shot_mode", True))
        self.use_latest_tf = bool(rospy.get_param("~use_latest_tf", True))
        self.confidence_min = float(rospy.get_param("~confidence_min", 0.2))
        self.save_landmarks = bool(rospy.get_param("~save_landmarks", True))
        self.landmark_file = rospy.get_param("~landmark_file", os.path.expanduser("~/.ros/qr_landmarks.json"))
        self.detections_topic = rospy.get_param("~detections_topic", "/qr_detector/detections")

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.pending_hits = {}
        self.landmarks = {}

        self.landmark_pub = rospy.Publisher("~landmark", QrLandmark, queue_size=20, latch=True)
        self.marker_pub = rospy.Publisher("~markers", MarkerArray, queue_size=1, latch=True)

        self._load_landmarks()
        self._publish_markers()

        rospy.Subscriber(self.detections_topic, QrDetection, self.detection_callback, queue_size=20)

    def _pose_to_dict(self, pose):
        return {
            "x": pose.position.x,
            "y": pose.position.y,
            "z": pose.position.z,
            "qx": pose.orientation.x,
            "qy": pose.orientation.y,
            "qz": pose.orientation.z,
            "qw": pose.orientation.w,
        }

    def _dict_to_landmark_msg(self, text, data):
        msg = QrLandmark()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.map_frame
        msg.text = text
        msg.pose.position.x = data["x"]
        msg.pose.position.y = data["y"]
        msg.pose.position.z = data["z"]
        msg.pose.orientation.x = data["qx"]
        msg.pose.orientation.y = data["qy"]
        msg.pose.orientation.z = data["qz"]
        msg.pose.orientation.w = data["qw"]
        msg.seen_count = int(data["seen_count"])
        return msg

    def _save_landmarks(self):
        if not self.save_landmarks:
            return
        parent = os.path.dirname(self.landmark_file)
        if parent and not os.path.exists(parent):
            os.makedirs(parent)
        with open(self.landmark_file, "w") as f:
            json.dump(self.landmarks, f, indent=2, sort_keys=True)

    def _load_landmarks(self):
        if not os.path.exists(self.landmark_file):
            return
        try:
            with open(self.landmark_file, "r") as f:
                data = json.load(f)
        except Exception:
            return
        for text, item in data.items():
            self.landmarks[text] = {
                "x": float(item["x"]),
                "y": float(item["y"]),
                "z": float(item["z"]),
                "qx": float(item["qx"]),
                "qy": float(item["qy"]),
                "qz": float(item["qz"]),
                "qw": float(item["qw"]),
                "seen_count": int(item.get("seen_count", 1)),
            }

    def _publish_markers(self):
        array_msg = MarkerArray()
        for text in sorted(self.landmarks.keys()):
            item = self.landmarks[text]
            base = stable_id(text) % 1000000000

            sphere = Marker()
            sphere.header.frame_id = self.map_frame
            sphere.header.stamp = rospy.Time.now()
            sphere.ns = "qr_point"
            sphere.id = base * 2
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = item["x"]
            sphere.pose.position.y = item["y"]
            sphere.pose.position.z = item["z"]
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.18
            sphere.scale.y = 0.18
            sphere.scale.z = 0.18
            sphere.color.a = 1.0
            sphere.color.r = 1.0
            sphere.color.g = 0.0
            sphere.color.b = 0.0
            array_msg.markers.append(sphere)

            text_marker = Marker()
            text_marker.header.frame_id = self.map_frame
            text_marker.header.stamp = rospy.Time.now()
            text_marker.ns = "qr_text"
            text_marker.id = base * 2 + 1
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = item["x"]
            text_marker.pose.position.y = item["y"]
            text_marker.pose.position.z = item["z"] + 0.22
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.16
            text_marker.color.a = 1.0
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.text = text
            array_msg.markers.append(text_marker)

        status_marker = Marker()
        status_marker.header.frame_id = self.map_frame
        status_marker.header.stamp = rospy.Time.now()
        status_marker.ns = "qr_status"
        status_marker.id = 2147483000
        status_marker.type = Marker.TEXT_VIEW_FACING
        status_marker.action = Marker.ADD
        status_marker.pose.position.x = 0.0
        status_marker.pose.position.y = 0.0
        status_marker.pose.position.z = 1.2
        status_marker.pose.orientation.w = 1.0
        status_marker.scale.z = 0.2
        status_marker.color.a = 1.0
        status_marker.color.r = 0.2
        status_marker.color.g = 0.8
        status_marker.color.b = 1.0
        status_marker.text = "recognized: %d" % len(self.landmarks)
        array_msg.markers.append(status_marker)

        self.marker_pub.publish(array_msg)

    def _to_map_pose(self, detection_msg):
        pose_msg = PoseStamped()
        pose_msg.header = detection_msg.header
        pose_msg.pose = detection_msg.pose
        if self.use_latest_tf:
            pose_msg.header.stamp = rospy.Time(0)
        pose_map = self.tf_buffer.transform(pose_msg, self.map_frame, rospy.Duration(0.3))
        return pose_map

    def _add_landmark(self, text, pose_map):
        current = self.landmarks.get(text)
        seen_count = 1
        if current is not None:
            seen_count = int(current["seen_count"]) + 1
        self.landmarks[text] = {
            "x": float(pose_map.pose.position.x),
            "y": float(pose_map.pose.position.y),
            "z": float(pose_map.pose.position.z),
            "qx": float(pose_map.pose.orientation.x),
            "qy": float(pose_map.pose.orientation.y),
            "qz": float(pose_map.pose.orientation.z),
            "qw": float(pose_map.pose.orientation.w),
            "seen_count": seen_count,
        }
        landmark_msg = self._dict_to_landmark_msg(text, self.landmarks[text])
        self.landmark_pub.publish(landmark_msg)
        self._publish_markers()
        self._save_landmarks()

    def detection_callback(self, msg):
        text = msg.text.strip()
        if not text:
            return
        if msg.confidence < self.confidence_min:
            return
        if self.one_shot_mode and text in self.landmarks:
            return

        hit_count = self.pending_hits.get(text, 0) + 1
        self.pending_hits[text] = hit_count
        if hit_count < self.confirm_hits:
            return

        try:
            pose_map = self._to_map_pose(msg)
        except Exception:
            return

        self._add_landmark(text, pose_map)
        self.pending_hits.pop(text, None)


if __name__ == "__main__":
    rospy.init_node("qr_mapper", anonymous=False)
    QrMapperNode()
    rospy.spin()
