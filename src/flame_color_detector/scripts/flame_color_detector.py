#!/usr/bin/env python
# -*- coding: utf-8 -*-

import ast

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


class FlameColorDetectorNode(object):
    @staticmethod
    def _read_hsv_param(name, default):
        value = rospy.get_param("~" + name, default)
        if isinstance(value, str):
            try:
                value = ast.literal_eval(value)
            except (ValueError, SyntaxError):
                rospy.logwarn("invalid HSV param ~%s=%s, use default %s", name, value, default)
                value = default

        if not isinstance(value, (list, tuple)) or len(value) != 3:
            rospy.logwarn("invalid HSV param ~%s=%s, use default %s", name, value, default)
            value = default

        return np.array([int(v) for v in value], dtype=np.uint8)

    def __init__(self):
        self.bridge = CvBridge()

        self.image_topic = rospy.get_param("~image_topic", "/camera/rgb/image_raw")
        self.detect_rate_hz = float(rospy.get_param("~detect_rate_hz", 10.0))
        self.publish_debug_image = bool(rospy.get_param("~publish_debug_image", True))

        # OpenCV HSV: H in [0, 179]
        # 红色跨越 HSV 色相首尾，因此拆成两个区间；火焰常含橙黄色，保留 orange 区间增强鲁棒性。
        # launch 文件中 [0, 120, 120] 在部分 ROS/Python2 环境会被读成字符串，这里统一做兼容解析。
        self.lower_red1 = self._read_hsv_param("lower_red1", [0, 120, 120])
        self.upper_red1 = self._read_hsv_param("upper_red1", [12, 255, 255])
        self.lower_red2 = self._read_hsv_param("lower_red2", [168, 120, 120])
        self.upper_red2 = self._read_hsv_param("upper_red2", [179, 255, 255])
        self.lower_orange = self._read_hsv_param("lower_orange", [10, 120, 120])
        self.upper_orange = self._read_hsv_param("upper_orange", [28, 255, 255])

        self.min_area = float(rospy.get_param("~min_area", 250.0))
        self.min_fill_ratio = float(rospy.get_param("~min_fill_ratio", 0.30))

        self.period = 1.0 / max(self.detect_rate_hz, 1.0)
        self.last_process = rospy.Time(0)
        self.kernel3 = np.ones((3, 3), np.uint8)

        self.detect_pub = rospy.Publisher("~detected", Bool, queue_size=10)
        self.debug_pub = rospy.Publisher("~debug_image", Image, queue_size=1) if self.publish_debug_image else None

        rospy.Subscriber(self.image_topic, Image, self.image_callback, queue_size=1, buff_size=2 ** 24)
        rospy.loginfo("flame_color_detector subscribe image: %s", self.image_topic)

    def _largest_box(self, mask):
        contours_info = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours_info) == 3:
            _, contours, _ = contours_info
        else:
            contours, _ = contours_info

        best_box = None
        best_area = 0.0
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < self.min_area or area <= best_area:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            if w <= 0 or h <= 0:
                continue

            fill_ratio = area / float(w * h)
            if fill_ratio < self.min_fill_ratio:
                continue

            best_box = (x, y, w, h)
            best_area = area

        return best_box, best_area

    def detect(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask_r1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        mask_r2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        mask_o = cv2.inRange(hsv, self.lower_orange, self.upper_orange)

        mask = cv2.bitwise_or(mask_r1, mask_r2)
        mask = cv2.bitwise_or(mask, mask_o)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel3)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel3)

        box, area = self._largest_box(mask)
        detected = box is not None
        return detected, box, area, mask

    def draw_debug(self, frame, detected, box, area):
        out = frame.copy()
        text = "flame: YES area: %.1f" % area if detected else "flame: NO"
        color = (0, 0, 255) if detected else (0, 255, 0)
        cv2.putText(out, text, (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        if box is not None:
            x, y, w, h = box
            cv2.rectangle(out, (x, y), (x + w, y + h), (0, 0, 255), 2)
        return out

    def image_callback(self, msg):
        now = rospy.Time.now()
        if (now - self.last_process).to_sec() < self.period:
            return
        self.last_process = now

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as err:
            rospy.logwarn("cv_bridge convert image failed: %s", err)
            return

        detected, box, area, _ = self.detect(frame)
        self.detect_pub.publish(Bool(data=detected))

        if self.publish_debug_image:
            debug = self.draw_debug(frame, detected, box, area)
            debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
            debug_msg.header = msg.header
            self.debug_pub.publish(debug_msg)


if __name__ == "__main__":
    rospy.init_node("flame_color_detector", anonymous=False)
    FlameColorDetectorNode()
    rospy.spin()
