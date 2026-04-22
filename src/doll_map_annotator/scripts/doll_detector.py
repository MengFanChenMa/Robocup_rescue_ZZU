#!/usr/bin/env python
import math

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from sensor_msgs.msg import CameraInfo, Image

from doll_map_annotator.msg import DollDetection


class DollDetectorNode(object):
    def __init__(self):
        self.bridge = CvBridge()

        self.image_topic = rospy.get_param("~image_topic", "/camera/image_raw")
        self.camera_info_topic = rospy.get_param("~camera_info_topic", "/camera/camera_info")
        self.detect_rate_hz = float(rospy.get_param("~detect_rate_hz", 10.0))
        self.publish_debug_image = bool(rospy.get_param("~publish_debug_image", True))
        self.label = rospy.get_param("~label", "doll")
        self.output_frame_id = rospy.get_param("~output_frame_id", "")

        self.lower_blue = np.array(rospy.get_param("~lower_blue", [100, 100, 120]), dtype=np.uint8)
        self.upper_blue = np.array(rospy.get_param("~upper_blue", [115, 220, 255]), dtype=np.uint8)
        self.lower_white = np.array(rospy.get_param("~lower_white", [0, 0, 200]), dtype=np.uint8)
        self.upper_white = np.array(rospy.get_param("~upper_white", [180, 30, 255]), dtype=np.uint8)

        self.min_blue_area = float(rospy.get_param("~min_blue_area", 1000.0))
        self.min_white_area = float(rospy.get_param("~min_white_area", 500.0))
        self.max_distance_ratio = float(rospy.get_param("~max_distance_ratio", 2.0))
        self.target_width_m = float(rospy.get_param("~target_width_m", 0.10))
        self.target_height_m = float(rospy.get_param("~target_height_m", 0.16))

        self.period = 1.0 / max(self.detect_rate_hz, 1.0)
        self.last_process = rospy.Time(0)

        self.camera_matrix = None
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.detection_pub = rospy.Publisher("~detections", DollDetection, queue_size=20)
        self.debug_pub = None
        if self.publish_debug_image:
            self.debug_pub = rospy.Publisher("~debug_image", Image, queue_size=1)

        rospy.Subscriber(self.camera_info_topic, CameraInfo, self.camera_info_callback, queue_size=1)
        rospy.Subscriber(self.image_topic, Image, self.image_callback, queue_size=1, buff_size=2 ** 24)

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.K, dtype=np.float64).reshape(3, 3)
        self.fx = float(self.camera_matrix[0, 0])
        self.fy = float(self.camera_matrix[1, 1])
        self.cx = float(self.camera_matrix[0, 2])
        self.cy = float(self.camera_matrix[1, 2])
        frame_id = msg.header.frame_id.strip()
        if not self.output_frame_id and frame_id:
            self.output_frame_id = frame_id

    def find_largest_contour(self, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            return max(contours, key=cv2.contourArea)
        return None

    def get_bbox_from_contour(self, contour):
        if contour is None:
            return None
        x, y, w, h = cv2.boundingRect(contour)
        return (float(x), float(y), float(w), float(h))

    def boxes_are_close(self, box1, box2):
        if box1 is None or box2 is None:
            return False

        x1, y1, w1, h1 = box1
        x2, y2, w2, h2 = box2
        center1_x = x1 + w1 * 0.5
        center1_y = y1 + h1 * 0.5
        center2_x = x2 + w2 * 0.5
        center2_y = y2 + h2 * 0.5
        distance = math.sqrt((center1_x - center2_x) ** 2 + (center1_y - center2_y) ** 2)
        avg_size = (w1 + h1 + w2 + h2) * 0.25
        return distance < (avg_size * self.max_distance_ratio)

    def merge_boxes(self, box1, box2):
        if box1 is None:
            return box2
        if box2 is None:
            return box1
        x1, y1, w1, h1 = box1
        x2, y2, w2, h2 = box2
        x_min = min(x1, x2)
        y_min = min(y1, y2)
        x_max = max(x1 + w1, x2 + w2)
        y_max = max(y1 + h1, y2 + h2)
        return (x_min, y_min, x_max - x_min, y_max - y_min)

    def detect_object(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        kernel3 = np.ones((3, 3), np.uint8)
        kernel5 = np.ones((5, 5), np.uint8)

        mask_blue = cv2.inRange(hsv, self.lower_blue, self.upper_blue)
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel3)
        mask_blue = cv2.dilate(mask_blue, kernel5, iterations=2)

        mask_white = cv2.inRange(hsv, self.lower_white, self.upper_white)
        mask_white = cv2.morphologyEx(mask_white, cv2.MORPH_OPEN, kernel3)
        mask_white = cv2.dilate(mask_white, kernel5, iterations=1)

        blue_contour = self.find_largest_contour(mask_blue)
        white_contour = self.find_largest_contour(mask_white)
        blue_box = self.get_bbox_from_contour(blue_contour)
        white_box = self.get_bbox_from_contour(white_contour)

        blue_area = cv2.contourArea(blue_contour) if blue_contour is not None else 0.0
        white_area = cv2.contourArea(white_contour) if white_contour is not None else 0.0
        if blue_area < self.min_blue_area:
            return None, blue_box, white_box, 0.0

        if white_area >= self.min_white_area and self.boxes_are_close(blue_box, white_box):
            final_box = self.merge_boxes(blue_box, white_box)
        else:
            final_box = blue_box

        confidence = min(1.0, (blue_area + white_area) / max(self.min_blue_area + self.min_white_area, 1.0))
        return final_box, blue_box, white_box, float(confidence)

    def estimate_pose(self, box):
        if self.fx is None or self.fy is None:
            return None
        x, y, w, h = box
        if w <= 1.0 or h <= 1.0:
            return None

        # Pin-hole approximation from bounding box pixel size.
        z_from_w = (self.fx * self.target_width_m) / w
        z_from_h = (self.fy * self.target_height_m) / h
        z = max(0.05, 0.5 * (z_from_w + z_from_h))
        u = x + w * 0.5
        v = y + h * 0.5

        pose = Pose()
        pose.position.x = (u - self.cx) * z / self.fx
        pose.position.y = (v - self.cy) * z / self.fy
        pose.position.z = z
        pose.orientation.w = 1.0
        return pose

    def draw_debug(self, frame, blue_box, white_box, final_box, confidence):
        out = frame.copy()
        if blue_box is not None:
            x, y, w, h = [int(v) for v in blue_box]
            cv2.rectangle(out, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(out, "Blue", (x, max(0, y - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        if white_box is not None:
            x, y, w, h = [int(v) for v in white_box]
            cv2.rectangle(out, (x, y), (x + w, y + h), (255, 255, 0), 2)
            cv2.putText(out, "White", (x, max(0, y - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
        if final_box is not None:
            x, y, w, h = [int(v) for v in final_box]
            cv2.rectangle(out, (x, y), (x + w, y + h), (0, 0, 255), 3)
            text = "%s %.2f" % (self.label, confidence)
            cv2.putText(out, text, (x, max(0, y - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        return out

    def image_callback(self, msg):
        now = rospy.Time.now()
        if (now - self.last_process).to_sec() < self.period:
            return
        self.last_process = now

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        final_box, blue_box, white_box, confidence = self.detect_object(frame)

        debug_image = None
        if self.publish_debug_image:
            debug_image = self.draw_debug(frame, blue_box, white_box, final_box, confidence)

        if final_box is not None:
            pose = self.estimate_pose(final_box)
            if pose is not None:
                out = DollDetection()
                out.header.stamp = msg.header.stamp
                out.header.frame_id = self.output_frame_id if self.output_frame_id else msg.header.frame_id
                out.label = self.label
                out.pose = pose
                out.confidence = confidence
                out.bbox_cx = final_box[0] + final_box[2] * 0.5
                out.bbox_cy = final_box[1] + final_box[3] * 0.5
                out.bbox_w = final_box[2]
                out.bbox_h = final_box[3]
                self.detection_pub.publish(out)

        if debug_image is not None:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding="bgr8")
            debug_msg.header = msg.header
            self.debug_pub.publish(debug_msg)


if __name__ == "__main__":
    rospy.init_node("doll_detector", anonymous=False)
    DollDetectorNode()
    rospy.spin()
