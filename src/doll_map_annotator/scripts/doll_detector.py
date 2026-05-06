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

        # HSV defaults tuned for current target color range (OpenCV H: 0-179)
        self.lower_blue = np.array(rospy.get_param("~lower_blue", [98, 95, 72]), dtype=np.uint8)
        self.upper_blue = np.array(rospy.get_param("~upper_blue", [124, 255, 255]), dtype=np.uint8)
        self.lower_white = np.array(rospy.get_param("~lower_white", [0, 0, 205]), dtype=np.uint8)
        self.upper_white = np.array(rospy.get_param("~upper_white", [180, 45, 255]), dtype=np.uint8)

        self.use_lab_constraint = bool(rospy.get_param("~use_lab_constraint", True))
        self.lower_lab = np.array(rospy.get_param("~lower_lab", [172, 122, 118]), dtype=np.uint8)
        self.upper_lab = np.array(rospy.get_param("~upper_lab", [255, 134, 138]), dtype=np.uint8)

        self.min_blue_area = float(rospy.get_param("~min_blue_area", 1100.0))
        self.min_white_area = float(rospy.get_param("~min_white_area", 240.0))
        self.min_white_ratio = float(rospy.get_param("~min_white_ratio", 0.06))
        self.blue_padding_ratio = float(rospy.get_param("~blue_padding_ratio", 0.16))
        self.target_width_m = float(rospy.get_param("~target_width_m", 0.10))
        self.target_height_m = float(rospy.get_param("~target_height_m", 0.16))

        # Warm-light compensation (gray-world white balance + optional CLAHE)
        self.enable_color_preprocess = bool(rospy.get_param("~enable_color_preprocess", True))
        self.enable_clahe = bool(rospy.get_param("~enable_clahe", False))
        self.clahe_clip_limit = float(rospy.get_param("~clahe_clip_limit", 2.0))
        self.clahe_tile_size = int(rospy.get_param("~clahe_tile_size", 8))

        self.period = 1.0 / max(self.detect_rate_hz, 1.0)
        self.last_process = rospy.Time(0)
        self.kernel3 = np.ones((3, 3), np.uint8)

        self.fallback_fx = float(rospy.get_param("~fallback_fx", 525.0))
        self.fallback_fy = float(rospy.get_param("~fallback_fy", 525.0))
        self.use_fallback_intrinsics = bool(rospy.get_param("~use_fallback_intrinsics", True))
        self.has_valid_intrinsics = False

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
        frame_id = msg.header.frame_id.strip()
        if not self.output_frame_id and frame_id:
            self.output_frame_id = frame_id

        fx = float(self.camera_matrix[0, 0])
        fy = float(self.camera_matrix[1, 1])
        cx = float(self.camera_matrix[0, 2])
        cy = float(self.camera_matrix[1, 2])
        if abs(fx) > 1e-6 and abs(fy) > 1e-6:
            self.fx = fx
            self.fy = fy
            self.cx = cx
            self.cy = cy
            self.has_valid_intrinsics = True
            return

        if self.use_fallback_intrinsics:
            self.fx = self.fallback_fx
            self.fy = self.fallback_fy
            self.cx = 0.5 * float(msg.width) if msg.width > 0 else 320.0
            self.cy = 0.5 * float(msg.height) if msg.height > 0 else 240.0
            self.has_valid_intrinsics = False
            rospy.logwarn_throttle(5.0, "doll_detector camera_info invalid, using fallback intrinsics fx=%.1f fy=%.1f cx=%.1f cy=%.1f", self.fx, self.fy, self.cx, self.cy)

    def find_primary_box(self, mask, min_area):
        contours_info = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # OpenCV 3 returns (image, contours, hierarchy), OpenCV 4 returns (contours, hierarchy)
        if len(contours_info) == 3:
            _, contours, _ = contours_info
        else:
            contours, _ = contours_info

        best_box = None
        best_area = 0.0
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < min_area or area <= best_area:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            best_box = (float(x), float(y), float(w), float(h))
            best_area = float(area)
        return best_box, best_area

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

    def box_inside(self, inner_box, outer_box):
        if inner_box is None or outer_box is None:
            return False
        ix, iy, iw, ih = inner_box
        ox, oy, ow, oh = outer_box
        return ix >= ox and iy >= oy and (ix + iw) <= (ox + ow) and (iy + ih) <= (oy + oh)

    def preprocess_frame(self, frame):
        # 1) Gray-world white balance: suppress warm/yellow cast by balancing B/G/R means.
        wb = frame.astype(np.float32)
        mean_b = float(np.mean(wb[:, :, 0]))
        mean_g = float(np.mean(wb[:, :, 1]))
        mean_r = float(np.mean(wb[:, :, 2]))
        mean_gray = max((mean_b + mean_g + mean_r) / 3.0, 1.0)

        gain_b = mean_gray / max(mean_b, 1.0)
        gain_g = mean_gray / max(mean_g, 1.0)
        gain_r = mean_gray / max(mean_r, 1.0)

        wb[:, :, 0] = np.clip(wb[:, :, 0] * gain_b, 0, 255)
        wb[:, :, 1] = np.clip(wb[:, :, 1] * gain_g, 0, 255)
        wb[:, :, 2] = np.clip(wb[:, :, 2] * gain_r, 0, 255)
        out = wb.astype(np.uint8)

        # 2) Optional local contrast enhancement on luminance channel.
        if self.enable_clahe:
            lab = cv2.cvtColor(out, cv2.COLOR_BGR2LAB)
            l, a, b = cv2.split(lab)
            tile = max(2, int(self.clahe_tile_size))
            clahe = cv2.createCLAHE(clipLimit=max(0.5, self.clahe_clip_limit), tileGridSize=(tile, tile))
            l = clahe.apply(l)
            out = cv2.cvtColor(cv2.merge((l, a, b)), cv2.COLOR_LAB2BGR)

        return out

    def detect_object(self, frame):
        proc = self.preprocess_frame(frame) if self.enable_color_preprocess else frame
        hsv = cv2.cvtColor(proc, cv2.COLOR_BGR2HSV)
        lab = cv2.cvtColor(proc, cv2.COLOR_BGR2LAB)

        mask_blue = cv2.inRange(hsv, self.lower_blue, self.upper_blue)
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, self.kernel3)
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, self.kernel3)
        blue_box, blue_area = self.find_primary_box(mask_blue, self.min_blue_area)
        if blue_box is None:
            return None, None, None, 0.0

        x, y, w, h = [int(v) for v in blue_box]
        pad_x = max(2, int(w * self.blue_padding_ratio))
        pad_y = max(2, int(h * self.blue_padding_ratio))
        x0 = max(0, x - pad_x)
        y0 = max(0, y - pad_y)
        x1 = min(frame.shape[1], x + w + pad_x)
        y1 = min(frame.shape[0], y + h + pad_y)
        roi = hsv[y0:y1, x0:x1]

        white_box = None
        white_area = 0.0
        white_ratio = 0.0
        if roi.size > 0:
            roi_lab = lab[y0:y1, x0:x1]
            l_chan, a_chan, b_chan = cv2.split(roi_lab)
            _ = (l_chan, a_chan, b_chan)
            hsv_mask = cv2.inRange(roi, self.lower_white, self.upper_white)
            if self.use_lab_constraint:
                lab_mask = cv2.inRange(roi_lab, self.lower_lab, self.upper_lab)
                mask_white = cv2.bitwise_and(hsv_mask, lab_mask)
            else:
                mask_white = hsv_mask
            mask_white = cv2.morphologyEx(mask_white, cv2.MORPH_OPEN, self.kernel3)
            mask_white = cv2.morphologyEx(mask_white, cv2.MORPH_CLOSE, self.kernel3)
            local_white_box, white_area = self.find_primary_box(mask_white, self.min_white_area)
            if local_white_box is not None:
                wx, wy, ww, wh = local_white_box
                white_box = (x0 + wx, y0 + wy, ww, wh)
                white_ratio = white_area / max(float(w * h), 1.0)

        final_box = blue_box
        if white_box is not None and self.box_inside(white_box, (x0, y0, x1 - x0, y1 - y0)):
            final_box = self.merge_boxes(blue_box, white_box)
            fx, fy, fw, fh = final_box
            expand_x = max(2, int(fw * 0.08))
            expand_y = max(2, int(fh * 0.10))
            final_box = (
                max(0, int(fx - expand_x)),
                max(0, int(fy - expand_y)),
                min(frame.shape[1] - int(max(0, fx - expand_x)), int(fw + expand_x * 2)),
                min(frame.shape[0] - int(max(0, fy - expand_y)), int(fh + expand_y * 2)),
            )

        confidence = min(1.0, (blue_area / max(self.min_blue_area, 1.0)) * 0.7 + max(white_ratio, white_area / max(self.min_white_area, 1.0)) * 0.3)
        if white_box is None:
            confidence *= 0.85
        return final_box, blue_box, white_box, float(confidence)

    def estimate_pose(self, box):
        if self.fx is None or self.fy is None or self.cx is None or self.cy is None:
            rospy.logwarn_throttle(2.0, "doll_detector waiting for camera_info on topic: %s", self.camera_info_topic)
            return None
        if abs(self.fx) < 1e-6 or abs(self.fy) < 1e-6:
            rospy.logwarn_throttle(2.0, "doll_detector waiting for valid camera intrinsics on topic: %s", self.camera_info_topic)
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

    def draw_debug(self, frame, final_box, confidence):
        if final_box is None:
            return frame
        out = frame.copy()
        x, y, w, h = [int(v) for v in final_box]
        cv2.rectangle(out, (x, y), (x + w, y + h), (0, 0, 255), 2)
        text = "%s %.2f" % (self.label, confidence)
        cv2.putText(out, text, (x, max(0, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        return out

    def image_callback(self, msg):
        now = rospy.Time.now()
        if (now - self.last_process).to_sec() < self.period:
            return
        self.last_process = now

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        final_box, blue_box, white_box, confidence = self.detect_object(frame)

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

        if self.publish_debug_image:
            debug_frame = self.draw_debug(frame, final_box, confidence)
            debug_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding="bgr8")
            debug_msg.header = msg.header
            self.debug_pub.publish(debug_msg)


if __name__ == "__main__":
    rospy.init_node("doll_detector", anonymous=False)
    DollDetectorNode()
    rospy.spin()
