#!/usr/bin/env python
import math

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from sensor_msgs.msg import CameraInfo, Image

from qr_map_annotator.msg import QrDetection

try:
    from pyzbar.pyzbar import decode as zbar_decode
    from pyzbar.pyzbar import ZBarSymbol

    HAS_PYZBAR = True
except Exception:
    HAS_PYZBAR = False


def rotation_to_quaternion(rotation_matrix):
    trace = rotation_matrix[0, 0] + rotation_matrix[1, 1] + rotation_matrix[2, 2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / s
        qy = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / s
        qz = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / s
    elif rotation_matrix[0, 0] > rotation_matrix[1, 1] and rotation_matrix[0, 0] > rotation_matrix[2, 2]:
        s = math.sqrt(1.0 + rotation_matrix[0, 0] - rotation_matrix[1, 1] - rotation_matrix[2, 2]) * 2.0
        qw = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / s
        qx = 0.25 * s
        qy = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
        qz = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
    elif rotation_matrix[1, 1] > rotation_matrix[2, 2]:
        s = math.sqrt(1.0 + rotation_matrix[1, 1] - rotation_matrix[0, 0] - rotation_matrix[2, 2]) * 2.0
        qw = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / s
        qx = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
        qy = 0.25 * s
        qz = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
    else:
        s = math.sqrt(1.0 + rotation_matrix[2, 2] - rotation_matrix[0, 0] - rotation_matrix[1, 1]) * 2.0
        qw = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / s
        qx = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
        qy = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
        qz = 0.25 * s
    return qx, qy, qz, qw


class QrDetectorNode(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.detector = cv2.QRCodeDetector()

        self.image_topic = rospy.get_param("~image_topic", "/camera/image_raw")
        self.camera_info_topic = rospy.get_param("~camera_info_topic", "/camera/camera_info")
        self.qr_size_m = float(rospy.get_param("~qr_size_m", 0.168))
        self.detect_rate_hz = float(rospy.get_param("~detect_rate_hz", 10.0))
        self.resize_width = int(rospy.get_param("~resize_width", 640))
        self.roi_margin_px = int(rospy.get_param("~roi_margin_px", 30))
        self.roi_hold_frames = int(rospy.get_param("~roi_hold_frames", 10))
        self.cache_hold_frames = int(rospy.get_param("~cache_hold_frames", 6))
        self.publish_dedup_sec = float(rospy.get_param("~publish_dedup_sec", 1.0))
        self.use_undistort = bool(rospy.get_param("~use_undistort", False))
        self.publish_debug_image = bool(rospy.get_param("~publish_debug_image", False))
        self.output_frame_id = rospy.get_param("~output_frame_id", "")

        self.period = 1.0 / max(self.detect_rate_hz, 1.0)
        self.last_process = rospy.Time(0)
        self.frame_count = 0

        self.camera_matrix = None
        self.dist_coeffs = None
        self.new_camera_matrix = None
        self.image_size = None

        self.last_detect_roi = None
        self.last_detect_frame = -1
        self.last_publish_time = {}
        self.candidate_cache = []

        self.detection_pub = rospy.Publisher("~detections", QrDetection, queue_size=20)
        self.debug_pub = None
        if self.publish_debug_image:
            self.debug_pub = rospy.Publisher("~debug_image", Image, queue_size=1)

        if not HAS_PYZBAR:
            rospy.logwarn("pyzbar is not available; decode step will fall back to OpenCV decode.")

        rospy.Subscriber(self.camera_info_topic, CameraInfo, self.camera_info_callback, queue_size=1)
        rospy.Subscriber(self.image_topic, Image, self.image_callback, queue_size=1, buff_size=2 ** 24)

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.K, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.D, dtype=np.float64)
        frame_id = msg.header.frame_id.strip()
        if not self.output_frame_id and frame_id:
            self.output_frame_id = frame_id

    def _resize_gray(self, gray):
        h, w = gray.shape[:2]
        if self.resize_width <= 0 or w <= self.resize_width:
            return gray, 1.0
        scale = float(self.resize_width) / float(w)
        new_h = max(1, int(h * scale))
        resized = cv2.resize(gray, (self.resize_width, new_h), interpolation=cv2.INTER_AREA)
        return resized, scale

    def _clamp_roi(self, roi, width, height):
        x1, y1, x2, y2 = roi
        x1 = max(0, min(width - 1, int(x1)))
        y1 = max(0, min(height - 1, int(y1)))
        x2 = max(0, min(width, int(x2)))
        y2 = max(0, min(height, int(y2)))
        if x2 <= x1 or y2 <= y1:
            return None
        return (x1, y1, x2, y2)

    def _roi_from_points(self, points, width, height):
        min_xy = np.min(points, axis=0)
        max_xy = np.max(points, axis=0)
        roi = (
            int(min_xy[0]) - self.roi_margin_px,
            int(min_xy[1]) - self.roi_margin_px,
            int(max_xy[0]) + self.roi_margin_px,
            int(max_xy[1]) + self.roi_margin_px,
        )
        return self._clamp_roi(roi, width, height)

    def _detect_candidates(self, gray):
        points_list = []
        if hasattr(self.detector, "detectMulti"):
            try:
                ok, points = self.detector.detectMulti(gray)
                if ok and points is not None:
                    for p in points:
                        points_list.append(np.array(p, dtype=np.float32).reshape(4, 2))
            except Exception:
                pass

        if points_list:
            return points_list

        try:
            ok, points = self.detector.detect(gray)
            if ok and points is not None:
                points_list.append(np.array(points, dtype=np.float32).reshape(4, 2))
        except Exception:
            pass
        return points_list

    def _decode_with_pyzbar(self, gray_small, quad_small):
        roi = self._roi_from_points(quad_small, gray_small.shape[1], gray_small.shape[0])
        if roi is None:
            return ""
        x1, y1, x2, y2 = roi
        roi_img = gray_small[y1:y2, x1:x2]
        if roi_img.size == 0:
            return ""

        if HAS_PYZBAR:
            try:
                decoded = zbar_decode(roi_img, symbols=[ZBarSymbol.QRCODE])
                for obj in decoded:
                    text = obj.data.decode("utf-8").strip()
                    if text:
                        return text
            except Exception:
                return ""
        else:
            text, _, _ = self.detector.detectAndDecode(roi_img)
            return text.strip()
        return ""

    def _center(self, quad):
        c = np.mean(quad, axis=0)
        return float(c[0]), float(c[1])

    def _try_cached_text(self, quad_small):
        cx, cy = self._center(quad_small)
        best_text = ""
        best_dist = 1e12
        kept = []
        for item in self.candidate_cache:
            if (self.frame_count - item["frame"]) > max(self.cache_hold_frames, 0):
                continue
            kept.append(item)
            dx = cx - item["cx"]
            dy = cy - item["cy"]
            d = dx * dx + dy * dy
            if d < best_dist:
                best_dist = d
                best_text = item["text"]
        self.candidate_cache = kept
        if best_dist <= 40.0 * 40.0:
            return best_text
        return ""

    def _remember_candidate(self, quad_small, text):
        if not text:
            return
        cx, cy = self._center(quad_small)
        self.candidate_cache.append({"cx": cx, "cy": cy, "text": text, "frame": self.frame_count})

    def _estimate_pose(self, points, camera_matrix, dist_coeffs, width, height):
        half_size = self.qr_size_m * 0.5
        object_points = np.array(
            [
                [-half_size, half_size, 0.0],
                [half_size, half_size, 0.0],
                [half_size, -half_size, 0.0],
                [-half_size, -half_size, 0.0],
            ],
            dtype=np.float32,
        )
        ok, rvec, tvec = cv2.solvePnP(object_points, points, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            return None, 0.0

        projected, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs)
        projected = projected.reshape(4, 2)
        reproj_error = np.linalg.norm(projected - points, axis=1).mean()
        diag = math.sqrt(float(width * width + height * height))
        confidence = float(max(0.0, 1.0 - reproj_error / max(diag * 0.05, 1.0)))

        rotation_matrix, _ = cv2.Rodrigues(rvec)
        qx, qy, qz, qw = rotation_to_quaternion(rotation_matrix)
        pose = Pose()
        pose.position.x = float(tvec[0][0])
        pose.position.y = float(tvec[1][0])
        pose.position.z = float(tvec[2][0])
        pose.orientation.x = float(qx)
        pose.orientation.y = float(qy)
        pose.orientation.z = float(qz)
        pose.orientation.w = float(qw)
        return pose, confidence

    def image_callback(self, msg):
        now = rospy.Time.now()
        if (now - self.last_process).to_sec() < self.period:
            return
        self.last_process = now
        self.frame_count += 1

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        if self.camera_matrix is None:
            rospy.logwarn_throttle(2.0, "qr_detector waiting for camera_info on topic: %s", self.camera_info_topic)
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        image_h, image_w = gray.shape[:2]
        camera_matrix = self.camera_matrix
        dist_coeffs = self.dist_coeffs

        if self.use_undistort:
            if self.image_size != (image_w, image_h) or self.new_camera_matrix is None:
                self.image_size = (image_w, image_h)
                self.new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, self.image_size, 0.0)
            gray = cv2.undistort(gray, camera_matrix, dist_coeffs, None, self.new_camera_matrix)
            camera_matrix = self.new_camera_matrix
            dist_coeffs = np.zeros((5, 1), dtype=np.float64)

        gray_small, scale = self._resize_gray(gray)
        small_h, small_w = gray_small.shape[:2]

        detect_img = gray_small
        detect_offset = (0, 0)

        if self.last_detect_roi is not None and (self.frame_count - self.last_detect_frame) <= max(self.roi_hold_frames, 0):
            roi = self._clamp_roi(self.last_detect_roi, small_w, small_h)
            if roi is not None:
                x1, y1, x2, y2 = roi
                roi_img = gray_small[y1:y2, x1:x2]
                if roi_img.size > 0:
                    detect_img = roi_img
                    detect_offset = (x1, y1)

        candidates = self._detect_candidates(detect_img)
        if not candidates and detect_offset != (0, 0):
            detect_img = gray_small
            detect_offset = (0, 0)
            candidates = self._detect_candidates(detect_img)

        frame_id = self.output_frame_id if self.output_frame_id else msg.header.frame_id
        debug_image = cv_image.copy() if self.publish_debug_image else None

        for quad in candidates:
            quad[:, 0] += detect_offset[0]
            quad[:, 1] += detect_offset[1]

            text = self._try_cached_text(quad)
            if not text:
                text = self._decode_with_pyzbar(gray_small, quad)
                self._remember_candidate(quad, text)

            if not text:
                continue

            self.last_detect_roi = self._roi_from_points(quad, small_w, small_h)
            self.last_detect_frame = self.frame_count

            last_pub_t = self.last_publish_time.get(text)
            if last_pub_t is not None and (now - last_pub_t).to_sec() < max(self.publish_dedup_sec, 0.0):
                continue

            points = quad.copy()
            if scale != 1.0:
                points *= (1.0 / scale)

            pose, confidence = self._estimate_pose(points, camera_matrix, dist_coeffs, image_w, image_h)
            if pose is None:
                continue

            out = QrDetection()
            out.header.stamp = msg.header.stamp
            out.header.frame_id = frame_id
            out.text = text
            out.pose = pose
            out.confidence = confidence
            self.detection_pub.publish(out)
            self.last_publish_time[text] = now

            if debug_image is not None:
                int_pts = points.astype(np.int32).reshape((-1, 1, 2))
                cv2.polylines(debug_image, [int_pts], True, (0, 255, 0), 2)
                cv2.putText(debug_image, text, (int(points[0][0]), int(points[0][1]) - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        if debug_image is not None:
            if self.last_detect_roi is not None and (self.frame_count - self.last_detect_frame) <= max(self.roi_hold_frames, 0):
                roi = self._clamp_roi(self.last_detect_roi, small_w, small_h)
                if roi is not None:
                    x1, y1, x2, y2 = roi
                    inv = 1.0 / scale
                    cv2.rectangle(
                        debug_image,
                        (int(x1 * inv), int(y1 * inv)),
                        (int(x2 * inv), int(y2 * inv)),
                        (255, 0, 0),
                        1,
                    )
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding="bgr8")
            debug_msg.header = msg.header
            self.debug_pub.publish(debug_msg)


if __name__ == "__main__":
    rospy.init_node("qr_detector", anonymous=False)
    QrDetectorNode()
    rospy.spin()
