#!/usr/bin/env python
import math

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from sensor_msgs.msg import CameraInfo, Image

from qr_map_annotator.msg import QrDetection


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
        self.use_undistort = bool(rospy.get_param("~use_undistort", False))
        self.publish_debug_image = bool(rospy.get_param("~publish_debug_image", False))
        self.output_frame_id = rospy.get_param("~output_frame_id", "")

        self.period = 1.0 / max(self.detect_rate_hz, 1.0)
        self.last_process = rospy.Time(0)
        self.camera_matrix = None
        self.dist_coeffs = None
        self.new_camera_matrix = None
        self.image_size = None

        self.detection_pub = rospy.Publisher("~detections", QrDetection, queue_size=20)
        self.debug_pub = None
        if self.publish_debug_image:
            self.debug_pub = rospy.Publisher("~debug_image", Image, queue_size=1)

        rospy.Subscriber(self.camera_info_topic, CameraInfo, self.camera_info_callback, queue_size=1)
        rospy.Subscriber(self.image_topic, Image, self.image_callback, queue_size=1, buff_size=2 ** 24)

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.K, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.D, dtype=np.float64)
        frame_id = msg.header.frame_id.strip()
        if not self.output_frame_id and frame_id:
            self.output_frame_id = frame_id

    def _get_decode_results(self, gray_image):
        results = []
        if hasattr(self.detector, "detectAndDecodeMulti"):
            try:
                multi_output = self.detector.detectAndDecodeMulti(gray_image)
                if len(multi_output) >= 3:
                    success = multi_output[0]
                    decoded = multi_output[1]
                    points = multi_output[2]
                    if success and points is not None:
                        for idx in range(len(decoded)):
                            text = decoded[idx]
                            pts = np.array(points[idx], dtype=np.float32).reshape(4, 2)
                            results.append((text, pts))
            except Exception:
                pass
        if results:
            return results

        single_output = self.detector.detectAndDecode(gray_image)
        if len(single_output) >= 2:
            text = single_output[0]
            points = single_output[1]
            if points is not None:
                pts = np.array(points, dtype=np.float32).reshape(4, 2)
                results.append((text, pts))
        return results

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

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        if self.camera_matrix is None:
            rospy.logwarn_throttle(2.0, "qr_detector waiting for camera_info on topic: %s", self.camera_info_topic)
            if self.publish_debug_image:
                debug_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
                debug_msg.header = msg.header
                self.debug_pub.publish(debug_msg)
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

        frame_id = self.output_frame_id if self.output_frame_id else msg.header.frame_id
        decode_results = self._get_decode_results(gray)
        debug_image = None
        if self.publish_debug_image:
            debug_image = cv_image.copy()

        for text, points in decode_results:
            text = text.strip()
            if not text:
                continue
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

            if debug_image is not None:
                int_pts = points.astype(np.int32).reshape((-1, 1, 2))
                cv2.polylines(debug_image, [int_pts], True, (0, 255, 0), 2)
                px = int(points[0][0])
                py = int(points[0][1]) - 6
                cv2.putText(debug_image, text, (px, py), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        if debug_image is not None:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding="bgr8")
            debug_msg.header = msg.header
            self.debug_pub.publish(debug_msg)


if __name__ == "__main__":
    rospy.init_node("qr_detector", anonymous=False)
    QrDetectorNode()
    rospy.spin()

