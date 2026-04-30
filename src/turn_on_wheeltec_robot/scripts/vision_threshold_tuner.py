#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
from collections import deque

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class VisionThresholdTuner(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.image_topic = rospy.get_param("~image_topic", "/camera/rgb/image_raw")
        self.profile = rospy.get_param("~profile", "doll").strip().lower()  # doll | qr_a4
        self.window = rospy.get_param("~window", "vision_threshold_tuner")
        self.ctrl_window = rospy.get_param("~ctrl_window", "vision_threshold_controls")
        self.show_fps = bool(rospy.get_param("~show_fps", True))
        self.publish_preview = bool(rospy.get_param("~publish_preview", False))
        self.preview_topic = rospy.get_param("~preview_topic", "~preview")
        self.view_scale = float(rospy.get_param("~view_scale", 1.6))
        self.min_view_width = int(rospy.get_param("~min_view_width", 1600))

        if self.profile not in ["doll", "qr_a4"]:
            rospy.logwarn("Unknown profile=%s, fallback to doll", self.profile)
            self.profile = "doll"

        self.frame = None
        self.frame_stamp = rospy.Time(0)
        self.last_tick = time.time()
        self.fps = 0.0
        self.lat_hist = deque(maxlen=30)
        self.score_hist = deque(maxlen=60)

        self.pub_preview = rospy.Publisher(self.preview_topic, Image, queue_size=1) if self.publish_preview else None
        rospy.Subscriber(self.image_topic, Image, self.image_cb, queue_size=1, buff_size=2 ** 24)

        self.kernel3 = np.ones((3, 3), np.uint8)
        self._build_ui()

    def _build_ui(self):
        cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window, 1920, 1080)
        cv2.namedWindow(self.ctrl_window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.ctrl_window, 720, 900)

        if self.profile == "doll":
            cv2.createTrackbar("BH_min", self.ctrl_window, int(rospy.get_param("~lower_blue_h", 100)), 180, lambda x: None)
            cv2.createTrackbar("BS_min", self.ctrl_window, int(rospy.get_param("~lower_blue_s", 90)), 255, lambda x: None)
            cv2.createTrackbar("BV_min", self.ctrl_window, int(rospy.get_param("~lower_blue_v", 70)), 255, lambda x: None)
            cv2.createTrackbar("BH_max", self.ctrl_window, int(rospy.get_param("~upper_blue_h", 124)), 180, lambda x: None)
            cv2.createTrackbar("BS_max", self.ctrl_window, int(rospy.get_param("~upper_blue_s", 255)), 255, lambda x: None)
            cv2.createTrackbar("BV_max", self.ctrl_window, int(rospy.get_param("~upper_blue_v", 255)), 255, lambda x: None)

            cv2.createTrackbar("WH_min", self.ctrl_window, int(rospy.get_param("~lower_white_h", 0)), 180, lambda x: None)
            cv2.createTrackbar("WS_min", self.ctrl_window, int(rospy.get_param("~lower_white_s", 0)), 255, lambda x: None)
            cv2.createTrackbar("WV_min", self.ctrl_window, int(rospy.get_param("~lower_white_v", 175)), 255, lambda x: None)
            cv2.createTrackbar("WH_max", self.ctrl_window, int(rospy.get_param("~upper_white_h", 180)), 180, lambda x: None)
            cv2.createTrackbar("WS_max", self.ctrl_window, int(rospy.get_param("~upper_white_s", 70)), 255, lambda x: None)
            cv2.createTrackbar("WV_max", self.ctrl_window, int(rospy.get_param("~upper_white_v", 255)), 255, lambda x: None)

            cv2.createTrackbar("min_blue_area", self.ctrl_window, int(rospy.get_param("~min_blue_area", 700)), 20000, lambda x: None)
            cv2.createTrackbar("min_white_area", self.ctrl_window, int(rospy.get_param("~min_white_area", 180)), 8000, lambda x: None)
            cv2.createTrackbar("min_white_ratio_x1000", self.ctrl_window, int(rospy.get_param("~min_white_ratio_x1000", 10)), 300, lambda x: None)
            cv2.createTrackbar("blue_pad_x100", self.ctrl_window, int(rospy.get_param("~blue_padding_ratio_x100", 18)), 100, lambda x: None)
        else:
            cv2.createTrackbar("L_min", self.ctrl_window, int(rospy.get_param("~a4_lab_lower_l", 180)), 255, lambda x: None)
            cv2.createTrackbar("A_min", self.ctrl_window, int(rospy.get_param("~a4_lab_lower_a", 95)), 255, lambda x: None)
            cv2.createTrackbar("B_min", self.ctrl_window, int(rospy.get_param("~a4_lab_lower_b", 89)), 255, lambda x: None)
            cv2.createTrackbar("L_max", self.ctrl_window, int(rospy.get_param("~a4_lab_upper_l", 245)), 255, lambda x: None)
            cv2.createTrackbar("A_max", self.ctrl_window, int(rospy.get_param("~a4_lab_upper_a", 134)), 255, lambda x: None)
            cv2.createTrackbar("B_max", self.ctrl_window, int(rospy.get_param("~a4_lab_upper_b", 139)), 255, lambda x: None)

            cv2.createTrackbar("min_white_area", self.ctrl_window, int(rospy.get_param("~a4_min_white_area", 2500)), 50000, lambda x: None)
            cv2.createTrackbar("min_black_x1000", self.ctrl_window, int(rospy.get_param("~a4_min_black_ratio_x1000", 5)), 400, lambda x: None)
            cv2.createTrackbar("max_black_x1000", self.ctrl_window, int(rospy.get_param("~a4_max_black_ratio_x1000", 250)), 1000, lambda x: None)
            cv2.createTrackbar("aspect_min_x100", self.ctrl_window, int(rospy.get_param("~a4_min_aspect_x100", 50)), 300, lambda x: None)
            cv2.createTrackbar("aspect_max_x100", self.ctrl_window, int(rospy.get_param("~a4_max_aspect_x100", 180)), 400, lambda x: None)

    def image_cb(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.frame_stamp = msg.header.stamp

    def _find_primary_box(self, mask, min_area):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best_box = None
        best_area = 0.0
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < min_area or area <= best_area:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            best_box = (int(x), int(y), int(w), int(h))
            best_area = float(area)
        return best_box, best_area

    def _overlay(self, img, lines, color=(0, 255, 0)):
        y = 25
        for line in lines:
            cv2.putText(img, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
            y += 24

    def _process_doll(self, frame):
        t0 = time.time()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lb = np.array([
            cv2.getTrackbarPos("BH_min", self.ctrl_window),
            cv2.getTrackbarPos("BS_min", self.ctrl_window),
            cv2.getTrackbarPos("BV_min", self.ctrl_window)], dtype=np.uint8)
        ub = np.array([
            cv2.getTrackbarPos("BH_max", self.window),
            cv2.getTrackbarPos("BS_max", self.window),
            cv2.getTrackbarPos("BV_max", self.window)], dtype=np.uint8)
        lw = np.array([
            cv2.getTrackbarPos("WH_min", self.window),
            cv2.getTrackbarPos("WS_min", self.window),
            cv2.getTrackbarPos("WV_min", self.window)], dtype=np.uint8)
        uw = np.array([
            cv2.getTrackbarPos("WH_max", self.window),
            cv2.getTrackbarPos("WS_max", self.window),
            cv2.getTrackbarPos("WV_max", self.window)], dtype=np.uint8)

        min_blue_area = float(cv2.getTrackbarPos("min_blue_area", self.window))
        min_white_area = float(cv2.getTrackbarPos("min_white_area", self.window))
        min_white_ratio = float(cv2.getTrackbarPos("min_white_ratio_x1000", self.window)) / 1000.0
        blue_padding_ratio = float(cv2.getTrackbarPos("blue_pad_x100", self.window)) / 100.0

        mask_blue = cv2.inRange(hsv, lb, ub)
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, self.kernel3)
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, self.kernel3)
        blue_box, blue_area = self._find_primary_box(mask_blue, min_blue_area)

        white_ratio = 0.0
        white_area = 0.0
        white_box = None
        final_box = blue_box

        if blue_box is not None:
            x, y, w, h = blue_box
            pad_x = max(2, int(w * blue_padding_ratio))
            pad_y = max(2, int(h * blue_padding_ratio))
            x0 = max(0, x - pad_x)
            y0 = max(0, y - pad_y)
            x1 = min(frame.shape[1], x + w + pad_x)
            y1 = min(frame.shape[0], y + h + pad_y)
            roi = hsv[y0:y1, x0:x1]
            if roi.size > 0:
                mask_white = cv2.inRange(roi, lw, uw)
                mask_white = cv2.morphologyEx(mask_white, cv2.MORPH_OPEN, self.kernel3)
                mask_white = cv2.morphologyEx(mask_white, cv2.MORPH_CLOSE, self.kernel3)
                local_white_box, white_area = self._find_primary_box(mask_white, min_white_area)
                if local_white_box is not None:
                    wx, wy, ww, wh = local_white_box
                    white_box = (x0 + wx, y0 + wy, ww, wh)
                    white_ratio = white_area / max(float(w * h), 1.0)

        blue_score = min(1.0, blue_area / max(min_blue_area, 1.0)) if blue_box is not None else 0.0
        white_area_score = min(1.0, white_area / max(min_white_area, 1.0)) if white_box is not None else 0.0
        white_ratio_score = min(1.0, white_ratio / max(min_white_ratio, 1e-6)) if white_box is not None else 0.0
        stability_score = 1.0 if blue_box is not None else 0.0
        effective_score = 0.45 * blue_score + 0.25 * white_area_score + 0.20 * white_ratio_score + 0.10 * stability_score

        self.score_hist.append(float(effective_score))
        latency = (time.time() - t0) * 1000.0
        self.lat_hist.append(latency)

        out = frame.copy()
        if blue_box is not None:
            x, y, w, h = blue_box
            cv2.rectangle(out, (x, y), (x + w, y + h), (255, 0, 0), 2)
        if white_box is not None:
            x, y, w, h = white_box
            cv2.rectangle(out, (x, y), (x + w, y + h), (0, 255, 255), 2)
        if final_box is not None:
            x, y, w, h = final_box
            cv2.rectangle(out, (x, y), (x + w, y + h), (0, 0, 255), 2)

        lines = [
            "profile=doll | q=quit s=save",
            "effective_score={:.3f} stable_avg={:.3f}".format(effective_score, np.mean(self.score_hist) if self.score_hist else 0.0),
            "blue_score={:.3f} white_area_score={:.3f} white_ratio_score={:.3f}".format(blue_score, white_area_score, white_ratio_score),
            "blue_area={:.1f} white_area={:.1f} white_ratio={:.4f}".format(blue_area, white_area, white_ratio),
            "latency_ms={:.1f} avg={:.1f}".format(latency, np.mean(self.lat_hist) if self.lat_hist else 0.0),
            "params: lower_blue={} upper_blue={}".format(lb.tolist(), ub.tolist()),
            "params: lower_white={} upper_white={}".format(lw.tolist(), uw.tolist()),
            "params: min_blue_area={} min_white_area={} min_white_ratio={:.3f} pad={:.2f}".format(int(min_blue_area), int(min_white_area), min_white_ratio, blue_padding_ratio),
        ]
        self._overlay(out, lines)

        mask_blue_bgr = cv2.cvtColor(mask_blue, cv2.COLOR_GRAY2BGR)
        panel = np.hstack([out, mask_blue_bgr])
        return panel

    def _process_qr_a4(self, frame):
        t0 = time.time()
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        lower = np.array([
            cv2.getTrackbarPos("L_min", self.window),
            cv2.getTrackbarPos("A_min", self.window),
            cv2.getTrackbarPos("B_min", self.window)], dtype=np.uint8)
        upper = np.array([
            cv2.getTrackbarPos("L_max", self.window),
            cv2.getTrackbarPos("A_max", self.window),
            cv2.getTrackbarPos("B_max", self.window)], dtype=np.uint8)

        min_white_area = float(cv2.getTrackbarPos("min_white_area", self.window))
        min_black_ratio = float(cv2.getTrackbarPos("min_black_x1000", self.window)) / 1000.0
        max_black_ratio = float(cv2.getTrackbarPos("max_black_x1000", self.window)) / 1000.0
        min_aspect = float(cv2.getTrackbarPos("aspect_min_x100", self.window)) / 100.0
        max_aspect = float(cv2.getTrackbarPos("aspect_max_x100", self.window)) / 100.0

        mask_white = cv2.inRange(lab, lower, upper)
        mask_white = cv2.morphologyEx(mask_white, cv2.MORPH_OPEN, self.kernel3)
        mask_white = cv2.morphologyEx(mask_white, cv2.MORPH_CLOSE, self.kernel3)

        contours, _ = cv2.findContours(mask_white, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        out = frame.copy()
        accepted = 0
        best_black_ratio = 0.0
        best_area = 0.0

        for c in contours:
            area = cv2.contourArea(c)
            if area < min_white_area:
                continue
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.03 * peri, True)
            if approx is None or len(approx) != 4:
                continue
            x, y, w, h = cv2.boundingRect(approx)
            if w <= 2 or h <= 2:
                continue
            aspect = float(w) / float(h)
            if aspect < min_aspect or aspect > max_aspect:
                continue

            roi = gray[y:y + h, x:x + w]
            if roi.size == 0:
                continue
            _, black_mask = cv2.threshold(roi, 80, 255, cv2.THRESH_BINARY_INV)
            black_ratio = float(np.count_nonzero(black_mask)) / float(w * h)
            if black_ratio < min_black_ratio or black_ratio > max_black_ratio:
                continue

            accepted += 1
            best_black_ratio = max(best_black_ratio, black_ratio)
            best_area = max(best_area, area)
            cv2.polylines(out, [approx], True, (0, 0, 255), 2)

        candidate_score = min(1.0, accepted / 1.0)
        area_score = min(1.0, best_area / max(min_white_area, 1.0))
        black_center = 0.5 * (min_black_ratio + max_black_ratio)
        black_width = max(1e-4, 0.5 * (max_black_ratio - min_black_ratio))
        black_score = max(0.0, 1.0 - abs(best_black_ratio - black_center) / black_width) if accepted > 0 else 0.0
        effective_score = 0.5 * candidate_score + 0.3 * area_score + 0.2 * black_score

        self.score_hist.append(float(effective_score))
        latency = (time.time() - t0) * 1000.0
        self.lat_hist.append(latency)

        lines = [
            "profile=qr_a4 | q=quit s=save",
            "effective_score={:.3f} stable_avg={:.3f}".format(effective_score, np.mean(self.score_hist) if self.score_hist else 0.0),
            "accepted={} best_area={:.1f} best_black_ratio={:.4f}".format(accepted, best_area, best_black_ratio),
            "candidate_score={:.3f} area_score={:.3f} black_score={:.3f}".format(candidate_score, area_score, black_score),
            "latency_ms={:.1f} avg={:.1f}".format(latency, np.mean(self.lat_hist) if self.lat_hist else 0.0),
            "params: lab_lower={} lab_upper={}".format(lower.tolist(), upper.tolist()),
            "params: min_area={} black=[{:.3f},{:.3f}] aspect=[{:.2f},{:.2f}]".format(int(min_white_area), min_black_ratio, max_black_ratio, min_aspect, max_aspect),
        ]
        self._overlay(out, lines)

        mask_bgr = cv2.cvtColor(mask_white, cv2.COLOR_GRAY2BGR)
        panel = np.hstack([out, mask_bgr])
        return panel

    def _save_params(self):
        if self.profile == "doll":
            data = {
                "lower_blue": [cv2.getTrackbarPos("BH_min", self.window), cv2.getTrackbarPos("BS_min", self.window), cv2.getTrackbarPos("BV_min", self.window)],
                "upper_blue": [cv2.getTrackbarPos("BH_max", self.window), cv2.getTrackbarPos("BS_max", self.window), cv2.getTrackbarPos("BV_max", self.window)],
                "lower_white": [cv2.getTrackbarPos("WH_min", self.window), cv2.getTrackbarPos("WS_min", self.window), cv2.getTrackbarPos("WV_min", self.window)],
                "upper_white": [cv2.getTrackbarPos("WH_max", self.window), cv2.getTrackbarPos("WS_max", self.window), cv2.getTrackbarPos("WV_max", self.window)],
                "min_blue_area": int(cv2.getTrackbarPos("min_blue_area", self.window)),
                "min_white_area": int(cv2.getTrackbarPos("min_white_area", self.window)),
                "min_white_ratio": float(cv2.getTrackbarPos("min_white_ratio_x1000", self.window)) / 1000.0,
                "blue_padding_ratio": float(cv2.getTrackbarPos("blue_pad_x100", self.window)) / 100.0,
            }
        else:
            data = {
                "a4_lab_lower_l": int(cv2.getTrackbarPos("L_min", self.window)),
                "a4_lab_lower_a": int(cv2.getTrackbarPos("A_min", self.window)),
                "a4_lab_lower_b": int(cv2.getTrackbarPos("B_min", self.window)),
                "a4_lab_upper_l": int(cv2.getTrackbarPos("L_max", self.window)),
                "a4_lab_upper_a": int(cv2.getTrackbarPos("A_max", self.window)),
                "a4_lab_upper_b": int(cv2.getTrackbarPos("B_max", self.window)),
                "a4_min_white_area": int(cv2.getTrackbarPos("min_white_area", self.window)),
                "a4_min_black_ratio": float(cv2.getTrackbarPos("min_black_x1000", self.window)) / 1000.0,
                "a4_max_black_ratio": float(cv2.getTrackbarPos("max_black_x1000", self.window)) / 1000.0,
                "a4_min_aspect": float(cv2.getTrackbarPos("aspect_min_x100", self.window)) / 100.0,
                "a4_max_aspect": float(cv2.getTrackbarPos("aspect_max_x100", self.window)) / 100.0,
            }

        for k, v in data.items():
            rospy.set_param("~saved/" + k, v)
        rospy.loginfo("Saved params under namespace: %s/saved", rospy.get_name())

    def _resize_for_view(self, img):
        h, w = img.shape[:2]
        target_w = max(self.min_view_width, int(w * self.view_scale))
        scale = float(target_w) / float(max(1, w))
        target_h = int(max(1, h * scale))
        return cv2.resize(img, (target_w, target_h), interpolation=cv2.INTER_LINEAR)

    def spin(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.frame is None:
                rate.sleep()
                continue

            now = time.time()
            dt = max(1e-6, now - self.last_tick)
            self.last_tick = now
            self.fps = 0.9 * self.fps + 0.1 * (1.0 / dt)

            if self.profile == "doll":
                panel = self._process_doll(self.frame)
            else:
                panel = self._process_qr_a4(self.frame)

            if self.show_fps:
                cv2.putText(panel, "FPS {:.1f}".format(self.fps), (10, panel.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

            view = self._resize_for_view(panel)
            cv2.imshow(self.window, view)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            if key == ord('s'):
                self._save_params()

            if self.pub_preview is not None:
                msg = self.bridge.cv2_to_imgmsg(panel, encoding="bgr8")
                msg.header.stamp = self.frame_stamp
                self.pub_preview.publish(msg)

            rate.sleep()

        cv2.destroyWindow(self.window)
        cv2.destroyWindow(self.ctrl_window)


if __name__ == "__main__":
    rospy.init_node("vision_threshold_tuner", anonymous=False)
    node = VisionThresholdTuner()
    node.spin()
