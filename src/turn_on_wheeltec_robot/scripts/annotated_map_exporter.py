#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""在节点退出时保存地图，并叠加二维码/娃娃标记和时间戳，导出GeoTIFF。"""

import datetime
import json
import os
import subprocess

import cv2
import numpy as np
import rospy
import tf.transformations
from nav_msgs.msg import OccupancyGrid

try:
    import yaml
except Exception:
    yaml = None


class AnnotatedMapExporter(object):
    def __init__(self):
        self.output_dir = rospy.get_param("~output_dir", os.path.expanduser("~/.ros/auto2d_maps"))
        self.map_topic = rospy.get_param("~map_topic", "map")
        self.map_name_prefix = rospy.get_param("~map_name_prefix", "auto2d_map")

        self.qr_landmark_file = rospy.get_param("~qr_landmark_file", os.path.expanduser("~/.ros/qr_landmarks.json"))
        self.doll_landmark_file = rospy.get_param("~doll_landmark_file", os.path.expanduser("~/.ros/doll_landmarks.json"))

        self.qr_color_bgr = tuple(rospy.get_param("~qr_color_bgr", [0, 0, 255]))      # 红色
        self.doll_color_bgr = tuple(rospy.get_param("~doll_color_bgr", [255, 0, 0]))  # 蓝色

        self.circle_radius = int(rospy.get_param("~circle_radius_px", 8))
        self.circle_thickness = int(rospy.get_param("~circle_thickness_px", 2))
        self.text_scale = float(rospy.get_param("~text_scale", 0.5))
        self.text_thickness = int(rospy.get_param("~text_thickness", 1))
        self.trigger_on_shutdown = bool(rospy.get_param("~trigger_on_shutdown", True))
        self.occupied_thresh = float(rospy.get_param("~occupied_thresh", 0.65))
        self.free_thresh = float(rospy.get_param("~free_thresh", 0.196))

        self._current_map = None
        self._saved_once = False

        rospy.Subscriber(self.map_topic, OccupancyGrid, self._map_callback, queue_size=1)
        rospy.on_shutdown(self._on_shutdown)

        rospy.loginfo("[annotated_map_exporter] ready, output_dir=%s", self.output_dir)

    def _ensure_dir(self, path):
        if not os.path.exists(path):
            os.makedirs(path)

    def _map_callback(self, msg):
        self._current_map = msg

    def _save_map_locally(self, msg, base_path):
        """替代 map_saver，直接从内存保存地图到磁盘。"""
        try:
            w = msg.info.width
            h = msg.info.height
            if w <= 0 or h <= 0:
                return False

            # 数据处理：ROS地图是1D数组，值范围[-1, 100]
            data = np.array(msg.data, dtype=np.int8).reshape((h, w))
            # ROS坐标原点在左下角，PGM图像原点在左上角，需要上下翻转
            data = np.flipud(data)

            # 模拟 map_saver 的 trinary 逻辑
            img = np.zeros((h, w), dtype=np.uint8)
            img[data == -1] = 205                                     # 未知
            img[data >= self.occupied_thresh * 100] = 0               # 占用
            img[(data <= self.free_thresh * 100) & (data >= 0)] = 254 # 空闲
            img[(data > self.free_thresh * 100) & (data < self.occupied_thresh * 100)] = 205

            pgm_path = base_path + ".pgm"
            cv2.imwrite(pgm_path, img)

            # 提取 Yaw 角
            q = [
                msg.info.origin.orientation.x,
                msg.info.origin.orientation.y,
                msg.info.origin.orientation.z,
                msg.info.origin.orientation.w
            ]
            _, _, yaw = tf.transformations.euler_from_quaternion(q)

            yaml_path = base_path + ".yaml"
            with open(yaml_path, "w") as f:
                f.write("image: %s\n" % os.path.basename(pgm_path))
                f.write("resolution: %f\n" % msg.info.resolution)
                f.write("origin: [%f, %f, %f]\n" % (
                    msg.info.origin.position.x,
                    msg.info.origin.position.y,
                    yaw
                ))
                f.write("negate: 0\n")
                f.write("occupied_thresh: %f\n" % self.occupied_thresh)
                f.write("free_thresh: %f\n" % self.free_thresh)

            return True
        except Exception as exc:
            rospy.logerr("[annotated_map_exporter] local map save failed: %s", str(exc))
            return False

    def _run_map_saver(self, base_path):
        cmd = ["rosrun", "map_server", "map_saver", "-f", base_path, "map:=%s" % self.map_topic]
        rospy.loginfo("[annotated_map_exporter] running: %s", " ".join(cmd))
        try:
            ret = subprocess.call(cmd)
            return ret == 0
        except Exception as exc:
            rospy.logerr("[annotated_map_exporter] map_saver failed: %s", str(exc))
            return False

    def _load_yaml(self, yaml_path):
        if yaml is not None:
            with open(yaml_path, "r") as f:
                return yaml.safe_load(f)

        # 无PyYAML时的简易解析器
        data = {}
        with open(yaml_path, "r") as f:
            lines = f.readlines()
        for line in lines:
            line = line.strip()
            if not line or line.startswith("#") or ":" not in line:
                continue
            key, val = line.split(":", 1)
            key = key.strip()
            val = val.strip().strip("'").strip('"')
            if key == "origin":
                val = val.strip("[]")
                parts = [p.strip() for p in val.split(",")]
                data[key] = [float(parts[0]), float(parts[1]), float(parts[2])]
            elif key in ("resolution", "occupied_thresh", "free_thresh"):
                data[key] = float(val)
            elif key == "negate":
                data[key] = int(val)
            else:
                data[key] = val
        return data

    def _load_landmarks(self, file_path, fallback_label):
        if not os.path.exists(file_path):
            return []
        try:
            with open(file_path, "r") as f:
                data = json.load(f)
        except Exception as exc:
            rospy.logwarn("[annotated_map_exporter] load landmark failed %s: %s", file_path, str(exc))
            return []

        out = []
        for key, item in data.items():
            try:
                out.append({
                    "label": str(item.get("label", fallback_label if fallback_label else key)),
                    "x": float(item["x"]),
                    "y": float(item["y"]),
                })
            except Exception:
                continue
        return out

    def _map_to_pixel(self, x, y, origin_x, origin_y, resolution, image_h):
        px = int((x - origin_x) / resolution)
        py = image_h - 1 - int((y - origin_y) / resolution)
        return px, py

    def _draw_landmarks(self, image_bgr, landmarks, color_bgr, origin_x, origin_y, resolution):
        h, w = image_bgr.shape[:2]
        for item in landmarks:
            px, py = self._map_to_pixel(item["x"], item["y"], origin_x, origin_y, resolution, h)
            if px < 0 or py < 0 or px >= w or py >= h:
                continue
            cv2.circle(image_bgr, (px, py), self.circle_radius, color_bgr, self.circle_thickness)
            cv2.putText(
                image_bgr,
                item["label"],
                (px + 6, max(0, py - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                self.text_scale,
                color_bgr,
                self.text_thickness,
            )

    def _export_geotiff(self, image_bgr, geotiff_path, origin_x, origin_y, resolution):
        h, w = image_bgr.shape[:2]

        # 优先使用GDAL输出严格GeoTIFF
        try:
            from osgeo import gdal

            driver = gdal.GetDriverByName("GTiff")
            ds = driver.Create(geotiff_path, w, h, 3, gdal.GDT_Byte)
            if ds is None:
                raise RuntimeError("cannot create GTiff dataset")

            # map_server地图坐标原点在左下角，影像坐标原点在左上角
            gt = [origin_x, resolution, 0.0, origin_y + h * resolution, 0.0, -resolution]
            ds.SetGeoTransform(gt)

            rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
            ds.GetRasterBand(1).WriteArray(rgb[:, :, 0])
            ds.GetRasterBand(2).WriteArray(rgb[:, :, 1])
            ds.GetRasterBand(3).WriteArray(rgb[:, :, 2])
            ds.FlushCache()
            ds = None
            rospy.loginfo("[annotated_map_exporter] GeoTIFF exported via GDAL: %s", geotiff_path)
            return True
        except Exception as exc:
            rospy.logwarn("[annotated_map_exporter] GDAL unavailable, fallback TIFF+TFW: %s", str(exc))

        # 降级：普通TIFF + world file
        if not cv2.imwrite(geotiff_path, image_bgr):
            rospy.logerr("[annotated_map_exporter] write TIFF failed: %s", geotiff_path)
            return False

        tfw_path = os.path.splitext(geotiff_path)[0] + ".tfw"
        x_center = origin_x + 0.5 * resolution
        y_center = origin_y + (h - 0.5) * resolution
        with open(tfw_path, "w") as f:
            f.write("%.12f\n" % resolution)
            f.write("0.0\n")
            f.write("0.0\n")
            f.write("%.12f\n" % (-resolution))
            f.write("%.12f\n" % x_center)
            f.write("%.12f\n" % y_center)

        rospy.loginfo("[annotated_map_exporter] TIFF+TFW exported: %s , %s", geotiff_path, tfw_path)
        return True

    def save_all(self):
        if self._saved_once:
            return
        self._saved_once = True

        self._ensure_dir(self.output_dir)
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        base_path = os.path.join(self.output_dir, "%s_%s" % (self.map_name_prefix, stamp))
        yaml_path = base_path + ".yaml"

        success = False
        if self._current_map is not None:
            rospy.loginfo("[annotated_map_exporter] saving map from memory...")
            success = self._save_map_locally(self._current_map, base_path)
        else:
            rospy.logwarn("[annotated_map_exporter] no map in memory, trying map_saver as fallback...")
            success = self._run_map_saver(base_path)

        if not success:
            rospy.logerr("[annotated_map_exporter] map save failed")
            return
        if not os.path.exists(yaml_path):
            rospy.logerr("[annotated_map_exporter] yaml not found after save: %s", yaml_path)
            return

        map_info = self._load_yaml(yaml_path)
        map_img_rel = map_info.get("image", os.path.basename(base_path) + ".pgm")
        map_img_path = map_img_rel
        if not os.path.isabs(map_img_rel):
            map_img_path = os.path.join(os.path.dirname(yaml_path), map_img_rel)

        image_gray = cv2.imread(map_img_path, cv2.IMREAD_GRAYSCALE)
        if image_gray is None:
            rospy.logerr("[annotated_map_exporter] map image read failed: %s", map_img_path)
            return
        image_bgr = cv2.cvtColor(image_gray, cv2.COLOR_GRAY2BGR)

        resolution = float(map_info["resolution"])
        origin_x = float(map_info["origin"][0])
        origin_y = float(map_info["origin"][1])

        qr_landmarks = self._load_landmarks(self.qr_landmark_file, "qr")
        doll_landmarks = self._load_landmarks(self.doll_landmark_file, "doll")

        self._draw_landmarks(image_bgr, qr_landmarks, self.qr_color_bgr, origin_x, origin_y, resolution)
        self._draw_landmarks(image_bgr, doll_landmarks, self.doll_color_bgr, origin_x, origin_y, resolution)

        # 左上角打时间标记
        time_text = "Saved: %s" % datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        cv2.putText(image_bgr, time_text, (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (30, 30, 30), 3)
        cv2.putText(image_bgr, time_text, (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        annotated_png = base_path + "_annotated.png"
        cv2.imwrite(annotated_png, image_bgr)

        geotiff_path = base_path + "_annotated.tif"
        self._export_geotiff(image_bgr, geotiff_path, origin_x, origin_y, resolution)

        rospy.loginfo("[annotated_map_exporter] done. base=%s", base_path)
        rospy.loginfo("[annotated_map_exporter] QR landmarks=%d, Doll landmarks=%d", len(qr_landmarks), len(doll_landmarks))

    def _on_shutdown(self):
        if not self.trigger_on_shutdown:
            return
        rospy.loginfo("[annotated_map_exporter] shutdown triggered, saving map...")
        self.save_all()


def main():
    rospy.init_node("annotated_map_exporter", anonymous=False)
    exporter = AnnotatedMapExporter()
    rospy.spin()
    # 多数情况下spin结束后已触发shutdown回调，这里做兜底。
    exporter.save_all()


if __name__ == "__main__":
    main()
