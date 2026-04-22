#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""[auto2d-v0.1.0][2026-04-14] 自动发布RRT探索初始化点。"""

import rospy
from geometry_msgs.msg import PointStamped


def _build_point(frame_id, x, y):
    point = PointStamped()
    point.header.frame_id = frame_id
    point.header.stamp = rospy.Time.now()
    point.point.x = x
    point.point.y = y
    point.point.z = 0.0
    return point


def main():
    rospy.init_node("auto_init_explore")

    # [auto2d-v0.1.0][2026-04-14] 启动与发布节奏参数
    startup_delay = rospy.get_param("~startup_delay", 8.0)
    min_subscribers = rospy.get_param("~min_subscribers", 2)
    point_interval = rospy.get_param("~point_interval", 0.3)
    publish_rounds = rospy.get_param("~publish_rounds", 2)

    # [auto2d-v0.1.0][2026-04-14] 探索区域与起点参数
    frame_id = rospy.get_param("~frame_id", "map")
    min_x = rospy.get_param("~min_x", -4.0)
    max_x = rospy.get_param("~max_x", 4.0)
    min_y = rospy.get_param("~min_y", -4.0)
    max_y = rospy.get_param("~max_y", 4.0)
    start_x = rospy.get_param("~start_x", 0.0)
    start_y = rospy.get_param("~start_y", 0.0)

    pub = rospy.Publisher("/clicked_point", PointStamped, queue_size=20)

    rospy.loginfo(
        "[auto2d-v0.1.0] auto_init_explore start, delay=%.2fs, min_subscribers=%d",
        startup_delay,
        min_subscribers,
    )
    rospy.sleep(startup_delay)

    wait_rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if pub.get_num_connections() >= min_subscribers:
            break
        wait_rate.sleep()

    # 注意：rrt_exploration约定前4个点为边界，第5个点为生长树起点
    points = [
        _build_point(frame_id, min_x, min_y),
        _build_point(frame_id, min_x, max_y),
        _build_point(frame_id, max_x, max_y),
        _build_point(frame_id, max_x, min_y),
        _build_point(frame_id, start_x, start_y),
    ]

    for _ in range(max(1, int(publish_rounds))):
        for point in points:
            if rospy.is_shutdown():
                return
            point.header.stamp = rospy.Time.now()
            pub.publish(point)
            rospy.sleep(point_interval)

    rospy.loginfo(
        "[auto2d-v0.1.0] clicked_point initialized: [%.2f, %.2f]~[%.2f, %.2f], start=(%.2f, %.2f)",
        min_x,
        min_y,
        max_x,
        max_y,
        start_x,
        start_y,
    )


if __name__ == "__main__":
    main()
