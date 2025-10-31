#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2

def cb(msg):
    now = rospy.Time.now()
    stamp = msg.header.stamp
    diff = (now - stamp).to_sec()
    print(f"PointCloud stamp: {stamp.to_sec():.6f}, Now: {now.to_sec():.6f}, Δ={diff:.3f}s")

rospy.init_node("stamp_checker")
rospy.Subscriber("/camera0/depth/color/points", PointCloud2, cb, queue_size=1)
rospy.spin()
