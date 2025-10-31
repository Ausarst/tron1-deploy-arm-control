#!/usr/bin/env python3
import rospy
import tf2_ros
import tf2_sensor_msgs.tf2_sensor_msgs as tf2_sm
from sensor_msgs.msg import PointCloud2

class DynamicPointCloudTransformer:
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.sub = rospy.Subscriber("/camera0/depth/color/points", PointCloud2, self.callback, queue_size=1)
        self.pub = rospy.Publisher("/points_in_ankle", PointCloud2, queue_size=1)

    def get_lower_ankle(self, stamp):
        try:
            ankle_L = self.tf_buffer.lookup_transform("base_Link", "ankle_L_Link", rospy.Time(0), rospy.Duration(0.05))
            ankle_R = self.tf_buffer.lookup_transform("base_Link", "ankle_R_Link", rospy.Time(0), rospy.Duration(0.05))

            zL = ankle_L.transform.translation.z
            zR = ankle_R.transform.translation.z

            if zL <= zR:
                return "ankle_L_Link"
            else:
                return "ankle_R_Link"
        except Exception as e:
            rospy.logwarn("TF lookup failed: %s" % e)
            return None

    def callback(self, msg):
        # 保存原始点云时间戳
        orig_stamp = msg.header.stamp
        # 覆盖成当前时间，避免 extrapolation
        msg.header.stamp = rospy.Time.now()
        
        # 动态选择目标脚踝
        target_frame = self.get_lower_ankle(msg.header.stamp)
        if target_frame is None:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,          # 目标坐标系（低脚）
                msg.header.frame_id,   # 点云原始坐标系（相机）
                rospy.Time(0), 
                rospy.Duration(0.1)
            )
            cloud_out = tf2_sm.do_transform_cloud(msg, transform)
            cloud_out.header.frame_id = target_frame
            self.pub.publish(cloud_out)

            rospy.loginfo_throttle(1.0, f"PointCloud transformed to {target_frame}")
        except Exception as e:
            rospy.logwarn("PointCloud transform failed: %s" % e)

if __name__ == "__main__":
    rospy.init_node("dynamic_pointcloud_to_ankle")
    node = DynamicPointCloudTransformer()
    rospy.loginfo("Dynamic PointCloud Transformer started")
    rospy.spin()
