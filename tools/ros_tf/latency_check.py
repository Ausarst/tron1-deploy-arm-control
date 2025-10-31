#!/usr/bin/env python3
import rospy
import tf2_ros
from sensor_msgs.msg import JointState

class LatencyChecker:
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # 订阅关节角
        rospy.Subscriber("/joint_states", JointState, self.joint_cb)

        # 足底 link 名称（根据 URDF 修改）
        self.base_frame = "base_Link"
        self.foot_frame = "ankle_L_Link" 

    def joint_cb(self, msg: JointState):
        now = rospy.Time.now()
        joint_stamp = msg.header.stamp

        # 计算 joint_states 的延迟
        joint_delay = (now - joint_stamp).to_sec() * 1000.0

        try:
            # 查询 TF：足底相对 base
            trans = self.tf_buffer.lookup_transform(
                self.base_frame, self.foot_frame, rospy.Time(0), rospy.Duration(0.05)
            )
            tf_stamp = trans.header.stamp
            tf_delay = (now - tf_stamp).to_sec() * 1000.0

            print(f"[LatencyCheck] now={now.to_sec():.3f} "
                  f"joint_stamp={joint_stamp.to_sec():.3f} delay={joint_delay:.2f} ms | "
                  f"tf_stamp={tf_stamp.to_sec():.3f} delay={tf_delay:.2f} ms | "
                  f"foot_pos=({trans.transform.translation.x:.3f}, "
                  f"{trans.transform.translation.y:.3f}, "
                  f"{trans.transform.translation.z:.3f})")

        except Exception as e:
            rospy.logwarn_throttle(5, f"TF lookup failed: {e}")

if __name__ == "__main__":
    rospy.init_node("latency_checker")
    LatencyChecker()
    rospy.spin()
