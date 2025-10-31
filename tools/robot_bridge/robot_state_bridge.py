#!/usr/bin/env python3
import sys
import rospy
from sensor_msgs.msg import JointState
from functools import partial
import limxsdk.robot.Robot as Robot
import limxsdk.robot.RobotType as RobotType
import limxsdk.datatypes as datatypes

class RobotStateBridge:
    def __init__(self, joint_names):
        self.joint_names = joint_names
        self.pub = rospy.Publisher("/joint_states", JointState, queue_size=10)

    def robot_state_callback(self, robot_state: datatypes.RobotState):
        msg = JointState()

        # 时间戳：SDK 的 stamp 是纳秒整数，转成 ROS Time
        if hasattr(robot_state, "stamp") and robot_state.stamp > 0:
            msg.header.stamp = rospy.Time.from_sec(robot_state.stamp * 1e-9)
        else:
            msg.header.stamp = rospy.Time.now()

        # 保证长度一致：截断或补零到和 joint_names 一样长
        n = len(self.joint_names)
        msg.name     = self.joint_names
        msg.position = list(robot_state.q[:n]) if len(robot_state.q) >= n else list(robot_state.q) + [0.0]*(n-len(robot_state.q))
        msg.velocity = list(robot_state.dq[:n]) if len(robot_state.dq) >= n else list(robot_state.dq) + [0.0]*(n-len(robot_state.dq))
        msg.effort   = list(robot_state.tau[:n]) if len(robot_state.tau) >= n else list(robot_state.tau) + [0.0]*(n-len(robot_state.tau))

        self.pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node("robot_state_bridge")

    joint_names = [
        "abad_L_Joint", "hip_L_Joint", "knee_L_Joint", "ankle_L_Joint",
        "abad_R_Joint", "hip_R_Joint", "knee_R_Joint", "ankle_R_Joint"
    ]

    # 初始化 SDK
    robot = Robot(RobotType.PointFoot)
    robot_ip = "10.192.1.2"
    if len(sys.argv) > 1:
        robot_ip = sys.argv[1]

    if not robot.init(robot_ip):
        rospy.logerr("Failed to init robot at IP %s" % robot_ip)
        sys.exit(1)

    bridge = RobotStateBridge(joint_names)
    cb = partial(bridge.robot_state_callback)
    robot.subscribeRobotState(cb)

    rospy.loginfo("RobotStateBridge started, publishing /joint_states")
    rospy.spin()

