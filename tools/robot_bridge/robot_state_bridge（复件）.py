import rospy
from sensor_msgs.msg import JointState
from functools import partial
import limxsdk.robot.Robot as Robot
import limxsdk.robot.RobotType as RobotType
import limxsdk.datatypes as datatypes

class RobotReceiver:
    def __init__(self, joint_names):
        self.joint_names = joint_names
        self.pub = rospy.Publisher("/joint_states", JointState, queue_size=10)

    def robotStateCallback(self, robot_state: datatypes.RobotState):
        msg = JointState()
        # SDK 的 stamp 是纳秒整数，需要转成 ROS Time
        msg.header.stamp = rospy.Time.from_sec(robot_state.stamp * 1e-9)
        msg.name = self.joint_names
        msg.position = list(robot_state.q)
        msg.velocity = list(robot_state.dq)
        msg.effort   = list(robot_state.tau)
        self.pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node("robot_state_bridge")

    # 根据机器人类型选择关节名顺序
    joint_names = [
        "abad_L_Joint", "hip_L_Joint", "knee_L_Joint",
        "abad_R_Joint", "hip_R_Joint", "knee_R_Joint"
    ]

    robot = Robot(RobotType.PointFoot)
    if not robot.init("10.192.1.2"):
        sys.exit()

    receiver = RobotReceiver(joint_names)
    cb = partial(receiver.robotStateCallback)
    robot.subscribeRobotState(cb)

    rospy.spin()
